#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <cmath>
#include <chrono>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <opencv2/opencv.hpp>

// Constants for visualization map geometry and herding logic
constexpr double MAP_HALF_WIDTH = 10.0; // Displays 20m x 20m map (from -10m to +10m on X and Y)
constexpr int GRID_SIZE = 40;          // Grid resolution N x N cells
constexpr int RELAXATION_ITERATIONS = 100;

// Herding Constants
const cv::Point2f GOAL_POSITION(10.0f, 10.0f);
constexpr double COW_EXCLUSION_RADIUS = 2.0;   // meters
constexpr double PUSH_POINT_MARGIN = 0.2;       // meters
constexpr double MAX_GOTO_MOVEMENT = 0.5;       // meters

class OccupancyGridVisualizer : public rclcpp::Node
{
public:
  OccupancyGridVisualizer()
  : Node("occupancy_grid_visualizer")
  {
    // Declare and get namespace parameter
    this->declare_parameter<std::string>("name", "");
    this->declare_parameter<std::string>("namespace", "");

    std::string name_param = this->get_parameter("name").as_string();
    if (name_param.empty()) {
      name_param = this->get_parameter("namespace").as_string();
    }

    // Construct topic names dynamically
    std::string drone_gt_topic = name_param.empty() ? "gt_pose" : (name_param.front() == '/' ? name_param + "/gt_pose" : "/" + name_param + "/gt_pose");
    std::string goto_topic     = name_param.empty() ? "goto"    : (name_param.front() == '/' ? name_param + "/goto"    : "/" + name_param + "/goto");
    std::string focus_topic    = name_param.empty() ? "focusin" : (name_param.front() == '/' ? name_param + "/focusin" : "/" + name_param + "/focusin");

    RCLCPP_INFO(this->get_logger(), "Drone Pose Sub Topic: %s", drone_gt_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Cows Pose Sub Topic: /cows_pos");
    RCLCPP_INFO(this->get_logger(), "Drone Goto Pub Topic: %s", goto_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Drone Focus Pub Topic: %s", focus_topic.c_str());

    // Initialize subscriptions
    drone_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      drone_gt_topic,
      10,
      std::bind(&OccupancyGridVisualizer::drone_callback, this, std::placeholders::_1)
    );

    cow_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/cows_pos",
      10,
      std::bind(&OccupancyGridVisualizer::cow_callback, this, std::placeholders::_1)
    );

    // Initialize publishers
    goto_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(goto_topic, 10);
    focusin_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(focus_topic, 10);

    // Initialize 5Hz visualization timer (200ms)
    timer_5hz_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&OccupancyGridVisualizer::timer_5hz_callback, this)
    );

    // Initialize 2Hz movement control timer (500ms)
    timer_2hz_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&OccupancyGridVisualizer::timer_2hz_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Node initialized with 5Hz UI and 2Hz Control Timers.");
  }

private:
  enum CellState {
    EMPTY = 0,
    DRONE = 1,
    COW = 2,
    EXCLUSION = 3,
    PUSH_OBJECTIVE = 4
  };

  void drone_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_drone_pose_ = msg;
    received_drone_ = true;
  }

  void cow_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cows_ = msg;
    received_cows_ = true;
  }

  // -------------------------------------------------------------
  // HELPER MAPPING FUNCTIONS
  // -------------------------------------------------------------

  static bool map_to_grid(double x, double y, int& row, int& col)
  {
    if (x < -MAP_HALF_WIDTH || x > MAP_HALF_WIDTH || y < -MAP_HALF_WIDTH || y > MAP_HALF_WIDTH) {
      return false;
    }
    double norm_x = (x + MAP_HALF_WIDTH) / (2.0 * MAP_HALF_WIDTH);
    double norm_y = (MAP_HALF_WIDTH - y) / (2.0 * MAP_HALF_WIDTH);

    col = static_cast<int>(std::floor(norm_x * GRID_SIZE));
    row = static_cast<int>(std::floor(norm_y * GRID_SIZE));

    col = std::max(0, std::min(GRID_SIZE - 1, col));
    row = std::max(0, std::min(GRID_SIZE - 1, row));
    return true;
  }

  static cv::Point2f grid_to_world(int row, int col)
  {
    float norm_x = (col + 0.5f) / GRID_SIZE;
    float norm_y = (row + 0.5f) / GRID_SIZE;

    float x = norm_x * (2.0f * MAP_HALF_WIDTH) - MAP_HALF_WIDTH;
    float y = MAP_HALF_WIDTH - norm_y * (2.0f * MAP_HALF_WIDTH);
    return cv::Point2f(x, y);
  }

  // -------------------------------------------------------------
  // 5Hz TIMER CALLBACK (SEQUENTIAL MODULAR RENDERING)
  // -------------------------------------------------------------

  void timer_5hz_callback()
  {
    geometry_msgs::msg::Pose::SharedPtr drone_pose;
    geometry_msgs::msg::PoseArray::SharedPtr cows;
    bool has_drone = false;
    bool has_cows = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      drone_pose = latest_drone_pose_;
      cows = latest_cows_;
      has_drone = received_drone_;
      has_cows = received_cows_;
    }

    // 1. Compute state grids
    std::vector<std::vector<CellState>> grid(GRID_SIZE, std::vector<CellState>(GRID_SIZE, EMPTY));
    cv::Mat potential_grid(GRID_SIZE, GRID_SIZE, CV_32F, cv::Scalar(0.5f));
    cv::Mat is_fixed(GRID_SIZE, GRID_SIZE, CV_8UC1, cv::Scalar(0));

    // Circular Cow Exclusion & Pushing Objective Setup
    if (has_cows && cows) {
      for (const auto& cow_pose : cows->poses) {
        float cow_x = cow_pose.position.x;
        float cow_y = cow_pose.position.y;

        int c_row = 0, c_col = 0;
        if (map_to_grid(cow_x, cow_y, c_row, c_col)) {
          grid[c_row][c_col] = COW;
        }

        // Circular Exclusion Radius (cow_exclusion_radius = 2.0m)
        for (int r = 0; r < GRID_SIZE; ++r) {
          for (int c = 0; c < GRID_SIZE; ++c) {
            cv::Point2f cell_w = grid_to_world(r, c);
            double dist = std::hypot(cell_w.x - cow_x, cell_w.y - cow_y);
            if (dist <= COW_EXCLUSION_RADIUS) {
              if (grid[r][c] != COW) {
                grid[r][c] = EXCLUSION;
              }
              potential_grid.at<float>(r, c) = 1.0f;
              is_fixed.at<uchar>(r, c) = 1;
            }
          }
        }

        // Dynamic Drone Objectives (Pushing Points)
        cv::Point2f cow_pt(cow_x, cow_y);
        cv::Point2f dir_to_goal = GOAL_POSITION - cow_pt;
        float goal_dist = std::sqrt(dir_to_goal.x * dir_to_goal.x + dir_to_goal.y * dir_to_goal.y);

        if (goal_dist > 1e-4f) {
          cv::Point2f norm_dir(dir_to_goal.x / goal_dist, dir_to_goal.y / goal_dist);
          // Push point placed opposite to goal direction
          cv::Point2f push_pt = cow_pt - norm_dir * (COW_EXCLUSION_RADIUS + PUSH_POINT_MARGIN);

          int p_row = 0, p_col = 0;
          if (map_to_grid(push_pt.x, push_pt.y, p_row, p_col)) {
            grid[p_row][p_col] = PUSH_OBJECTIVE;
            potential_grid.at<float>(p_row, p_col) = 0.0f;
            is_fixed.at<uchar>(p_row, p_col) = 1;
          }
        }
      }
    }

    // Set drone cell state
    if (has_drone && drone_pose) {
      int d_row = 0, d_col = 0;
      if (map_to_grid(drone_pose->position.x, drone_pose->position.y, d_row, d_col)) {
        grid[d_row][d_col] = DRONE;
      }
    }

    // Outer Perimeter boundaries set to 1.0 (fixed)
    for (int c = 0; c < GRID_SIZE; ++c) {
      potential_grid.at<float>(0, c) = 1.0f;
      potential_grid.at<float>(GRID_SIZE - 1, c) = 1.0f;
      is_fixed.at<uchar>(0, c) = 1;
      is_fixed.at<uchar>(GRID_SIZE - 1, c) = 1;
    }
    for (int r = 0; r < GRID_SIZE; ++r) {
      potential_grid.at<float>(r, 0) = 1.0f;
      potential_grid.at<float>(r, GRID_SIZE - 1) = 1.0f;
      is_fixed.at<uchar>(r, 0) = 1;
      is_fixed.at<uchar>(r, GRID_SIZE - 1) = 1;
    }

    // Laplace Potential Field Relaxation Loop (100 Iterations)
    for (int iter = 0; iter < RELAXATION_ITERATIONS; ++iter) {
      cv::Mat copy_grid = potential_grid.clone();
      for (int r = 1; r < GRID_SIZE - 1; ++r) {
        for (int c = 1; c < GRID_SIZE - 1; ++c) {
          if (is_fixed.at<uchar>(r, c) == 1) {
            continue;
          }
          float left  = copy_grid.at<float>(r, c - 1);
          float right = copy_grid.at<float>(r, c + 1);
          float up    = copy_grid.at<float>(r - 1, c);
          float down  = copy_grid.at<float>(r + 1, c);

          potential_grid.at<float>(r, c) = (left + right + up + down) / 4.0f;
        }
      }
    }

    // Vector Grid Computation (Gradient)
    std::vector<std::vector<cv::Point2f>> vector_grid(GRID_SIZE, std::vector<cv::Point2f>(GRID_SIZE, cv::Point2f(0.0f, 0.0f)));
    for (int r = 1; r < GRID_SIZE - 1; ++r) {
      for (int c = 1; c < GRID_SIZE - 1; ++c) {
        if (is_fixed.at<uchar>(r, c) == 1) {
          continue;
        }
        float left  = potential_grid.at<float>(r, c - 1);
        float right = potential_grid.at<float>(r, c + 1);
        float up    = potential_grid.at<float>(r - 1, c);
        float down  = potential_grid.at<float>(r + 1, c);

        // vector_x: left - right (positive moves right / +X)
        // vector_y: down - up (positive moves up / +Y in world coordinates, because row index r decreases upward in world frame)
        vector_grid[r][c] = cv::Point2f(left - right, down - up);
      }
    }

    // Store vector grid safely for 2Hz controller thread access
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_vector_grid_ = vector_grid;
      has_vector_grid_ = true;
    }

    // Render Displays (Modular Functions)
    compute_and_display_occupancy_map(grid);
    compute_and_display_vector_grid(grid, vector_grid, is_fixed);
  }

  // -------------------------------------------------------------
  // MODULAR FUNCTION 1: OCCUPANCY GRID RENDERING
  // -------------------------------------------------------------

  void compute_and_display_occupancy_map(const std::vector<std::vector<CellState>>& grid)
  {
    cv::Mat display_img(GRID_SIZE, GRID_SIZE, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int r = 0; r < GRID_SIZE; ++r) {
      for (int c = 0; c < GRID_SIZE; ++c) {
        if (grid[r][c] == DRONE) {
          display_img.at<cv::Vec3b>(r, c) = cv::Vec3b(255, 0, 0);     // Blue
        } else if (grid[r][c] == COW) {
          display_img.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 255);     // Red
        } else if (grid[r][c] == EXCLUSION) {
          display_img.at<cv::Vec3b>(r, c) = cv::Vec3b(50, 50, 50);    // Dark Gray Exclusion
        } else if (grid[r][c] == PUSH_OBJECTIVE) {
          display_img.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 255, 0);     // Green Objective
        }
      }
    }

    cv::Mat enlarged_img;
    cv::resize(display_img, enlarged_img, cv::Size(400, 400), 0, 0, cv::INTER_NEAREST);
    cv::imshow("Occupancy Grid", enlarged_img);
    cv::waitKey(1);
  }

  // -------------------------------------------------------------
  // MODULAR FUNCTION 2: VECTOR GRID RENDERING
  // -------------------------------------------------------------

  void compute_and_display_vector_grid(
    const std::vector<std::vector<CellState>>& grid,
    const std::vector<std::vector<cv::Point2f>>& vector_grid,
    const cv::Mat& is_fixed)
  {
    const int CANVAS_SIZE = 600;
    const float cell_size = static_cast<float>(CANVAS_SIZE) / GRID_SIZE;

    cv::Mat vector_img(CANVAS_SIZE, CANVAS_SIZE, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int r = 0; r < GRID_SIZE; ++r) {
      for (int c = 0; c < GRID_SIZE; ++c) {
        float center_x = (c + 0.5f) * cell_size;
        float center_y = (r + 0.5f) * cell_size;
        cv::Point2f center(center_x, center_y);

        if (grid[r][c] == COW) {
          cv::rectangle(vector_img, cv::Point(c * cell_size, r * cell_size), cv::Point((c + 1) * cell_size, (r + 1) * cell_size), cv::Scalar(0, 0, 255), cv::FILLED);
          continue;
        } else if (grid[r][c] == DRONE) {
          cv::rectangle(vector_img, cv::Point(c * cell_size, r * cell_size), cv::Point((c + 1) * cell_size, (r + 1) * cell_size), cv::Scalar(255, 0, 0), cv::FILLED);
        } else if (grid[r][c] == PUSH_OBJECTIVE) {
          cv::rectangle(vector_img, cv::Point(c * cell_size, r * cell_size), cv::Point((c + 1) * cell_size, (r + 1) * cell_size), cv::Scalar(0, 255, 0), cv::FILLED);
        }

        cv::Point2f vec = vector_grid[r][c];
        float mag = std::sqrt(vec.x * vec.x + vec.y * vec.y);

        if (mag > 1e-4f && is_fixed.at<uchar>(r, c) == 0) {
          float max_arrow_len = cell_size * 0.8f;
          float arrow_scale = std::min(mag * 5.0f, 1.0f) * (max_arrow_len / 2.0f);

          cv::Point2f dir(vec.x / mag, -vec.y / mag);
          cv::Point2f start_pt = center - dir * arrow_scale;
          cv::Point2f end_pt   = center + dir * arrow_scale;

          cv::arrowedLine(vector_img, start_pt, end_pt, cv::Scalar(50, 50, 50), 1, cv::LINE_AA, 0, 0.3);
        }
      }
    }

    cv::imshow("Vector Movement Grid", vector_img);
    cv::waitKey(1);
  }

  // -------------------------------------------------------------
  // 2Hz TIMER CALLBACK (MOVEMENT & FOCUS CONTROL)
  // -------------------------------------------------------------

  void timer_2hz_callback()
  {
    geometry_msgs::msg::Pose::SharedPtr drone_pose;
    geometry_msgs::msg::PoseArray::SharedPtr cows;
    std::vector<std::vector<cv::Point2f>> vector_grid;
    bool has_drone = false;
    bool has_cows = false;
    bool has_grid = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      drone_pose = latest_drone_pose_;
      cows = latest_cows_;
      vector_grid = latest_vector_grid_;
      has_drone = received_drone_;
      has_cows = received_cows_;
      has_grid = has_vector_grid_;
    }

    if (!has_drone || !drone_pose) {
      return;
    }

    double drone_x = drone_pose->position.x;
    double drone_y = drone_pose->position.y;

    // 1. Goto Calculation
    if (has_grid) {
      int d_row = 0, d_col = 0;
      if (map_to_grid(drone_x, drone_y, d_row, d_col)) {
        cv::Point2f vec = vector_grid[d_row][d_col];
        float mag = std::sqrt(vec.x * vec.x + vec.y * vec.y);

        cv::Point2f move_vec(0.0f, 0.0f);
        if (mag > 1e-4f) {
          // Normalize vector to magnitude 1.0 (unit vector) so drone always moves at maximum speed (MAX_GOTO_MOVEMENT = 0.5m)
          move_vec = cv::Point2f((vec.x / mag) * static_cast<float>(MAX_GOTO_MOVEMENT),
                                 (vec.y / mag) * static_cast<float>(MAX_GOTO_MOVEMENT));
        }

        geometry_msgs::msg::Pose goto_msg;
        goto_msg.position.x = drone_x + move_vec.x;
        goto_msg.position.y = drone_y + move_vec.y;
        goto_msg.position.z = drone_pose->position.z;
        goto_msg.orientation = drone_pose->orientation;

        goto_pub_->publish(goto_msg);
      }
    }

    // 2. Focus Calculation (Closest Cow)
    if (has_cows && cows && !cows->poses.empty()) {
      double min_dist = std::numeric_limits<double>::max();
      geometry_msgs::msg::Pose closest_cow_pose = cows->poses[0];

      for (const auto& cow_pose : cows->poses) {
        double dist = std::hypot(cow_pose.position.x - drone_x, cow_pose.position.y - drone_y);
        if (dist < min_dist) {
          min_dist = dist;
          closest_cow_pose = cow_pose;
        }
      }

      focusin_pub_->publish(closest_cow_pose);
    }
  }

  // Subscriptions, Publishers, Timers & Mutex
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr drone_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cow_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goto_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr focusin_pub_;

  rclcpp::TimerBase::SharedPtr timer_5hz_;
  rclcpp::TimerBase::SharedPtr timer_2hz_;

  std::mutex mutex_;
  geometry_msgs::msg::Pose::SharedPtr latest_drone_pose_;
  geometry_msgs::msg::PoseArray::SharedPtr latest_cows_;
  std::vector<std::vector<cv::Point2f>> latest_vector_grid_;
  bool received_drone_{false};
  bool received_cows_{false};
  bool has_vector_grid_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OccupancyGridVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
