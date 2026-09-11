#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <opencv2/opencv.hpp>

// Constants for visualization map geometry and herding logic
constexpr double MAP_HALF_WIDTH = 10.0; // Displays 20m x 20m map (from -10m to +10m on X and Y)
constexpr int GRID_SIZE = 40;          // Grid resolution N x N cells
constexpr double GRID_CELL_SIZE = (2.0 * MAP_HALF_WIDTH) / GRID_SIZE;
constexpr int RELAXATION_MAX_ITERATIONS = 1000;
constexpr double RELAXATION_TOLERANCE = 1.0e-8;
constexpr double SOR_RELAXATION_FACTOR = 1.7;
constexpr double LOW_GRADIENT_THRESHOLD = 1.0e-6;
constexpr double POTENTIAL_DESCENT_EPSILON = 1.0e-10;
constexpr bool ENABLE_SADDLE_NEIGHBOR_DESCENT = false;
constexpr bool ENABLE_GEODESIC_FIELD_FALLBACK = true;
constexpr int CONTROL_RECOVERY_SEARCH_RADIUS_CELLS = 8;

// Herding Constants
const cv::Point2f GOAL_POSITION(10.0f, 10.0f);
constexpr double COW_EXCLUSION_RADIUS = 2.0;   // meters
constexpr double PUSH_POINT_MARGIN = 0.2;       // meters
constexpr double MAX_GOTO_MOVEMENT = 0.5;       // meters
constexpr double COW_MEMORY_ASSOCIATION_DISTANCE = 3.0;  // meters

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
    // Empty and partial detections must not erase remembered APF objectives.
    if (msg->poses.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_cows_ || !latest_cows_) {
      auto valid_cows =
        std::make_shared<geometry_msgs::msg::PoseArray>();
      valid_cows->header = msg->header;
      for (const auto& pose : msg->poses) {
        if (
          std::isfinite(pose.position.x) &&
          std::isfinite(pose.position.y))
        {
          valid_cows->poses.push_back(pose);
        }
      }
      if (valid_cows->poses.empty()) {
        return;
      }
      latest_cows_ = valid_cows;
      received_cows_ = true;
      return;
    }

    auto merged_cows =
      std::make_shared<geometry_msgs::msg::PoseArray>(*latest_cows_);
    merged_cows->header = msg->header;
    std::vector<bool> remembered_cow_updated(
      merged_cows->poses.size(), false);

    for (const auto& detected_cow : msg->poses) {
      if (
        !std::isfinite(detected_cow.position.x) ||
        !std::isfinite(detected_cow.position.y))
      {
        continue;
      }

      double nearest_distance =
        COW_MEMORY_ASSOCIATION_DISTANCE;
      int nearest_index = -1;
      for (std::size_t index = 0;
        index < merged_cows->poses.size();
        ++index)
      {
        if (remembered_cow_updated[index]) {
          continue;
        }

        const auto& remembered_cow = merged_cows->poses[index];
        const double distance = std::hypot(
          detected_cow.position.x - remembered_cow.position.x,
          detected_cow.position.y - remembered_cow.position.y);
        if (distance <= nearest_distance) {
          nearest_distance = distance;
          nearest_index = static_cast<int>(index);
        }
      }

      if (nearest_index >= 0) {
        merged_cows->poses[nearest_index] = detected_cow;
        remembered_cow_updated[nearest_index] = true;
      } else {
        merged_cows->poses.push_back(detected_cow);
        remembered_cow_updated.push_back(true);
      }
    }

    latest_cows_ = merged_cows;
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

  static bool find_nearest_free_interior_cell(
    const cv::Mat& is_fixed,
    int desired_row,
    int desired_col,
    int& result_row,
    int& result_col)
  {
    double best_distance_sq = std::numeric_limits<double>::infinity();
    result_row = -1;
    result_col = -1;

    for (int r = 1; r < GRID_SIZE - 1; ++r) {
      for (int c = 1; c < GRID_SIZE - 1; ++c) {
        if (is_fixed.at<uchar>(r, c) == 1) {
          continue;
        }

        const double row_delta = r - desired_row;
        const double col_delta = c - desired_col;
        const double distance_sq =
          row_delta * row_delta + col_delta * col_delta;
        if (
          distance_sq < best_distance_sq ||
          (
            distance_sq == best_distance_sq &&
            (
              result_row < 0 ||
              r < result_row ||
              (r == result_row && c < result_col))))
        {
          best_distance_sq = distance_sq;
          result_row = r;
          result_col = c;
        }
      }
    }

    return result_row >= 0;
  }

  static cv::Mat compute_goal_distance_grid(
    const cv::Mat& is_fixed,
    const cv::Mat& objective_mask)
  {
    using QueueEntry = std::tuple<double, int, int>;
    std::priority_queue<
      QueueEntry,
      std::vector<QueueEntry>,
      std::greater<QueueEntry>> open_cells;

    cv::Mat distance_grid(
      GRID_SIZE,
      GRID_SIZE,
      CV_64F,
      cv::Scalar(std::numeric_limits<double>::infinity()));

    for (int r = 1; r < GRID_SIZE - 1; ++r) {
      for (int c = 1; c < GRID_SIZE - 1; ++c) {
        if (objective_mask.at<uchar>(r, c) == 0) {
          continue;
        }
        distance_grid.at<double>(r, c) = 0.0;
        open_cells.emplace(0.0, r, c);
      }
    }

    while (!open_cells.empty()) {
      const auto [distance, row, col] = open_cells.top();
      open_cells.pop();
      if (distance > distance_grid.at<double>(row, col)) {
        continue;
      }

      for (int row_offset = -1; row_offset <= 1; ++row_offset) {
        for (int col_offset = -1; col_offset <= 1; ++col_offset) {
          if (row_offset == 0 && col_offset == 0) {
            continue;
          }

          const int neighbor_row = row + row_offset;
          const int neighbor_col = col + col_offset;
          if (
            neighbor_row <= 0 ||
            neighbor_row >= GRID_SIZE - 1 ||
            neighbor_col <= 0 ||
            neighbor_col >= GRID_SIZE - 1)
          {
            continue;
          }

          const bool is_goal =
            objective_mask.at<uchar>(neighbor_row, neighbor_col) == 1;
          if (
            is_fixed.at<uchar>(neighbor_row, neighbor_col) == 1 &&
            !is_goal)
          {
            continue;
          }

          // Do not pass diagonally through the corner of two fixed cells.
          if (row_offset != 0 && col_offset != 0) {
            const bool row_side_is_blocked =
              is_fixed.at<uchar>(row + row_offset, col) == 1 &&
              objective_mask.at<uchar>(row + row_offset, col) == 0;
            const bool col_side_is_blocked =
              is_fixed.at<uchar>(row, col + col_offset) == 1 &&
              objective_mask.at<uchar>(row, col + col_offset) == 0;
            if (row_side_is_blocked || col_side_is_blocked) {
              continue;
            }
          }

          const double step_distance = GRID_CELL_SIZE *
            std::hypot(
              static_cast<double>(row_offset),
              static_cast<double>(col_offset));
          const double candidate_distance = distance + step_distance;
          double& stored_distance =
            distance_grid.at<double>(neighbor_row, neighbor_col);
          if (candidate_distance + POTENTIAL_DESCENT_EPSILON < stored_distance) {
            stored_distance = candidate_distance;
            open_cells.emplace(
              candidate_distance, neighbor_row, neighbor_col);
          }
        }
      }
    }

    return distance_grid;
  }

  static bool find_local_recovery_direction(
    double drone_x,
    double drone_y,
    int drone_row,
    int drone_col,
    const std::vector<std::vector<cv::Point2f>>& vector_grid,
    const cv::Mat& is_fixed,
    cv::Point2f& recovery_direction)
  {
    double best_distance_sq = std::numeric_limits<double>::infinity();
    int best_row = -1;
    int best_col = -1;

    for (int radius = 1;
      radius <= CONTROL_RECOVERY_SEARCH_RADIUS_CELLS;
      ++radius)
    {
      const int min_row = std::max(1, drone_row - radius);
      const int max_row = std::min(GRID_SIZE - 2, drone_row + radius);
      const int min_col = std::max(1, drone_col - radius);
      const int max_col = std::min(GRID_SIZE - 2, drone_col + radius);

      for (int row = min_row; row <= max_row; ++row) {
        for (int col = min_col; col <= max_col; ++col) {
          if (
            std::max(
              std::abs(row - drone_row),
              std::abs(col - drone_col)) != radius ||
            is_fixed.at<uchar>(row, col) == 1)
          {
            continue;
          }

          const cv::Point2f candidate_vector = vector_grid[row][col];
          if (std::hypot(candidate_vector.x, candidate_vector.y) <= 0.5f) {
            continue;
          }

          const cv::Point2f candidate_position = grid_to_world(row, col);
          const double delta_x = candidate_position.x - drone_x;
          const double delta_y = candidate_position.y - drone_y;
          const double distance_sq = delta_x * delta_x + delta_y * delta_y;
          if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            best_row = row;
            best_col = col;
          }
        }
      }

      if (best_row >= 0) {
        break;
      }
    }

    if (best_row < 0) {
      return false;
    }

    const cv::Point2f recovery_target = grid_to_world(best_row, best_col);
    const double direction_x = recovery_target.x - drone_x;
    const double direction_y = recovery_target.y - drone_y;
    const double magnitude = std::hypot(direction_x, direction_y);
    if (magnitude <= POTENTIAL_DESCENT_EPSILON) {
      return false;
    }

    recovery_direction = cv::Point2f(
      static_cast<float>(direction_x / magnitude),
      static_cast<float>(direction_y / magnitude));
    return true;
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
    cv::Mat potential_grid(
      GRID_SIZE, GRID_SIZE, CV_64F, cv::Scalar(0.5));
    cv::Mat is_fixed(
      GRID_SIZE, GRID_SIZE, CV_8UC1, cv::Scalar(0));
    cv::Mat objective_mask(
      GRID_SIZE, GRID_SIZE, CV_8UC1, cv::Scalar(0));
    std::vector<cv::Point2f> push_points;

    // Circular Cow Exclusion & Pushing Objective Setup
    if (has_cows && cows) {
      for (const auto& cow_pose : cows->poses) {
        const float cow_x = cow_pose.position.x;
        const float cow_y = cow_pose.position.y;
        if (!std::isfinite(cow_x) || !std::isfinite(cow_y)) {
          continue;
        }

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
              potential_grid.at<double>(r, c) = 1.0;
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

          if (std::isfinite(push_pt.x) && std::isfinite(push_pt.y)) {
            push_points.push_back(push_pt);
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
      potential_grid.at<double>(0, c) = 1.0;
      potential_grid.at<double>(GRID_SIZE - 1, c) = 1.0;
      is_fixed.at<uchar>(0, c) = 1;
      is_fixed.at<uchar>(GRID_SIZE - 1, c) = 1;
    }
    for (int r = 0; r < GRID_SIZE; ++r) {
      potential_grid.at<double>(r, 0) = 1.0;
      potential_grid.at<double>(r, GRID_SIZE - 1) = 1.0;
      is_fixed.at<uchar>(r, 0) = 1;
      is_fixed.at<uchar>(r, GRID_SIZE - 1) = 1;
    }

    // Apply objectives after every high-potential boundary has been built.
    // A blocked or out-of-map objective is moved to the nearest free interior
    // cell, so a visible objective always remains a real zero-potential sink.
    for (const auto& push_point : push_points) {
      const double normalized_x =
        (push_point.x + MAP_HALF_WIDTH) / (2.0 * MAP_HALF_WIDTH);
      const double normalized_y =
        (MAP_HALF_WIDTH - push_point.y) / (2.0 * MAP_HALF_WIDTH);
      const int desired_col = std::clamp(
        static_cast<int>(std::floor(
          std::clamp(normalized_x, 0.0, 1.0) * GRID_SIZE)),
        1,
        GRID_SIZE - 2);
      const int desired_row = std::clamp(
        static_cast<int>(std::floor(
          std::clamp(normalized_y, 0.0, 1.0) * GRID_SIZE)),
        1,
        GRID_SIZE - 2);

      int objective_row = -1;
      int objective_col = -1;
      if (!find_nearest_free_interior_cell(
          is_fixed,
          desired_row,
          desired_col,
          objective_row,
          objective_col))
      {
        continue;
      }

      grid[objective_row][objective_col] = PUSH_OBJECTIVE;
      potential_grid.at<double>(objective_row, objective_col) = 0.0;
      is_fixed.at<uchar>(objective_row, objective_col) = 1;
      objective_mask.at<uchar>(objective_row, objective_col) = 1;
    }

    const bool has_push_objective =
      cv::countNonZero(objective_mask) > 0;

    // Solve Laplace's equation with in-place SOR until the field converges.
    if (has_push_objective) {
      for (int iter = 0; iter < RELAXATION_MAX_ITERATIONS; ++iter) {
        double max_change = 0.0;
        for (int r = 1; r < GRID_SIZE - 1; ++r) {
          for (int c = 1; c < GRID_SIZE - 1; ++c) {
            if (is_fixed.at<uchar>(r, c) == 1) {
              continue;
            }

            const double old_value = potential_grid.at<double>(r, c);
            const double neighbor_average = (
              potential_grid.at<double>(r, c - 1) +
              potential_grid.at<double>(r, c + 1) +
              potential_grid.at<double>(r - 1, c) +
              potential_grid.at<double>(r + 1, c)) / 4.0;
            const double new_value = old_value +
              SOR_RELAXATION_FACTOR * (neighbor_average - old_value);

            potential_grid.at<double>(r, c) = new_value;
            max_change = std::max(max_change, std::abs(new_value - old_value));
          }
        }

        if (max_change < RELAXATION_TOLERANCE) {
          break;
        }
      }
    }

    // Store unit movement directions. When the centered gradient cancels at a
    // saddle, descend to the steepest lower neighbor in the same potential grid.
    std::vector<std::vector<cv::Point2f>> vector_grid(
      GRID_SIZE,
      std::vector<cv::Point2f>(
        GRID_SIZE, cv::Point2f(0.0f, 0.0f)));
    cv::Mat saddle_escape_grid(
      GRID_SIZE, GRID_SIZE, CV_8UC1, cv::Scalar(0));
    const cv::Mat goal_distance_grid = compute_goal_distance_grid(
      is_fixed, objective_mask);

    if (has_push_objective) {
      for (int r = 1; r < GRID_SIZE - 1; ++r) {
        for (int c = 1; c < GRID_SIZE - 1; ++c) {
          if (is_fixed.at<uchar>(r, c) == 1) {
            continue;
          }

          const double left = potential_grid.at<double>(r, c - 1);
          const double right = potential_grid.at<double>(r, c + 1);
          const double up = potential_grid.at<double>(r - 1, c);
          const double down = potential_grid.at<double>(r + 1, c);
          const double vector_x =
            (left - right) / (2.0 * GRID_CELL_SIZE);
          const double vector_y =
            (down - up) / (2.0 * GRID_CELL_SIZE);
          const double magnitude = std::hypot(vector_x, vector_y);

          if (magnitude >= LOW_GRADIENT_THRESHOLD) {
            vector_grid[r][c] = cv::Point2f(
              static_cast<float>(vector_x / magnitude),
              static_cast<float>(vector_y / magnitude));
            continue;
          }

          if (!ENABLE_SADDLE_NEIGHBOR_DESCENT) {
            if (!ENABLE_GEODESIC_FIELD_FALLBACK) {
              continue;
            }

            const double current_distance =
              goal_distance_grid.at<double>(r, c);
            if (!std::isfinite(current_distance)) {
              continue;
            }

            double best_distance = current_distance;
            int best_distance_row = -1;
            int best_distance_col = -1;
            for (int row_offset = -1; row_offset <= 1; ++row_offset) {
              for (int col_offset = -1; col_offset <= 1; ++col_offset) {
                if (row_offset == 0 && col_offset == 0) {
                  continue;
                }

                const int neighbor_row = r + row_offset;
                const int neighbor_col = c + col_offset;
                if (row_offset != 0 && col_offset != 0) {
                  const bool row_side_is_blocked =
                    is_fixed.at<uchar>(r + row_offset, c) == 1 &&
                    objective_mask.at<uchar>(r + row_offset, c) == 0;
                  const bool col_side_is_blocked =
                    is_fixed.at<uchar>(r, c + col_offset) == 1 &&
                    objective_mask.at<uchar>(r, c + col_offset) == 0;
                  if (row_side_is_blocked || col_side_is_blocked) {
                    continue;
                  }
                }
                const double neighbor_distance =
                  goal_distance_grid.at<double>(
                    neighbor_row, neighbor_col);
                if (
                  neighbor_distance + POTENTIAL_DESCENT_EPSILON <
                  best_distance)
                {
                  best_distance = neighbor_distance;
                  best_distance_row = neighbor_row;
                  best_distance_col = neighbor_col;
                }
              }
            }

            if (best_distance_row >= 0) {
              const double direction_x = best_distance_col - c;
              const double direction_y = r - best_distance_row;
              const double direction_magnitude =
                std::hypot(direction_x, direction_y);
              vector_grid[r][c] = cv::Point2f(
                static_cast<float>(
                  direction_x / direction_magnitude),
                static_cast<float>(
                  direction_y / direction_magnitude));
            }
            continue;
          }

          const double current_potential =
            potential_grid.at<double>(r, c);
          double best_descent = POTENTIAL_DESCENT_EPSILON;
          int best_row = -1;
          int best_col = -1;

          for (int row_offset = -1; row_offset <= 1; ++row_offset) {
            for (int col_offset = -1; col_offset <= 1; ++col_offset) {
              if (row_offset == 0 && col_offset == 0) {
                continue;
              }

              const int neighbor_row = r + row_offset;
              const int neighbor_col = c + col_offset;
              const bool is_goal =
                grid[neighbor_row][neighbor_col] == PUSH_OBJECTIVE;
              if (
                is_fixed.at<uchar>(neighbor_row, neighbor_col) == 1 &&
                !is_goal)
              {
                continue;
              }

              const double neighbor_potential =
                potential_grid.at<double>(neighbor_row, neighbor_col);
              const double neighbor_distance = GRID_CELL_SIZE *
                std::hypot(
                  static_cast<double>(row_offset),
                  static_cast<double>(col_offset));
              const double descent =
                (current_potential - neighbor_potential) /
                neighbor_distance;

              const bool stronger_descent =
                descent >
                best_descent + POTENTIAL_DESCENT_EPSILON;
              const bool deterministic_tie =
                std::abs(descent - best_descent) <=
                  POTENTIAL_DESCENT_EPSILON &&
                descent > POTENTIAL_DESCENT_EPSILON &&
                (
                  best_row < 0 ||
                  neighbor_row < best_row ||
                  (
                    neighbor_row == best_row &&
                    neighbor_col < best_col));

              if (stronger_descent || deterministic_tie) {
                best_descent = descent;
                best_row = neighbor_row;
                best_col = neighbor_col;
              }
            }
          }

          if (best_row >= 0) {
            const double direction_x = best_col - c;
            const double direction_y = r - best_row;
            const double direction_magnitude =
              std::hypot(direction_x, direction_y);
            vector_grid[r][c] = cv::Point2f(
              static_cast<float>(
                direction_x / direction_magnitude),
              static_cast<float>(
                direction_y / direction_magnitude));
            saddle_escape_grid.at<uchar>(r, c) = 1;
          }
        }
      }
    }

    std::size_t usable_vector_count = 0;
    for (const auto& row : vector_grid) {
      for (const auto& vector : row) {
        if (std::hypot(vector.x, vector.y) > 0.5f) {
          ++usable_vector_count;
        }
      }
    }
    const bool candidate_field_is_valid =
      has_push_objective &&
      usable_vector_count > 0 &&
      cv::checkRange(potential_grid);

    // Atomically commit only complete, usable fields. A failed candidate keeps
    // the last field for both control and visualization.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (candidate_field_is_valid) {
        latest_vector_grid_ = vector_grid;
        latest_cell_grid_ = grid;
        latest_is_fixed_grid_ = is_fixed.clone();
        latest_saddle_escape_grid_ = saddle_escape_grid.clone();
        has_vector_grid_ = true;
        has_field_snapshot_ = true;
      } else if (has_field_snapshot_) {
        vector_grid = latest_vector_grid_;
        grid = latest_cell_grid_;
        is_fixed = latest_is_fixed_grid_.clone();
        saddle_escape_grid = latest_saddle_escape_grid_.clone();
      }
    }

    // Render Displays (Modular Functions)
    compute_and_display_occupancy_map(grid);
    compute_and_display_vector_grid(
      grid, vector_grid, is_fixed, saddle_escape_grid);
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
    const cv::Mat& is_fixed,
    const cv::Mat& saddle_escape_grid)
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

        if (mag > 0.5f && is_fixed.at<uchar>(r, c) == 0) {
          const float max_arrow_len = cell_size * 0.8f;
          const float arrow_scale = max_arrow_len / 2.0f;

          const cv::Point2f dir(vec.x / mag, -vec.y / mag);
          const cv::Point2f start_pt = center - dir * arrow_scale;
          const cv::Point2f end_pt = center + dir * arrow_scale;
          const cv::Scalar arrow_color =
            saddle_escape_grid.at<uchar>(r, c) == 1 ?
            cv::Scalar(0, 140, 255) : cv::Scalar(50, 50, 50);

          cv::arrowedLine(
            vector_img, start_pt, end_pt, arrow_color,
            1, cv::LINE_AA, 0, 0.3);
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
    std::vector<std::vector<CellState>> cell_grid;
    cv::Mat is_fixed_grid;
    cv::Point2f last_move_direction;
    bool has_drone = false;
    bool has_cows = false;
    bool has_grid = false;
    bool has_last_move_direction = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      drone_pose = latest_drone_pose_;
      cows = latest_cows_;
      vector_grid = latest_vector_grid_;
      cell_grid = latest_cell_grid_;
      is_fixed_grid = latest_is_fixed_grid_.clone();
      last_move_direction = last_nonzero_move_direction_;
      has_drone = received_drone_;
      has_cows = received_cows_;
      has_grid = has_vector_grid_;
      has_last_move_direction = has_last_nonzero_move_direction_;
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

        const bool is_push_objective =
          !cell_grid.empty() &&
          cell_grid[d_row][d_col] == PUSH_OBJECTIVE;

        if (mag <= 0.5f && !is_push_objective) {
          cv::Point2f recovery_direction;
          if (
            !is_fixed_grid.empty() &&
            find_local_recovery_direction(
              drone_x,
              drone_y,
              d_row,
              d_col,
              vector_grid,
              is_fixed_grid,
              recovery_direction))
          {
            vec = recovery_direction;
            mag = 1.0f;
          } else if (has_last_move_direction) {
            vec = last_move_direction;
            mag = std::hypot(vec.x, vec.y);
          }
        }

        cv::Point2f move_vec(0.0f, 0.0f);
        if (mag > 0.5f) {
          // The grid stores either a unit direction or zero, so every valid
          // direction produces the same fixed waypoint displacement.
          move_vec = cv::Point2f((vec.x / mag) * static_cast<float>(MAX_GOTO_MOVEMENT),
                                 (vec.y / mag) * static_cast<float>(MAX_GOTO_MOVEMENT));
          {
            std::lock_guard<std::mutex> lock(mutex_);
            last_nonzero_move_direction_ = cv::Point2f(
              vec.x / mag, vec.y / mag);
            has_last_nonzero_move_direction_ = true;
          }
        } else if (!is_push_objective) {
          // Keep the previous nonzero waypoint active rather than replacing it
          // with an accidental command to hold the current position.
          return;
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
  std::vector<std::vector<CellState>> latest_cell_grid_;
  cv::Mat latest_is_fixed_grid_;
  cv::Mat latest_saddle_escape_grid_;
  cv::Point2f last_nonzero_move_direction_{0.0f, 0.0f};
  bool received_drone_{false};
  bool received_cows_{false};
  bool has_vector_grid_{false};
  bool has_field_snapshot_{false};
  bool has_last_nonzero_move_direction_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OccupancyGridVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
