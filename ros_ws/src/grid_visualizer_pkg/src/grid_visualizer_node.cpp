#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <opencv2/opencv.hpp>

// Constants for visualization map geometry
constexpr double MAP_HALF_WIDTH = 10.0; // Displays 10m x 10m map (from -5m to +5m on X and Y)
constexpr int GRID_SIZE = 40;          // Grid resolution N x N cells

class OccupancyGridVisualizer : public rclcpp::Node
{
public:
  OccupancyGridVisualizer()
  : Node("occupancy_grid_visualizer")
  {
    // Declare and get namespace parameter
    this->declare_parameter<std::string>("namespace", "");
    std::string ns = this->get_parameter("namespace").as_string();

    // Construct the drone topic dynamically
    std::string drone_topic = "";
    if (ns.empty()) {
      drone_topic = "gt_pose";
    } else if (ns.front() == '/') {
      drone_topic = ns + "/gt_pose";
    } else {
      drone_topic = "/" + ns + "/gt_pose";
    }

    RCLCPP_INFO(this->get_logger(), "Drone topic selected: %s", drone_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Cows topic selected: /cows_pos");

    // Initialize subscriptions
    drone_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      drone_topic,
      10,
      std::bind(&OccupancyGridVisualizer::drone_callback, this, std::placeholders::_1)
    );

    cow_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/cows_pos",
      10,
      std::bind(&OccupancyGridVisualizer::cow_callback, this, std::placeholders::_1)
    );

    // Initialize timer running at 5Hz (200ms)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&OccupancyGridVisualizer::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Grid size: %dx%d (%f m cell width)", GRID_SIZE, GRID_SIZE, (2.0 * MAP_HALF_WIDTH) / GRID_SIZE);
  }

private:
  enum CellState {
    EMPTY = 0,
    DRONE = 1,
    COW = 2
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

  void timer_callback()
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

    // Initialize grid map (GRID_SIZE x GRID_SIZE) to EMPTY
    std::vector<std::vector<CellState>> grid(GRID_SIZE, std::vector<CellState>(GRID_SIZE, EMPTY));

    // Helper lambda to map world coordinates (x,y) to grid (row, col) inside 10x10m frame [-5, +5]
    auto map_to_grid = [](double x, double y, int& row, int& col) -> bool {
      if (x < -MAP_HALF_WIDTH || x > MAP_HALF_WIDTH || y < -MAP_HALF_WIDTH || y > MAP_HALF_WIDTH) {
        return false;
      }
      // Normalize range [ -MAP_HALF_WIDTH, MAP_HALF_WIDTH ] -> [ 0, 1 ]
      double norm_x = (x + MAP_HALF_WIDTH) / (2.0 * MAP_HALF_WIDTH);
      double norm_y = (MAP_HALF_WIDTH - y) / (2.0 * MAP_HALF_WIDTH);

      col = static_cast<int>(std::floor(norm_x * GRID_SIZE));
      row = static_cast<int>(std::floor(norm_y * GRID_SIZE));

      // Boundary clamp checks
      col = std::max(0, std::min(GRID_SIZE - 1, col));
      row = std::max(0, std::min(GRID_SIZE - 1, row));
      return true;
    };

    // Mark cows on grid (COW = Red)
    if (has_cows && cows) {
      for (const auto& pose : cows->poses) {
        int row = 0, col = 0;
        if (map_to_grid(pose.position.x, pose.position.y, row, col)) {
          grid[row][col] = COW;
        }
      }
    }

    // Mark drone on grid (DRONE = Blue)
    if (has_drone && drone_pose) {
      int row = 0, col = 0;
      if (map_to_grid(drone_pose->position.x, drone_pose->position.y, row, col)) {
        grid[row][col] = DRONE;
      }
    }

    // Render 3-channel color image (CV_8UC3)
    cv::Mat display_img(GRID_SIZE, GRID_SIZE, CV_8UC3, cv::Scalar(255, 255, 255)); // White background
    for (int r = 0; r < GRID_SIZE; ++r) {
      for (int c = 0; c < GRID_SIZE; ++c) {
        if (grid[r][c] == DRONE) {
          display_img.at<cv::Vec3b>(r, c) = cv::Vec3b(255, 0, 0); // Blue (BGR)
        } else if (grid[r][c] == COW) {
          display_img.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 255); // Red (BGR)
        }
      }
    }

    // Resize image to 400x400 using INTER_NEAREST for clear grid rendering
    cv::Mat enlarged_img;
    cv::resize(display_img, enlarged_img, cv::Size(400, 400), 0, 0, cv::INTER_NEAREST);

    // Display rendering window
    cv::imshow("Occupancy Grid (10x10m)", enlarged_img);
    cv::waitKey(1);
  }

  // Subscriptions, Timer & Mutex
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr drone_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cow_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  geometry_msgs::msg::Pose::SharedPtr latest_drone_pose_;
  geometry_msgs::msg::PoseArray::SharedPtr latest_cows_;
  bool received_drone_{false};
  bool received_cows_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OccupancyGridVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
