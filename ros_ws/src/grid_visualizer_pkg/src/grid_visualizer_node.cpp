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
#include <stdexcept>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <opencv2/opencv.hpp>

// Constants for visualization map geometry and herding logic
constexpr double MAP_HALF_WIDTH = 10.0; // Displays 20m x 20m map (from -10m to +10m on X and Y)
constexpr int GRID_SIZE = 40;          // Grid resolution N x N cells
constexpr double GRID_CELL_SIZE = (2.0 * MAP_HALF_WIDTH) / GRID_SIZE;
constexpr int GRID_CENTER_ROW = GRID_SIZE / 2;
constexpr int GRID_CENTER_COL = GRID_SIZE / 2;
constexpr double LOCAL_OBJECTIVE_HALF_EXTENT =
  MAP_HALF_WIDTH - 1.5 * GRID_CELL_SIZE;
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
constexpr double DEFAULT_COW_EXCLUSION_RADIUS = 3.0;  // meters
constexpr double DEFAULT_PUSH_POINT_MARGIN = 0.2;       // meters
constexpr double DEFAULT_MAX_NEAR_GOAL_OBJECTIVE_POTENTIAL = 0.5;
constexpr double DEFAULT_OBJECTIVE_PRIORITY_MIN_SPREAD = 1.0;  // meters
constexpr double DEFAULT_OBJECTIVE_PRIORITY_FALLBACK_BIAS = 10.0;  // meters
constexpr double MAX_GOTO_MOVEMENT = 0.5;       // meters
constexpr double FINAL_GOAL_REACHED_RADIUS = 2.0;           // meters
constexpr const char* DRONE_NAMESPACE_PREFIX = "/simple_drone";

class OccupancyGridVisualizer : public rclcpp::Node
{
public:
  OccupancyGridVisualizer()
  : Node("occupancy_grid_visualizer")
  {
    drone_index_ = this->declare_parameter<int>("drone_index", 0);
    total_drones_ = this->declare_parameter<int>("total_drones", 1);
    global_map_min_x_ =
      this->declare_parameter<double>("global_map_min_x", -50.0);
    global_map_max_x_ =
      this->declare_parameter<double>("global_map_max_x", 50.0);
    global_map_min_y_ =
      this->declare_parameter<double>("global_map_min_y", -50.0);
    global_map_max_y_ =
      this->declare_parameter<double>("global_map_max_y", 50.0);
    cow_exclusion_radius_ = this->declare_parameter<double>(
      "cow_exclusion_radius", DEFAULT_COW_EXCLUSION_RADIUS);
    push_point_margin_ = this->declare_parameter<double>(
      "push_point_margin", DEFAULT_PUSH_POINT_MARGIN);
    max_near_goal_objective_potential_ = this->declare_parameter<double>(
      "max_near_goal_objective_potential",
      DEFAULT_MAX_NEAR_GOAL_OBJECTIVE_POTENTIAL);
    objective_priority_min_spread_ = this->declare_parameter<double>(
      "objective_priority_min_spread",
      DEFAULT_OBJECTIVE_PRIORITY_MIN_SPREAD);
    objective_priority_fallback_bias_m_ = this->declare_parameter<double>(
      "objective_priority_fallback_bias_m",
      DEFAULT_OBJECTIVE_PRIORITY_FALLBACK_BIAS);

    if (total_drones_ <= 0) {
      throw std::invalid_argument("total_drones must be greater than zero");
    }
    if (drone_index_ < 0 || drone_index_ >= total_drones_) {
      throw std::invalid_argument(
        "drone_index must be in the range [0, total_drones)");
    }
    if (
      !std::isfinite(global_map_min_x_) ||
      !std::isfinite(global_map_max_x_) ||
      !std::isfinite(global_map_min_y_) ||
      !std::isfinite(global_map_max_y_) ||
      global_map_min_x_ >= global_map_max_x_ ||
      global_map_min_y_ >= global_map_max_y_)
    {
      throw std::invalid_argument(
        "global map bounds must be finite and each minimum must be "
        "smaller than its maximum");
    }

    if (
      !std::isfinite(cow_exclusion_radius_) ||
      cow_exclusion_radius_ <= 0.0 ||
      !std::isfinite(push_point_margin_) ||
      push_point_margin_ < 0.0)
    {
      throw std::invalid_argument(
        "cow_exclusion_radius must be positive and push_point_margin "
        "must be nonnegative");
    }

    if (
      !std::isfinite(max_near_goal_objective_potential_) ||
      max_near_goal_objective_potential_ < 0.0 ||
      max_near_goal_objective_potential_ >= 1.0 ||
      !std::isfinite(objective_priority_min_spread_) ||
      objective_priority_min_spread_ <= 0.0 ||
      !std::isfinite(objective_priority_fallback_bias_m_) ||
      objective_priority_fallback_bias_m_ < 0.0)
    {
      throw std::invalid_argument(
        "max_near_goal_objective_potential must be in [0, 1), "
        "objective_priority_min_spread must be positive, and "
        "objective_priority_fallback_bias_m must be nonnegative");
    }

    global_map_center_ = cv::Point2f(
      static_cast<float>((global_map_min_x_ + global_map_max_x_) / 2.0),
      static_cast<float>((global_map_min_y_ + global_map_max_y_) / 2.0));

    drone_namespace_ =
      std::string(DRONE_NAMESPACE_PREFIX) + std::to_string(drone_index_);
    grid_name_ = "grid" + std::to_string(drone_index_);
    occupancy_window_name_ = "Occupancy Grid - " + grid_name_;
    vector_window_name_ = "Vector Movement Grid - " + grid_name_;

    // The index parameter owns topic selection. The ROS namespace only
    // organizes the node in the graph and cannot redirect another drone.
    const std::string drone_gt_topic = drone_namespace_ + "/gt_pose";
    const std::string goto_topic = drone_namespace_ + "/goto";
    const std::string focus_topic = drone_namespace_ + "/focusin";

    RCLCPP_INFO(
      this->get_logger(),
      "%s controls drone %d of %d in %s",
      grid_name_.c_str(),
      drone_index_,
      total_drones_,
      drone_namespace_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Global map bounds: X=[%.2f, %.2f], Y=[%.2f, %.2f]",
      global_map_min_x_,
      global_map_max_x_,
      global_map_min_y_,
      global_map_max_y_);
    RCLCPP_INFO(
      this->get_logger(),
      "Cow approach distance: %.2f m (exclusion %.2f + margin %.2f)",
      cow_exclusion_radius_ + push_point_margin_,
      cow_exclusion_radius_,
      push_point_margin_);
    RCLCPP_INFO(
      this->get_logger(),
      "Cow objective priority: max potential %.2f, minimum spread %.2f m, "
      "fallback bias %.2f m",
      max_near_goal_objective_potential_,
      objective_priority_min_spread_,
      objective_priority_fallback_bias_m_);
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

  struct PushObjective
  {
    cv::Point2f global_point;
    double cow_goal_distance{0.0};
    double boundary_potential{0.0};
  };

  void drone_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    if (
      !std::isfinite(msg->position.x) ||
      !std::isfinite(msg->position.y))
    {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_drone_pose_ = msg;
    received_drone_ = true;
  }

  void cow_callback(
    const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    // Every tracker message is an authoritative snapshot. An empty array
    // clears retired tracks; if messages stop entirely, latest_cows_ remains
    // unchanged and the previous map continues to drive the drone.
    auto valid_cows =
      std::make_shared<geometry_msgs::msg::PoseArray>();
    valid_cows->header = msg->header;
    for (const auto& pose : msg->poses) {
      if (
        is_inside_global_map(
          pose.position.x,
          pose.position.y))
      {
        valid_cows->poses.push_back(pose);
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cows_ = valid_cows;
    received_cows_ = true;
  }

  // -------------------------------------------------------------
  // HELPER MAPPING FUNCTIONS
  // -------------------------------------------------------------

  static bool local_to_grid(double x, double y, int& row, int& col)
  {
    if (
      !std::isfinite(x) ||
      !std::isfinite(y) ||
      x < -MAP_HALF_WIDTH ||
      x > MAP_HALF_WIDTH ||
      y < -MAP_HALF_WIDTH ||
      y > MAP_HALF_WIDTH)
    {
      return false;
    }

    const double norm_x =
      (x + MAP_HALF_WIDTH) / (2.0 * MAP_HALF_WIDTH);
    const double norm_y =
      (MAP_HALF_WIDTH - y) / (2.0 * MAP_HALF_WIDTH);

    col = static_cast<int>(std::floor(norm_x * GRID_SIZE));
    row = static_cast<int>(std::floor(norm_y * GRID_SIZE));

    col = std::clamp(col, 0, GRID_SIZE - 1);
    row = std::clamp(row, 0, GRID_SIZE - 1);
    return true;
  }

  static cv::Point2f grid_to_local(int row, int col)
  {
    const float norm_x = (col + 0.5f) / GRID_SIZE;
    const float norm_y = (row + 0.5f) / GRID_SIZE;

    const float x =
      norm_x * (2.0f * MAP_HALF_WIDTH) - MAP_HALF_WIDTH;
    const float y =
      MAP_HALF_WIDTH - norm_y * (2.0f * MAP_HALF_WIDTH);
    return cv::Point2f(x, y);
  }

  static cv::Point2f grid_to_global(
    int row,
    int col,
    const cv::Point2f& grid_center)
  {
    return grid_center + grid_to_local(row, col);
  }

  bool is_inside_global_map(double x, double y) const
  {
    return
      std::isfinite(x) &&
      std::isfinite(y) &&
      x >= global_map_min_x_ &&
      x <= global_map_max_x_ &&
      y >= global_map_min_y_ &&
      y <= global_map_max_y_;
  }

  bool global_to_grid(
    double global_x,
    double global_y,
    const cv::Point2f& grid_center,
    int& row,
    int& col) const
  {
    return local_to_grid(
      global_x - grid_center.x,
      global_y - grid_center.y,
      row,
      col);
  }

  bool clip_target_to_valid_region(
    const cv::Point2f& grid_center,
    const cv::Point2f& target,
    cv::Point2f& clipped_target) const
  {
    const double valid_min_x = std::max(
      static_cast<double>(grid_center.x) - LOCAL_OBJECTIVE_HALF_EXTENT,
      global_map_min_x_);
    const double valid_max_x = std::min(
      static_cast<double>(grid_center.x) + LOCAL_OBJECTIVE_HALF_EXTENT,
      global_map_max_x_);
    const double valid_min_y = std::max(
      static_cast<double>(grid_center.y) - LOCAL_OBJECTIVE_HALF_EXTENT,
      global_map_min_y_);
    const double valid_max_y = std::min(
      static_cast<double>(grid_center.y) + LOCAL_OBJECTIVE_HALF_EXTENT,
      global_map_max_y_);

    if (
      valid_min_x > valid_max_x ||
      valid_min_y > valid_max_y ||
      grid_center.x < valid_min_x ||
      grid_center.x > valid_max_x ||
      grid_center.y < valid_min_y ||
      grid_center.y > valid_max_y)
    {
      return false;
    }

    const double direction_x = target.x - grid_center.x;
    const double direction_y = target.y - grid_center.y;
    if (
      std::abs(direction_x) <= POTENTIAL_DESCENT_EPSILON &&
      std::abs(direction_y) <= POTENTIAL_DESCENT_EPSILON)
    {
      clipped_target = grid_center;
      return true;
    }

    double scale = 1.0;
    if (direction_x > POTENTIAL_DESCENT_EPSILON) {
      scale = std::min(
        scale, (valid_max_x - grid_center.x) / direction_x);
    } else if (direction_x < -POTENTIAL_DESCENT_EPSILON) {
      scale = std::min(
        scale, (valid_min_x - grid_center.x) / direction_x);
    }
    if (direction_y > POTENTIAL_DESCENT_EPSILON) {
      scale = std::min(
        scale, (valid_max_y - grid_center.y) / direction_y);
    } else if (direction_y < -POTENTIAL_DESCENT_EPSILON) {
      scale = std::min(
        scale, (valid_min_y - grid_center.y) / direction_y);
    }

    scale = std::clamp(scale, 0.0, 1.0);
    if (scale <= POTENTIAL_DESCENT_EPSILON) {
      return false;
    }

    clipped_target = cv::Point2f(
      static_cast<float>(grid_center.x + scale * direction_x),
      static_cast<float>(grid_center.y + scale * direction_y));
    return
      std::isfinite(clipped_target.x) &&
      std::isfinite(clipped_target.y);
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
    const cv::Mat& objective_mask,
    const cv::Mat& objective_source_cost)
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
        const double source_cost =
          objective_source_cost.at<double>(r, c);
        distance_grid.at<double>(r, c) = source_cost;
        open_cells.emplace(source_cost, r, c);
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
    const cv::Point2f& grid_center,
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

          const cv::Point2f candidate_position =
            grid_to_global(row, col, grid_center);
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

    const cv::Point2f recovery_target =
      grid_to_global(best_row, best_col, grid_center);
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
    std::vector<PushObjective> remembered_push_objectives;
    bool has_drone = false;
    bool has_cows = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      drone_pose = latest_drone_pose_;
      cows = latest_cows_;
      remembered_push_objectives = latest_valid_push_objectives_;
      has_drone = received_drone_;
      has_cows = received_cows_;
    }

    if (!has_drone || !drone_pose) {
      return;
    }

    const cv::Point2f grid_center(
      static_cast<float>(drone_pose->position.x),
      static_cast<float>(drone_pose->position.y));

    std::vector<std::vector<CellState>> grid(
      GRID_SIZE,
      std::vector<CellState>(GRID_SIZE, EMPTY));
    cv::Mat potential_grid(
      GRID_SIZE, GRID_SIZE, CV_64F, cv::Scalar(0.5));
    cv::Mat is_fixed(
      GRID_SIZE, GRID_SIZE, CV_8UC1, cv::Scalar(0));
    cv::Mat objective_mask(
      GRID_SIZE, GRID_SIZE, CV_8UC1, cv::Scalar(0));
    cv::Mat objective_source_cost(
      GRID_SIZE, GRID_SIZE, CV_64F, cv::Scalar(0.0));
    std::vector<PushObjective> current_push_objectives;
    bool evaluated_at_least_one_cow = false;

    // The local perimeter closes the relaxation problem. Cells whose global
    // centers lie outside the finite map are also unavailable.
    for (int row = 0; row < GRID_SIZE; ++row) {
      for (int col = 0; col < GRID_SIZE; ++col) {
        const cv::Point2f global_cell =
          grid_to_global(row, col, grid_center);
        const bool is_local_perimeter =
          row == 0 ||
          row == GRID_SIZE - 1 ||
          col == 0 ||
          col == GRID_SIZE - 1;
        if (
          is_local_perimeter ||
          !is_inside_global_map(global_cell.x, global_cell.y))
        {
          grid[row][col] = EXCLUSION;
          potential_grid.at<double>(row, col) = 1.0;
          is_fixed.at<uchar>(row, col) = 1;
        }
      }
    }

    // Cow state remains global. Reproject every cow into this drone's local
    // moving window instead of shifting or mutating remembered positions.
    if (has_cows && cows) {
      for (const auto& cow_pose : cows->poses) {
        const float cow_x = cow_pose.position.x;
        const float cow_y = cow_pose.position.y;
        if (!is_inside_global_map(cow_x, cow_y)) {
          continue;
        }
        evaluated_at_least_one_cow = true;

        int cow_row = 0;
        int cow_col = 0;
        if (global_to_grid(
            cow_x, cow_y, grid_center, cow_row, cow_col))
        {
          grid[cow_row][cow_col] = COW;
        }

        // A cow outside the visible window can still affect cells where its
        // exclusion circle overlaps the window.
        for (int row = 0; row < GRID_SIZE; ++row) {
          for (int col = 0; col < GRID_SIZE; ++col) {
            const cv::Point2f global_cell =
              grid_to_global(row, col, grid_center);
            const double distance = std::hypot(
              global_cell.x - cow_x,
              global_cell.y - cow_y);
            if (distance <= cow_exclusion_radius_) {
              if (grid[row][col] != COW) {
                grid[row][col] = EXCLUSION;
              }
              potential_grid.at<double>(row, col) = 1.0;
              is_fixed.at<uchar>(row, col) = 1;
            }
          }
        }

        const cv::Point2f cow_point(cow_x, cow_y);
        const cv::Point2f direction_to_goal =
          GOAL_POSITION - cow_point;
        const float goal_distance = std::hypot(
          direction_to_goal.x,
          direction_to_goal.y);

        // A cow inside the final goal radius no longer needs a behind-cow
        // push objective. The objective is generated again if it leaves.
        if (goal_distance > FINAL_GOAL_REACHED_RADIUS) {
          const cv::Point2f normalized_direction(
            direction_to_goal.x / goal_distance,
            direction_to_goal.y / goal_distance);
          const cv::Point2f global_push_point =
            cow_point -
            normalized_direction *
            (cow_exclusion_radius_ + push_point_margin_);

          if (
            std::isfinite(global_push_point.x) &&
            std::isfinite(global_push_point.y))
          {
            current_push_objectives.push_back(
              PushObjective{
                global_push_point,
                static_cast<double>(goal_distance),
                0.0});
          }
        }
      }
    }

    const bool authoritative_empty_cow_set =
      has_cows &&
      cows &&
      cows->poses.empty();
    const bool has_authoritative_cow_evaluation =
      evaluated_at_least_one_cow ||
      authoritative_empty_cow_set;
    const bool all_evaluated_cows_at_goal =
      has_authoritative_cow_evaluation &&
      current_push_objectives.empty();

    if (!current_push_objectives.empty()) {
      const auto distance_limits = std::minmax_element(
        current_push_objectives.begin(),
        current_push_objectives.end(),
        [](const PushObjective& lhs, const PushObjective& rhs) {
          return lhs.cow_goal_distance < rhs.cow_goal_distance;
        });
      const double minimum_goal_distance =
        distance_limits.first->cow_goal_distance;
      const double maximum_goal_distance =
        distance_limits.second->cow_goal_distance;
      const double normalization_span = std::max(
        maximum_goal_distance - minimum_goal_distance,
        objective_priority_min_spread_);

      for (auto& objective : current_push_objectives) {
        objective.boundary_potential = std::clamp(
          max_near_goal_objective_potential_ *
          (maximum_goal_distance - objective.cow_goal_distance) /
          normalization_span,
          0.0,
          max_near_goal_objective_potential_);
      }

      // Reserve the best clipped/free cells for lagging cows when multiple
      // objectives project to the same part of the moving local grid.
      std::sort(
        current_push_objectives.begin(),
        current_push_objectives.end(),
        [](const PushObjective& lhs, const PushObjective& rhs) {
          if (
            std::abs(lhs.cow_goal_distance - rhs.cow_goal_distance) >
            POTENTIAL_DESCENT_EPSILON)
          {
            return lhs.cow_goal_distance > rhs.cow_goal_distance;
          }
          if (
            std::abs(lhs.global_point.x - rhs.global_point.x) >
            POTENTIAL_DESCENT_EPSILON)
          {
            return lhs.global_point.x < rhs.global_point.x;
          }
          return lhs.global_point.y < rhs.global_point.y;
        });
    }

    // A received cow snapshot is authoritative, including an empty one.
    // Objective memory is used only before any usable tracker snapshot exists.
    const std::vector<PushObjective>& active_push_objectives =
      has_authoritative_cow_evaluation ?
      current_push_objectives :
      remembered_push_objectives;

    // Project each global push destination into the intersection of the local
    // window and finite global map. Ray clipping preserves both its direction
    // and its cow-distance priority at an intermediate edge objective.
    for (const auto& push_objective : active_push_objectives) {
      cv::Point2f clipped_push_point;
      if (!clip_target_to_valid_region(
          grid_center,
          push_objective.global_point,
          clipped_push_point))
      {
        continue;
      }

      int desired_row = -1;
      int desired_col = -1;
      if (!global_to_grid(
          clipped_push_point.x,
          clipped_push_point.y,
          grid_center,
          desired_row,
          desired_col))
      {
        continue;
      }
      desired_row = std::clamp(desired_row, 1, GRID_SIZE - 2);
      desired_col = std::clamp(desired_col, 1, GRID_SIZE - 2);

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

      const double boundary_potential = std::clamp(
        push_objective.boundary_potential,
        0.0,
        max_near_goal_objective_potential_);
      const double normalized_priority_cost =
        max_near_goal_objective_potential_ >
          POTENTIAL_DESCENT_EPSILON ?
        boundary_potential / max_near_goal_objective_potential_ :
        0.0;

      grid[objective_row][objective_col] = PUSH_OBJECTIVE;
      potential_grid.at<double>(objective_row, objective_col) =
        boundary_potential;
      objective_source_cost.at<double>(objective_row, objective_col) =
        normalized_priority_cost * objective_priority_fallback_bias_m_;
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
      is_fixed, objective_mask, objective_source_cost);

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

    // Commit vectors together with the global center that defined their
    // coordinate frame. Never reinterpret an old raster at a new center.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (all_evaluated_cows_at_goal) {
        latest_vector_grid_ = vector_grid;
        latest_cell_grid_ = grid;
        latest_is_fixed_grid_ = is_fixed.clone();
        latest_saddle_escape_grid_ = saddle_escape_grid.clone();
        latest_grid_center_ = grid_center;
        latest_grid_offset_ = grid_center - global_map_center_;
        latest_valid_push_objectives_.clear();
        has_vector_grid_ = false;
        all_cows_at_goal_ = true;
        has_last_nonzero_move_direction_ = false;
      } else {
        // Never retain a completed-herd hold after a current cow is observed
        // outside the final goal. A valid previous field can remain as the
        // movement fallback if this particular relaxation cycle is invalid.
        all_cows_at_goal_ = false;
        if (candidate_field_is_valid) {
          latest_vector_grid_ = vector_grid;
          latest_cell_grid_ = grid;
          latest_is_fixed_grid_ = is_fixed.clone();
          latest_saddle_escape_grid_ = saddle_escape_grid.clone();
          latest_grid_center_ = grid_center;
          latest_grid_offset_ = grid_center - global_map_center_;
          if (!current_push_objectives.empty()) {
            latest_valid_push_objectives_ = current_push_objectives;
          }
          has_vector_grid_ = true;
        }
      }
    }

    // Render this center-aware candidate. Remembered global objectives make
    // empty detection updates regenerate at the current drone position.
    compute_and_display_occupancy_map(grid, grid_center);
    compute_and_display_vector_grid(
      grid, vector_grid, is_fixed, saddle_escape_grid, grid_center);
  }

  void draw_final_goal(
    cv::Mat& image,
    const cv::Point2f& grid_center) const
  {
    const double local_x = GOAL_POSITION.x - grid_center.x;
    const double local_y = GOAL_POSITION.y - grid_center.y;
    const bool goal_is_inside_local_grid =
      local_x >= -MAP_HALF_WIDTH &&
      local_x <= MAP_HALF_WIDTH &&
      local_y >= -MAP_HALF_WIDTH &&
      local_y <= MAP_HALF_WIDTH;

    // A moving local grid cannot show the true position of an off-screen
    // global goal. Keep a marker on the map edge along the ray toward it so
    // the final destination remains visible on every drone map.
    double displayed_local_x = local_x;
    double displayed_local_y = local_y;
    if (!goal_is_inside_local_grid) {
      const double marker_extent = MAP_HALF_WIDTH - GRID_CELL_SIZE;
      const double x_scale = std::abs(local_x) > marker_extent ?
        marker_extent / std::abs(local_x) : 1.0;
      const double y_scale = std::abs(local_y) > marker_extent ?
        marker_extent / std::abs(local_y) : 1.0;
      const double ray_scale = std::min(x_scale, y_scale);
      displayed_local_x *= ray_scale;
      displayed_local_y *= ray_scale;
    }

    const int pixel_x = std::clamp(
      static_cast<int>(std::lround(
        (displayed_local_x + MAP_HALF_WIDTH) /
        (2.0 * MAP_HALF_WIDTH) *
        (image.cols - 1))),
      0,
      image.cols - 1);
    const int pixel_y = std::clamp(
      static_cast<int>(std::lround(
        (MAP_HALF_WIDTH - displayed_local_y) /
        (2.0 * MAP_HALF_WIDTH) *
        (image.rows - 1))),
      0,
      image.rows - 1);
    const cv::Point center(pixel_x, pixel_y);
    const int radius = std::max(5, image.cols / 60);

    // Yellow distinguishes the fixed global cow destination from green
    // behind-cow push objectives.
    if (!goal_is_inside_local_grid) {
      const cv::Point image_center(image.cols / 2, image.rows / 2);
      const cv::Point2f direction(
        static_cast<float>(center.x - image_center.x),
        static_cast<float>(center.y - image_center.y));
      const float direction_length = std::hypot(direction.x, direction.y);
      if (direction_length > 0.0f) {
        const cv::Point2f unit_direction = direction / direction_length;
        const cv::Point arrow_start(
          cvRound(center.x - unit_direction.x * radius * 2.5f),
          cvRound(center.y - unit_direction.y * radius * 2.5f));
        cv::arrowedLine(
          image,
          arrow_start,
          center,
          cv::Scalar(0, 180, 180),
          2,
          cv::LINE_AA,
          0,
          0.45);
      }
    }

    cv::circle(
      image,
      center,
      radius,
      cv::Scalar(0, 255, 255),
      2,
      cv::LINE_AA);
    cv::line(
      image,
      center - cv::Point(radius, 0),
      center + cv::Point(radius, 0),
      cv::Scalar(0, 255, 255),
      1,
      cv::LINE_AA);
    cv::line(
      image,
      center - cv::Point(0, radius),
      center + cv::Point(0, radius),
      cv::Scalar(0, 255, 255),
      1,
      cv::LINE_AA);

    const std::string goal_label = goal_is_inside_local_grid ?
      "FINAL" :
      cv::format("FINAL %.1fm", std::hypot(local_x, local_y));
    const double font_scale = image.cols >= 600 ? 0.45 : 0.35;
    int text_baseline = 0;
    const cv::Size text_size = cv::getTextSize(
      goal_label,
      cv::FONT_HERSHEY_SIMPLEX,
      font_scale,
      1,
      &text_baseline);
    const cv::Point label_origin(
      std::clamp(
        center.x + radius + 3,
        2,
        std::max(2, image.cols - text_size.width - 2)),
      std::clamp(
        center.y - radius - 2,
        text_size.height + 2,
        std::max(text_size.height + 2, image.rows - text_baseline - 2)));
    cv::putText(
      image,
      goal_label,
      label_origin,
      cv::FONT_HERSHEY_SIMPLEX,
      font_scale,
      cv::Scalar(0, 160, 160),
      1,
      cv::LINE_AA);
  }

  // -------------------------------------------------------------
  // MODULAR FUNCTION 1: OCCUPANCY GRID RENDERING
  // -------------------------------------------------------------

  void compute_and_display_occupancy_map(
    const std::vector<std::vector<CellState>>& grid,
    const cv::Point2f& grid_center)
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
    cv::resize(
      display_img,
      enlarged_img,
      cv::Size(400, 400),
      0,
      0,
      cv::INTER_NEAREST);

    draw_final_goal(enlarged_img, grid_center);

    // GRID_SIZE is even, so draw the drone at the exact geometric center
    // instead of offsetting it into one of the four central cells.
    cv::circle(
      enlarged_img,
      cv::Point(enlarged_img.cols / 2, enlarged_img.rows / 2),
      6,
      cv::Scalar(255, 0, 0),
      cv::FILLED,
      cv::LINE_AA);
    const cv::Point2f global_offset = grid_center - global_map_center_;
    cv::putText(
      enlarged_img,
      cv::format(
        "global=(%.1f, %.1f) offset=(%.1f, %.1f)",
        grid_center.x,
        grid_center.y,
        global_offset.x,
        global_offset.y),
      cv::Point(8, 18),
      cv::FONT_HERSHEY_SIMPLEX,
      0.4,
      cv::Scalar(0, 0, 0),
      1,
      cv::LINE_AA);

    cv::imshow(occupancy_window_name_, enlarged_img);
    cv::waitKey(1);
  }

  // -------------------------------------------------------------
  // MODULAR FUNCTION 2: VECTOR GRID RENDERING
  // -------------------------------------------------------------

  void compute_and_display_vector_grid(
    const std::vector<std::vector<CellState>>& grid,
    const std::vector<std::vector<cv::Point2f>>& vector_grid,
    const cv::Mat& is_fixed,
    const cv::Mat& saddle_escape_grid,
    const cv::Point2f& grid_center)
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

    draw_final_goal(vector_img, grid_center);

    cv::circle(
      vector_img,
      cv::Point(vector_img.cols / 2, vector_img.rows / 2),
      7,
      cv::Scalar(255, 0, 0),
      cv::FILLED,
      cv::LINE_AA);
    const cv::Point2f global_offset = grid_center - global_map_center_;
    cv::putText(
      vector_img,
      cv::format(
        "global=(%.1f, %.1f) offset=(%.1f, %.1f)",
        grid_center.x,
        grid_center.y,
        global_offset.x,
        global_offset.y),
      cv::Point(8, 18),
      cv::FONT_HERSHEY_SIMPLEX,
      0.45,
      cv::Scalar(0, 0, 0),
      1,
      cv::LINE_AA);

    cv::imshow(vector_window_name_, vector_img);
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
    cv::Point2f grid_center;
    cv::Point2f last_move_direction;
    bool has_drone = false;
    bool has_cows = false;
    bool has_grid = false;
    bool has_last_move_direction = false;
    bool all_cows_at_goal = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      drone_pose = latest_drone_pose_;
      cows = latest_cows_;
      vector_grid = latest_vector_grid_;
      cell_grid = latest_cell_grid_;
      is_fixed_grid = latest_is_fixed_grid_.clone();
      grid_center = latest_grid_center_;
      last_move_direction = last_nonzero_move_direction_;
      has_drone = received_drone_;
      has_cows = received_cows_;
      has_grid = has_vector_grid_;
      has_last_move_direction = has_last_nonzero_move_direction_;
      all_cows_at_goal = all_cows_at_goal_;
    }

    if (!has_drone || !drone_pose) {
      return;
    }

    double drone_x = drone_pose->position.x;
    double drone_y = drone_pose->position.y;

    // 1. Goto Calculation. Hold position once every remembered cow is
    // inside the final goal radius; movement resumes if any cow leaves.
    if (all_cows_at_goal) {
      geometry_msgs::msg::Pose hold_msg = *drone_pose;
      goto_pub_->publish(hold_msg);
    } else if (
      has_grid &&
      vector_grid.size() == GRID_SIZE &&
      vector_grid[GRID_CENTER_ROW].size() == GRID_SIZE)
    {
      const int drone_row = GRID_CENTER_ROW;
      const int drone_col = GRID_CENTER_COL;
      cv::Point2f vector = vector_grid[drone_row][drone_col];
      float magnitude = std::hypot(vector.x, vector.y);

      const double bounded_x =
        std::clamp(drone_x, global_map_min_x_, global_map_max_x_);
      const double bounded_y =
        std::clamp(drone_y, global_map_min_y_, global_map_max_y_);
      const cv::Point2f boundary_recovery(
        static_cast<float>(bounded_x - drone_x),
        static_cast<float>(bounded_y - drone_y));
      const float boundary_recovery_magnitude =
        std::hypot(boundary_recovery.x, boundary_recovery.y);
      if (boundary_recovery_magnitude > 1.0e-6f) {
        vector = boundary_recovery / boundary_recovery_magnitude;
        magnitude = 1.0f;
      }

      const bool is_push_objective =
        !cell_grid.empty() &&
        cell_grid[drone_row][drone_col] == PUSH_OBJECTIVE;

      if (magnitude <= 0.5f && !is_push_objective) {
        cv::Point2f recovery_direction;
        if (
          !is_fixed_grid.empty() &&
          find_local_recovery_direction(
            drone_x,
            drone_y,
            drone_row,
            drone_col,
            vector_grid,
            is_fixed_grid,
            grid_center,
            recovery_direction))
        {
          vector = recovery_direction;
          magnitude = 1.0f;
        } else if (has_last_move_direction) {
          vector = last_move_direction;
          magnitude = std::hypot(vector.x, vector.y);
        }
      }

      cv::Point2f movement(0.0f, 0.0f);
      bool publish_goto = true;
      if (magnitude > 0.5f) {
        movement = cv::Point2f(
          (vector.x / magnitude) *
            static_cast<float>(MAX_GOTO_MOVEMENT),
          (vector.y / magnitude) *
            static_cast<float>(MAX_GOTO_MOVEMENT));
        {
          std::lock_guard<std::mutex> lock(mutex_);
          last_nonzero_move_direction_ = cv::Point2f(
            vector.x / magnitude,
            vector.y / magnitude);
          has_last_nonzero_move_direction_ = true;
        }
      } else if (!is_push_objective) {
        // Preserve the previously active nonzero waypoint instead of
        // publishing an accidental hold command.
        publish_goto = false;
      }

      if (publish_goto) {
        geometry_msgs::msg::Pose goto_msg;
        goto_msg.position.x = std::clamp(
          drone_x + movement.x,
          global_map_min_x_,
          global_map_max_x_);
        goto_msg.position.y = std::clamp(
          drone_y + movement.y,
          global_map_min_y_,
          global_map_max_y_);
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

  int drone_index_{0};
  int total_drones_{1};
  double global_map_min_x_{-50.0};
  double global_map_max_x_{50.0};
  double global_map_min_y_{-50.0};
  double global_map_max_y_{50.0};
  double cow_exclusion_radius_{DEFAULT_COW_EXCLUSION_RADIUS};
  double push_point_margin_{DEFAULT_PUSH_POINT_MARGIN};
  double max_near_goal_objective_potential_{
    DEFAULT_MAX_NEAR_GOAL_OBJECTIVE_POTENTIAL};
  double objective_priority_min_spread_{
    DEFAULT_OBJECTIVE_PRIORITY_MIN_SPREAD};
  double objective_priority_fallback_bias_m_{
    DEFAULT_OBJECTIVE_PRIORITY_FALLBACK_BIAS};
  cv::Point2f global_map_center_{0.0f, 0.0f};
  cv::Point2f latest_grid_center_{0.0f, 0.0f};
  cv::Point2f latest_grid_offset_{0.0f, 0.0f};
  std::vector<PushObjective> latest_valid_push_objectives_;
  std::string drone_namespace_;
  std::string grid_name_;
  std::string occupancy_window_name_;
  std::string vector_window_name_;

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
  bool has_last_nonzero_move_direction_{false};
  bool all_cows_at_goal_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OccupancyGridVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
