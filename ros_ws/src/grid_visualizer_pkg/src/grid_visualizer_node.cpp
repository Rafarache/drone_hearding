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
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "grid_visualizer_pkg/search_coverage.hpp"
#include "grid_visualizer_pkg/cow_evidence.hpp"
#include "cow_mapping_interfaces/msg/cow_observation.hpp"
#include "grid_visualizer_pkg/goal_quiet_zone.hpp"
#include "grid_visualizer_pkg/stall_recovery.hpp"
#include "grid_visualizer_pkg/distance_recovery.hpp"
#include <opencv2/opencv.hpp>

struct GridConfig
{
  std::string source_path;
  double map_half_width{0.0};
  double cell_size{0.0};
  int grid_size{0};
  int grid_center_row{0};
  int grid_center_col{0};
  double local_objective_half_extent{0.0};
  int relaxation_max_iterations{0};
  double relaxation_tolerance{0.0};
  double sor_relaxation_factor{0.0};
  double low_gradient_threshold{0.0};
  double potential_descent_epsilon{0.0};
  bool enable_saddle_neighbor_descent{false};
  bool enable_geodesic_field_fallback{false};
  int control_recovery_search_radius_cells{0};
  bool show_occupancy_grid{false};
  bool show_vector_grid{false};
  bool show_coverage_map{false};
  bool search_border_first{true};
  double stall_window_s{6},stall_motion_m{0.4},recovery_release_m{1};
  grid_visualizer_pkg::RecoverySettings recovery;
  double search_attraction_percent{60.0};
  double cow_attraction_percent{40.0};
  int visualization_period_ms{0};
  int control_period_ms{0};
  double free_cell_potential{0.0};
  double boundary_potential{0.0};
  cv::Point2f goal_position{0.0f, 0.0f};
  double final_goal_reached_radius{0.0};
  double goal_quiet_radius{0.0};
  double goal_quiet_duration_seconds{0.0};
  double max_goto_movement{0.0};
  double max_near_goal_objective_potential{0.0};
  double objective_priority_curve_exponent{0.0};
  double primary_objective_switch_margin{0.0};
  double primary_objective_association_radius{0.0};
  double objective_priority_min_spread{0.0};
  double objective_priority_fallback_bias{0.0};
};

template<typename T>
T read_required(const cv::FileStorage & storage, const char * key)
{
  const cv::FileNode node = storage[key];
  if (node.empty()) {
    throw std::invalid_argument(
      std::string("Missing grid configuration value: ") + key);
  }
  T value{};
  node >> value;
  return value;
}

GridConfig load_grid_config()
{
  GridConfig config;
  config.source_path =
    ament_index_cpp::get_package_share_directory("grid_visualizer_pkg") +
    "/config/grid_config.xml";

  cv::FileStorage storage(config.source_path, cv::FileStorage::READ);
  if (!storage.isOpened()) {
    throw std::runtime_error(
      "Unable to open grid configuration: " + config.source_path);
  }

  const double local_grid_extent =
    read_required<double>(storage, "local_grid_extent_m");
  config.cell_size = read_required<double>(storage, "cell_size_m");
  config.free_cell_potential =
    read_required<double>(storage, "free_cell_potential");
  config.boundary_potential =
    read_required<double>(storage, "boundary_potential");
  config.relaxation_max_iterations =
    read_required<int>(storage, "relaxation_max_iterations");
  config.relaxation_tolerance =
    read_required<double>(storage, "relaxation_tolerance");
  config.sor_relaxation_factor =
    read_required<double>(storage, "sor_relaxation_factor");
  config.low_gradient_threshold =
    read_required<double>(storage, "low_gradient_threshold");
  config.potential_descent_epsilon =
    read_required<double>(storage, "potential_descent_epsilon");
  config.enable_saddle_neighbor_descent =
    read_required<int>(storage, "enable_saddle_neighbor_descent") != 0;
  config.enable_geodesic_field_fallback =
    read_required<int>(storage, "enable_geodesic_field_fallback") != 0;
  config.control_recovery_search_radius_cells =
    read_required<int>(storage, "control_recovery_search_radius_cells");
  config.show_occupancy_grid =
    read_required<int>(storage, "show_occupancy_grid") != 0;
  config.show_vector_grid =
    read_required<int>(storage, "show_vector_grid") != 0;
  config.show_coverage_map =
    read_required<int>(storage, "show_coverage_map") != 0;
  config.search_border_first =
    read_required<int>(storage, "search_border_first") != 0;
  config.search_attraction_percent = read_required<double>(storage, "search_attraction_percent");
  config.cow_attraction_percent = read_required<double>(storage, "cow_attraction_percent");
  if (!std::isfinite(config.search_attraction_percent) ||
    !std::isfinite(config.cow_attraction_percent) ||
    config.search_attraction_percent < 0 || config.cow_attraction_percent < 0 ||
    std::abs(config.search_attraction_percent + config.cow_attraction_percent - 100.0) > 1e-6)
  {
    throw std::invalid_argument("Search and cow attraction percentages must be nonnegative and sum to 100");
  }
  config.stall_window_s=read_required<double>(storage,"stall_window_s");
  config.stall_motion_m=read_required<double>(storage,"stall_motion_m");
  config.recovery_release_m=read_required<double>(storage,"recovery_release_m");
  config.recovery.release_m=config.recovery_release_m;
  config.recovery.agreement_s=read_required<double>(storage,"recovery_agreement_s");
  config.recovery.alignment_deg=read_required<double>(storage,"recovery_alignment_deg");
  config.recovery.window_progress_m=read_required<double>(storage,"recovery_window_progress_m");
  config.recovery.watch_s=read_required<double>(storage,"recovery_watch_s");
  config.recovery.backtrack_m=read_required<double>(storage,"recovery_backtrack_m");
  config.recovery.opposition_s=read_required<double>(storage,"recovery_opposition_s");
  config.recovery.arrival_m=read_required<double>(storage,"recovery_arrival_m");
  for (double value : {config.recovery.agreement_s, config.recovery.window_progress_m,
      config.recovery.watch_s, config.recovery.backtrack_m, config.recovery.opposition_s,
      config.recovery.arrival_m}) {
    if (!std::isfinite(value) || value<=0) {throw std::invalid_argument("Invalid recovery threshold");}
  }
  if (!std::isfinite(config.recovery.alignment_deg) || config.recovery.alignment_deg<=0 ||
    config.recovery.alignment_deg>=90) {throw std::invalid_argument("Recovery alignment must be between 0 and 90 degrees");}
  if(!std::isfinite(config.stall_window_s) || !std::isfinite(config.stall_motion_m) ||
    !std::isfinite(config.recovery_release_m) || config.stall_window_s<=0 ||
    config.stall_motion_m<=0 || config.recovery_release_m<=config.stall_motion_m) {
    throw std::invalid_argument("Invalid stall recovery window or distances");
  }
  config.visualization_period_ms =
    read_required<int>(storage, "visualization_period_ms");
  config.control_period_ms =
    read_required<int>(storage, "control_period_ms");
  // Goal values are supplied by the authoritative simulation launch.
  config.final_goal_reached_radius = 2.0;
  config.goal_quiet_radius =
    read_required<double>(storage, "goal_quiet_radius_m");
  config.goal_quiet_duration_seconds =
    read_required<double>(storage, "goal_quiet_duration_seconds");
  config.max_goto_movement =
    read_required<double>(storage, "max_goto_movement_m");
  config.max_near_goal_objective_potential =
    read_required<double>(
      storage, "max_near_goal_objective_potential");
  config.objective_priority_curve_exponent =
    read_required<double>(storage, "objective_priority_curve_exponent");
  config.primary_objective_switch_margin =
    read_required<double>(storage, "primary_objective_switch_margin_m");
  config.primary_objective_association_radius =
    read_required<double>(
      storage, "primary_objective_association_radius_m");
  config.objective_priority_min_spread =
    read_required<double>(storage, "objective_priority_min_spread_m");
  config.objective_priority_fallback_bias =
    read_required<double>(storage, "objective_priority_fallback_bias_m");
  config.recovery.association_m=config.primary_objective_association_radius;
  config.recovery.max_step_m=config.max_goto_movement;
  config.recovery.epsilon=config.potential_descent_epsilon;

  if (
    !std::isfinite(local_grid_extent) ||
    !std::isfinite(config.cell_size) ||
    local_grid_extent <= 0.0 ||
    config.cell_size <= 0.0)
  {
    throw std::invalid_argument(
      "local_grid_extent_m and cell_size_m must be finite and positive");
  }

  const double exact_grid_size = local_grid_extent / config.cell_size;
  config.grid_size = static_cast<int>(std::llround(exact_grid_size));
  if (
    config.grid_size < 4 ||
    config.grid_size % 2 != 0 ||
    std::abs(exact_grid_size - config.grid_size) > 1.0e-9)
  {
    throw std::invalid_argument(
      "local_grid_extent_m must divide into an even number of cells");
  }

  config.map_half_width = local_grid_extent / 2.0;
  config.grid_center_row = config.grid_size / 2;
  config.grid_center_col = config.grid_size / 2;
  config.local_objective_half_extent =
    config.map_half_width - 1.5 * config.cell_size;

  if (
    config.relaxation_max_iterations <= 0 ||
    !std::isfinite(config.relaxation_tolerance) ||
    config.relaxation_tolerance <= 0.0 ||
    !std::isfinite(config.sor_relaxation_factor) ||
    config.sor_relaxation_factor <= 0.0 ||
    config.sor_relaxation_factor >= 2.0 ||
    !std::isfinite(config.low_gradient_threshold) ||
    config.low_gradient_threshold <= 0.0 ||
    !std::isfinite(config.potential_descent_epsilon) ||
    config.potential_descent_epsilon <= 0.0)
  {
    throw std::invalid_argument(
      "Grid relaxation values are outside their valid ranges");
  }

  if (
    config.control_recovery_search_radius_cells < 1 ||
    config.visualization_period_ms <= 0 ||
    config.control_period_ms <= 0 ||
    !std::isfinite(config.free_cell_potential) ||
    !std::isfinite(config.boundary_potential) ||
    config.boundary_potential <= config.free_cell_potential)
  {
    throw std::invalid_argument(
      "Grid timing, recovery, or potential values are invalid");
  }

  if (
    !std::isfinite(config.goal_position.x) ||
    !std::isfinite(config.goal_position.y) ||
    !std::isfinite(config.final_goal_reached_radius) ||
    config.final_goal_reached_radius <= 0.0 ||
    !std::isfinite(config.goal_quiet_radius) ||
    config.goal_quiet_radius <= config.final_goal_reached_radius ||
    !std::isfinite(config.goal_quiet_duration_seconds) ||
    config.goal_quiet_duration_seconds <= 0.0 ||
    !std::isfinite(config.max_goto_movement) ||
    config.max_goto_movement <= 0.0 ||
    !std::isfinite(config.max_near_goal_objective_potential) ||
    config.max_near_goal_objective_potential < 0.0 ||
    config.max_near_goal_objective_potential >= config.boundary_potential ||
    !std::isfinite(config.objective_priority_curve_exponent) ||
    config.objective_priority_curve_exponent <= 0.0 ||
    !std::isfinite(config.primary_objective_switch_margin) ||
    config.primary_objective_switch_margin < 0.0 ||
    !std::isfinite(config.primary_objective_association_radius) ||
    config.primary_objective_association_radius <= 0.0 ||
    !std::isfinite(config.objective_priority_min_spread) ||
    config.objective_priority_min_spread <= 0.0 ||
    !std::isfinite(config.objective_priority_fallback_bias) ||
    config.objective_priority_fallback_bias < 0.0)
  {
    throw std::invalid_argument(
      "Grid goal and objective-priority values are invalid");
  }

  return config;
}

class OccupancyGridVisualizer : public rclcpp::Node
{
public:
  OccupancyGridVisualizer()
  : Node("occupancy_grid_visualizer"), config_(load_grid_config())
  {
    config_.goal_position = cv::Point2f(
      declare_parameter<double>("goal_x", -10.0), declare_parameter<double>("goal_y", -10.0));
    config_.final_goal_reached_radius = declare_parameter<double>("final_goal_reached_radius", 2.0);
    if (!std::isfinite(config_.goal_position.x) || !std::isfinite(config_.goal_position.y) ||
      !std::isfinite(config_.final_goal_reached_radius) || config_.final_goal_reached_radius <= 0 ||
      config_.goal_quiet_radius <= config_.final_goal_reached_radius) {
      throw std::invalid_argument("Invalid goal parameters");
    }
    goal_quiet_zone_ = std::make_unique<grid_visualizer_pkg::GoalQuietZone>(
      grid_visualizer_pkg::CowPoint{config_.goal_position.x, config_.goal_position.y},
      config_.final_goal_reached_radius, config_.goal_quiet_radius, config_.goal_quiet_duration_seconds);
    latest_effective_goal_radius_ = config_.final_goal_reached_radius;
    config_.show_vector_grid = declare_parameter<bool>("show_vector_grid", config_.show_vector_grid);
    config_.show_occupancy_grid = declare_parameter<bool>("show_occupancy_grid", config_.show_occupancy_grid);
    config_.show_coverage_map = declare_parameter<bool>("show_coverage_map", config_.show_coverage_map);
    config_.search_border_first = declare_parameter<bool>("search_border_first", config_.search_border_first);
    search_near_ = declare_parameter<double>("search_near_m", 2.0);
    search_far_ = declare_parameter<double>("search_far_m", 10.0);
    search_fov_ = declare_parameter<double>("search_fov_rad", 2.09);
    search_planning_period_ = declare_parameter<double>("search_planning_period_s", 1.0);
    if (!std::isfinite(search_planning_period_) || search_planning_period_ <= 0) {
      throw std::runtime_error("search_planning_period_s must be finite and positive");
    }
    if (!std::isfinite(search_near_) || !std::isfinite(search_far_) ||
      !std::isfinite(search_fov_) || search_near_ < 0 || search_near_ >= 5 ||
      search_far_ <= 5 || search_fov_ <= 0 || search_fov_ > 2*std::acos(-1.0)) {
      throw std::invalid_argument("Search range must contain the 5 m observation viewpoint");
    }
    drone_index_ = this->declare_parameter<int>("drone_index", -1);
    total_drones_ = this->declare_parameter<int>("total_drones", 0);
    global_map_center_x_ = this->declare_parameter<double>(
      "global_map_center_x", std::numeric_limits<double>::quiet_NaN());
    global_map_center_y_ = this->declare_parameter<double>(
      "global_map_center_y", std::numeric_limits<double>::quiet_NaN());
    global_map_width_ = this->declare_parameter<double>(
      "global_map_width", -1.0);
    global_map_height_ = this->declare_parameter<double>(
      "global_map_height", -1.0);
    drone_namespace_prefix_ = this->declare_parameter<std::string>(
      "drone_namespace_prefix", "");
    cow_observations_topic_ = this->declare_parameter<std::string>(
      "cow_observations_topic", "/cow_observations");
    cow_exclusion_radius_ = this->declare_parameter<double>(
      "cow_exclusion_radius", -1.0);
    push_point_margin_ = this->declare_parameter<double>(
      "push_point_margin", -1.0);

    global_map_min_x_ = global_map_center_x_ - global_map_width_ / 2.0;
    global_map_max_x_ = global_map_center_x_ + global_map_width_ / 2.0;
    global_map_min_y_ = global_map_center_y_ - global_map_height_ / 2.0;
    global_map_max_y_ = global_map_center_y_ + global_map_height_ / 2.0;

    if (total_drones_ <= 0) {
      throw std::invalid_argument("total_drones must be greater than zero");
    }
    if (drone_index_ < 0 || drone_index_ >= total_drones_) {
      throw std::invalid_argument(
        "drone_index must be in the range [0, total_drones)");
    }
    if (
      !std::isfinite(global_map_center_x_) ||
      !std::isfinite(global_map_center_y_) ||
      !std::isfinite(global_map_width_) ||
      !std::isfinite(global_map_height_) ||
      global_map_width_ <= 0.0 ||
      global_map_height_ <= 0.0 ||
      drone_namespace_prefix_.empty() ||
      cow_observations_topic_.empty())
    {
      throw std::invalid_argument(
        "global map geometry and topic parameters are invalid");
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

    global_map_center_ = cv::Point2f(
      static_cast<float>(global_map_center_x_),
      static_cast<float>(global_map_center_y_));

    drone_namespace_ =
      std::string(drone_namespace_prefix_) + std::to_string(drone_index_);
    coverage_ = std::make_unique<grid_visualizer_pkg::SearchCoverage>(
      global_map_min_x_, global_map_min_y_, global_map_width_, global_map_height_, config_.cell_size);
    cow_evidence_ = std::make_unique<grid_visualizer_pkg::CowEvidence>(
      global_map_min_x_, global_map_min_y_, global_map_width_, global_map_height_,
      config_.cell_size, drone_index_, total_drones_);
    grid_name_ = "grid" + std::to_string(drone_index_);
    occupancy_window_name_ = "Occupancy Grid - " + grid_name_;
    vector_window_name_ = "Vector Movement Grid - " + grid_name_;
    coverage_window_name_ = "Visited Areas - " + drone_namespace_;

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
      "Goal quiet zone: %.2f m for %.2f s after a cow reaches %.2f m",
      config_.goal_quiet_radius,
      config_.goal_quiet_duration_seconds,
      config_.final_goal_reached_radius);
    RCLCPP_INFO(
      this->get_logger(),
      "Cow objective priority: max potential %.2f, minimum spread %.2f m, "
      "fallback bias %.2f m",
      config_.max_near_goal_objective_potential,
      config_.objective_priority_min_spread,
      config_.objective_priority_fallback_bias);
    RCLCPP_INFO(
      this->get_logger(),
      "Loaded grid configuration: %s (%d x %d cells, %.2f m/cell)",
      config_.source_path.c_str(), config_.grid_size, config_.grid_size,
      config_.cell_size);
    RCLCPP_INFO(this->get_logger(), "Drone Pose Sub Topic: %s", drone_gt_topic.c_str());
    RCLCPP_INFO(
      this->get_logger(), "Cow observations (no tracking): %s",
      cow_observations_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Drone Goto Pub Topic: %s", goto_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Drone Focus Pub Topic: %s", focus_topic.c_str());

    auto stop_qos = rclcpp::QoS(1).reliable().transient_local();
    stop_sub_ = create_subscription<std_msgs::msg::Empty>("/simulation/stop", stop_qos,
      [this](std_msgs::msg::Empty::SharedPtr) {
        stopped_ = true; has_vector_grid_ = false;
        stall_recovery_.reset(); distance_recovery_.reset(); commanded_movement_={0,0};
      });
    auto coverage_qos = rclcpp::QoS(50).reliable().durability_volatile();
    searched_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      drone_namespace_ + "/searched_area", coverage_qos);
    observation_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      drone_namespace_ + "/processed_view", coverage_qos,
      [this](geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
        if (apply_observation(*msg) && !stopped_) {
          has_own_observation_ = true;
          searched_pub_->publish(*msg);
          record_view_observation();
        }
      });
    for (int i = 0; i < total_drones_; ++i) {
      if (i == drone_index_) {continue;}
      coverage_subs_.push_back(create_subscription<geometry_msgs::msg::PolygonStamped>(
        drone_namespace_prefix_ + std::to_string(i) + "/searched_area", coverage_qos,
        [this](geometry_msgs::msg::PolygonStamped::SharedPtr msg) {apply_observation(*msg);}));
    }
    // Initialize subscriptions
    drone_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      drone_gt_topic,
      10,
      std::bind(&OccupancyGridVisualizer::drone_callback, this, std::placeholders::_1)
    );

    latest_peer_drone_poses_.resize(total_drones_);
    for (int peer_index = 0; peer_index < total_drones_; ++peer_index) {
      if (peer_index == drone_index_) {
        continue;
      }

      const std::string peer_topic =
        std::string(drone_namespace_prefix_) +
        std::to_string(peer_index) + "/gt_pose";
      peer_drone_subs_.push_back(
        this->create_subscription<geometry_msgs::msg::Pose>(
          peer_topic,
          10,
          [this, peer_index](
            const geometry_msgs::msg::Pose::SharedPtr msg)
          {
            peer_drone_callback(msg, peer_index);
          }));
      RCLCPP_INFO(
        this->get_logger(),
        "Peer drone obstacle topic: %s",
        peer_topic.c_str());
    }

    cow_sub_ = this->create_subscription<cow_mapping_interfaces::msg::CowObservation>(
      cow_observations_topic_,
      50,
      std::bind(&OccupancyGridVisualizer::cow_callback, this, std::placeholders::_1)
    );

    // Initialize publishers
    goto_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(goto_topic, 10);
    focusin_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(focus_topic, 10);

    // Initialize the configured visualization timer.
    timer_5hz_ = this->create_wall_timer(
      std::chrono::milliseconds(config_.visualization_period_ms),
      std::bind(&OccupancyGridVisualizer::timer_5hz_callback, this)
    );

    // Initialize the configured movement-control timer.
    timer_2hz_ = this->create_wall_timer(
      std::chrono::milliseconds(config_.control_period_ms),
      std::bind(&OccupancyGridVisualizer::timer_2hz_callback, this)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Node initialized with UI period %d ms and control period %d ms.",
      config_.visualization_period_ms, config_.control_period_ms);
  }

private:
  enum CellState {
    EMPTY = 0,
    DRONE = 1,
    COW = 2,
    EXCLUSION = 3,
    PUSH_OBJECTIVE = 4,
    SEARCH_OBJECTIVE = 5,
    SEARCHED = 6
  };

  struct PushObjective
  {
    cv::Point2f global_point;
    double cow_goal_distance{0.0};
    double boundary_potential{0.0};
    double strength{0.0};
    bool is_search{false};
    cv::Point2f look_at{0,0};
    cv::Point2f cow_identity{0,0};
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

  void peer_drone_callback(
    const geometry_msgs::msg::Pose::SharedPtr msg,
    int peer_index)
  {
    if (
      peer_index < 0 ||
      peer_index >= total_drones_ ||
      peer_index == drone_index_ ||
      !std::isfinite(msg->position.x) ||
      !std::isfinite(msg->position.y))
    {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_peer_drone_poses_[peer_index] = msg;
  }

  bool apply_observation(const geometry_msgs::msg::PolygonStamped& msg)
  {
    if (msg.header.frame_id != "map" || msg.polygon.points.size() < 3) {return false;}
    std::vector<grid_visualizer_pkg::SearchPoint> polygon;
    for (const auto& p : msg.polygon.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {return false;}
      polygon.push_back({p.x, p.y});
    }
    last_observation_gain_ = coverage_->observe(polygon);
    return true;
  }

  void record_view_observation()
  {
    if (!latest_drone_pose_) {return;}
    const auto& p = *latest_drone_pose_;
    const double yaw = std::atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),
      1-2*(p.orientation.y*p.orientation.y+p.orientation.z*p.orientation.z));
    for (const auto& view : latest_search_views_) {
      if (grid_visualizer_pkg::distance({p.position.x,p.position.y},view.position) > 0.75) {continue;}
      const double heading = std::atan2(view.look_at.y-p.position.y,view.look_at.x-p.position.x);
      if (std::abs(grid_visualizer_pkg::angle_difference(heading,yaw)) > 0.15) {continue;}
      int key = coverage_->index(view.look_at);
      int& misses = failed_observations_[key];
      const auto remaining=coverage_->patch_remaining(view.look_at,search_near_,search_far_,search_fov_);
      auto& previous=patch_previous_remaining_[key];
      misses = remaining==0 || remaining<previous ? 0 : misses+1;
      previous=remaining;
      if (misses >= 3) {
        failed_views_.push_back({view.position,now().seconds()+10.0});
        misses = 0;
      }
    }
  }

  void cow_callback(const cow_mapping_interfaces::msg::CowObservation::SharedPtr msg)
  {
    if (stopped_ || msg->header.frame_id != "map" ||
      msg->source_drone_index >= static_cast<unsigned>(total_drones_)) {return;}
    const auto& q=msg->camera_pose.orientation;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if (!std::isfinite(norm) || norm < 1e-12) {return;}
    grid_visualizer_pkg::EvidenceFrame frame;
    frame.source=static_cast<int>(msg->source_drone_index);
    frame.stamp_ns=static_cast<int64_t>(msg->header.stamp.sec)*1000000000LL+msg->header.stamp.nanosec;
    frame.x=msg->camera_pose.position.x;
    frame.y=msg->camera_pose.position.y;
    frame.yaw=std::atan2(2*(q.w*q.z+q.x*q.y),norm-2*(q.y*q.y+q.z*q.z));
    frame.near=msg->near_range_m;
    frame.far=msg->far_range_m;
    frame.fov=msg->horizontal_fov_rad;
    for (const auto& d : msg->detections) {
      frame.detections.push_back({d.position.x,d.position.y,d.position_valid,
        d.bearing_min_rad,d.bearing_max_rad});
    }
    std::lock_guard<std::mutex> lock(mutex_);
    cow_evidence_->advance_time(now().nanoseconds());
    cow_evidence_->enqueue(std::move(frame));
  }

  void update_cow_evidence()
  {
    // Called under mutex on the map timer, before taking the navigation snapshot.
    cow_evidence_->advance_time(now().nanoseconds());
    cow_evidence_->update();
    auto cows=std::make_shared<geometry_msgs::msg::PoseArray>();
    cows->header.frame_id="map";
    cows->header.stamp=now();
    for (const auto& p : cow_evidence_->active_positions()) {
      geometry_msgs::msg::Pose pose;
      pose.position.x=p.first;
      pose.position.y=p.second;
      pose.orientation.w=1;
      cows->poses.push_back(pose);
    }
    latest_cows_=cows;
    received_cows_=true;
  }

  // -------------------------------------------------------------
  // HELPER MAPPING FUNCTIONS
  // -------------------------------------------------------------

  bool local_to_grid(double x, double y, int& row, int& col) const
  {
    if (
      !std::isfinite(x) ||
      !std::isfinite(y) ||
      x < -config_.map_half_width ||
      x > config_.map_half_width ||
      y < -config_.map_half_width ||
      y > config_.map_half_width)
    {
      return false;
    }

    const double norm_x =
      (x + config_.map_half_width) / (2.0 * config_.map_half_width);
    const double norm_y =
      (config_.map_half_width - y) / (2.0 * config_.map_half_width);

    col = static_cast<int>(std::floor(norm_x * config_.grid_size));
    row = static_cast<int>(std::floor(norm_y * config_.grid_size));

    col = std::clamp(col, 0, config_.grid_size - 1);
    row = std::clamp(row, 0, config_.grid_size - 1);
    return true;
  }

  cv::Point2f grid_to_local(int row, int col) const
  {
    const float norm_x = (col + 0.5f) / config_.grid_size;
    const float norm_y = (row + 0.5f) / config_.grid_size;

    const float x =
      norm_x * (2.0f * config_.map_half_width) - config_.map_half_width;
    const float y =
      config_.map_half_width - norm_y * (2.0f * config_.map_half_width);
    return cv::Point2f(x, y);
  }

  cv::Point2f grid_to_global(
    int row,
    int col,
    const cv::Point2f& grid_center) const
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
      static_cast<double>(grid_center.x) - config_.local_objective_half_extent,
      global_map_min_x_);
    const double valid_max_x = std::min(
      static_cast<double>(grid_center.x) + config_.local_objective_half_extent,
      global_map_max_x_);
    const double valid_min_y = std::max(
      static_cast<double>(grid_center.y) - config_.local_objective_half_extent,
      global_map_min_y_);
    const double valid_max_y = std::min(
      static_cast<double>(grid_center.y) + config_.local_objective_half_extent,
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
      std::abs(direction_x) <= config_.potential_descent_epsilon &&
      std::abs(direction_y) <= config_.potential_descent_epsilon)
    {
      clipped_target = grid_center;
      return true;
    }

    double scale = 1.0;
    if (direction_x > config_.potential_descent_epsilon) {
      scale = std::min(
        scale, (valid_max_x - grid_center.x) / direction_x);
    } else if (direction_x < -config_.potential_descent_epsilon) {
      scale = std::min(
        scale, (valid_min_x - grid_center.x) / direction_x);
    }
    if (direction_y > config_.potential_descent_epsilon) {
      scale = std::min(
        scale, (valid_max_y - grid_center.y) / direction_y);
    } else if (direction_y < -config_.potential_descent_epsilon) {
      scale = std::min(
        scale, (valid_min_y - grid_center.y) / direction_y);
    }

    scale = std::clamp(scale, 0.0, 1.0);
    if (scale <= config_.potential_descent_epsilon) {
      return false;
    }

    clipped_target = cv::Point2f(
      static_cast<float>(grid_center.x + scale * direction_x),
      static_cast<float>(grid_center.y + scale * direction_y));
    return
      std::isfinite(clipped_target.x) &&
      std::isfinite(clipped_target.y);
  }

  bool find_nearest_free_interior_cell(
    const cv::Mat& is_fixed,
    int desired_row,
    int desired_col,
    int& result_row,
    int& result_col) const
  {
    double best_distance_sq = std::numeric_limits<double>::infinity();
    result_row = -1;
    result_col = -1;

    for (int r = 1; r < config_.grid_size - 1; ++r) {
      for (int c = 1; c < config_.grid_size - 1; ++c) {
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

  grid_visualizer_pkg::RecoveryGrid recovery_grid(
    const cv::Mat& fixed, const cv::Point2f& center) const
  {
    grid_visualizer_pkg::RecoveryGrid result;
    result.rows=fixed.rows; result.cols=fixed.cols; result.cell_size=config_.cell_size;
    result.center={center.x,center.y};
    result.blocked.reserve(fixed.total());
    for(int r=0;r<fixed.rows;++r) {for(int c=0;c<fixed.cols;++c) {
      result.blocked.push_back(fixed.at<uchar>(r,c));
    }}
    return result;
  }

  cv::Mat compute_goal_distance_grid(
    const cv::Mat& is_fixed, const cv::Mat& objective_mask,
    const cv::Mat& objective_source_cost) const
  {
    const auto frame=recovery_grid(is_fixed,{0,0});
    std::vector<std::pair<int,double>> sources;
    for(int r=0;r<frame.rows;++r) {for(int c=0;c<frame.cols;++c) {
      if(objective_mask.at<uchar>(r,c)) {
        sources.emplace_back(r*frame.cols+c,objective_source_cost.at<double>(r,c));
      }
    }}
    const auto costs=grid_visualizer_pkg::recovery_distances(frame,sources,config_.potential_descent_epsilon);
    cv::Mat result(frame.rows,frame.cols,CV_64F);
    std::copy(costs.begin(),costs.end(),result.ptr<double>());
    return result;
  }

  bool find_local_recovery_direction(
    double drone_x,
    double drone_y,
    int drone_row,
    int drone_col,
    const std::vector<std::vector<cv::Point2f>>& vector_grid,
    const cv::Mat& is_fixed,
    const cv::Point2f& grid_center,
    cv::Point2f& recovery_direction) const
  {
    double best_distance_sq = std::numeric_limits<double>::infinity();
    int best_row = -1;
    int best_col = -1;

    for (int radius = 1;
      radius <= config_.control_recovery_search_radius_cells;
      ++radius)
    {
      const int min_row = std::max(1, drone_row - radius);
      const int max_row = std::min(config_.grid_size - 2, drone_row + radius);
      const int min_col = std::max(1, drone_col - radius);
      const int max_col = std::min(config_.grid_size - 2, drone_col + radius);

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
    if (magnitude <= config_.potential_descent_epsilon) {
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
    // Keep the coverage view responsive before takeoff and after mission stop.
    update_coverage_map_window();
    if (stopped_) {return;}
    geometry_msgs::msg::Pose::SharedPtr drone_pose;
    geometry_msgs::msg::PoseArray::SharedPtr cows;
    std::vector<geometry_msgs::msg::Pose::SharedPtr> peer_drone_poses;
    cv::Point2f remembered_primary_push_objective;
    bool has_remembered_primary_push_objective = false;
    bool has_drone = false;
    bool has_cows = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      update_cow_evidence();
      drone_pose = latest_drone_pose_;
      cows = latest_cows_;
      peer_drone_poses = latest_peer_drone_poses_;
      remembered_primary_push_objective =
        latest_primary_push_objective_;
      has_remembered_primary_push_objective =
        has_primary_push_objective_;
      has_drone = received_drone_;
      has_cows = received_cows_;
    }

    if (!has_drone || !drone_pose) {
      return;
    }

    if (!startup_heading_initialized_) {
      if (total_drones_>1) {
        std::vector<grid_visualizer_pkg::SearchPoint> initial_peers;
        for (int i=0;i<total_drones_;++i) {
          if(i==drone_index_) {continue;}
          if(!peer_drone_poses[i]) {return;}
          initial_peers.push_back({peer_drone_poses[i]->position.x,peer_drone_poses[i]->position.y});
        }
        startup_direction_=grid_visualizer_pkg::separation_direction(
          {drone_pose->position.x,drone_pose->position.y},initial_peers,drone_index_,total_drones_);
        startup_heading_pending_=true;
      }
      startup_heading_initialized_=true;
    }

    const cv::Point2f grid_center(
      static_cast<float>(drone_pose->position.x),
      static_cast<float>(drone_pose->position.y));

    std::vector<grid_visualizer_pkg::CowPoint> cow_points;
    if (has_cows && cows) {
      cow_points.reserve(cows->poses.size());
      for (const auto & cow_pose : cows->poses) {
        if (is_inside_global_map(cow_pose.position.x, cow_pose.position.y)) {
          cow_points.push_back({cow_pose.position.x, cow_pose.position.y});
        }
      }
    }
    const double effective_goal_radius = goal_quiet_zone_->update(
      cow_points, grid_visualizer_pkg::GoalQuietZone::Clock::now());

    std::vector<std::vector<CellState>> grid(
      config_.grid_size,
      std::vector<CellState>(config_.grid_size, EMPTY));
    cv::Mat potential_grid(
      config_.grid_size, config_.grid_size, CV_64F, cv::Scalar(config_.free_cell_potential));
    cv::Mat is_fixed(
      config_.grid_size, config_.grid_size, CV_8UC1, cv::Scalar(0));
    cv::Mat absorption(config_.grid_size, config_.grid_size, CV_64F, cv::Scalar(0));
    cv::Mat objective_mask(
      config_.grid_size, config_.grid_size, CV_8UC1, cv::Scalar(0));
    cv::Mat objective_source_cost(
      config_.grid_size, config_.grid_size, CV_64F, cv::Scalar(0.0));
    std::vector<PushObjective> current_push_objectives;

    // The local perimeter closes the relaxation problem. Cells whose global
    // centers lie outside the finite map are also unavailable.
    for (int row = 0; row < config_.grid_size; ++row) {
      for (int col = 0; col < config_.grid_size; ++col) {
        const cv::Point2f global_cell =
          grid_to_global(row, col, grid_center);
        if (coverage_->seen({global_cell.x, global_cell.y})) {grid[row][col] = SEARCHED;}
        const bool is_local_perimeter =
          row == 0 ||
          row == config_.grid_size - 1 ||
          col == 0 ||
          col == config_.grid_size - 1;
        if (
          is_local_perimeter ||
          !is_inside_global_map(global_cell.x, global_cell.y))
        {
          grid[row][col] = EXCLUSION;
          potential_grid.at<double>(row, col) = config_.boundary_potential;
          is_fixed.at<uchar>(row, col) = 1;
        }
      }
    }

    // Project active global confidence cells into this moving local window.
    if (has_cows && cows) {
      for (const auto& cow_pose : cows->poses) {
        const float cow_x = cow_pose.position.x;
        const float cow_y = cow_pose.position.y;
        if (!is_inside_global_map(cow_x, cow_y)) {
          continue;
        }

        int cow_row = 0;
        int cow_col = 0;
        if (global_to_grid(
            cow_x, cow_y, grid_center, cow_row, cow_col))
        {
          grid[cow_row][cow_col] = COW;
        }

        // A cow outside the visible window can still affect cells where its
        // exclusion circle overlaps the window.
        for (int row = 0; row < config_.grid_size; ++row) {
          for (int col = 0; col < config_.grid_size; ++col) {
            const cv::Point2f global_cell =
              grid_to_global(row, col, grid_center);
            const double distance = std::hypot(
              global_cell.x - cow_x,
              global_cell.y - cow_y);
            if (distance <= cow_exclusion_radius_) {
              if (grid[row][col] != COW) {
                grid[row][col] = EXCLUSION;
              }
              potential_grid.at<double>(row, col) = config_.boundary_potential;
              is_fixed.at<uchar>(row, col) = 1;
            }
          }
        }

        const cv::Point2f cow_point(cow_x, cow_y);
        const cv::Point2f direction_to_goal =
          config_.goal_position - cow_point;
        const float goal_distance = std::hypot(
          direction_to_goal.x,
          direction_to_goal.y);

        // The goal radius widens for five seconds after a cow arrives.
        // Cows inside the active radius cannot create push objectives.
        if (goal_distance > effective_goal_radius) {
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
            PushObjective objective;
            objective.global_point=global_push_point;
            objective.cow_goal_distance=goal_distance;
            objective.cow_identity={cow_x,cow_y};
            current_push_objectives.push_back(objective);
          }
        }
      }
    }

    // Peer drone poses stay in global coordinates and are reprojected into
    // this drone's moving grid. For now, each peer occupies one fixed cell
    // with no surrounding radius.
    for (int peer_index = 0;
      peer_index < static_cast<int>(peer_drone_poses.size());
      ++peer_index)
    {
      if (peer_index == drone_index_ || !peer_drone_poses[peer_index]) {
        continue;
      }

      const auto& peer_pose = peer_drone_poses[peer_index];
      if (!is_inside_global_map(
          peer_pose->position.x,
          peer_pose->position.y))
      {
        continue;
      }

      int peer_row = 0;
      int peer_col = 0;
      if (!global_to_grid(
          peer_pose->position.x,
          peer_pose->position.y,
          grid_center,
          peer_row,
          peer_col))
      {
        continue;
      }

      grid[peer_row][peer_col] = DRONE;
      potential_grid.at<double>(peer_row, peer_col) = config_.boundary_potential;
      is_fixed.at<uchar>(peer_row, peer_col) = 1;
    }

    cv::Point2f current_primary_push_objective;
    bool has_current_primary_push_objective = false;
    if (!current_push_objectives.empty()) {
      const auto distance_limits = std::minmax_element(
        current_push_objectives.begin(),
        current_push_objectives.end(),
        [this](const PushObjective& lhs, const PushObjective& rhs) {
          return lhs.cow_goal_distance < rhs.cow_goal_distance;
        });
      const double minimum_goal_distance =
        distance_limits.first->cow_goal_distance;
      const double maximum_goal_distance =
        distance_limits.second->cow_goal_distance;
      const double normalization_span = std::max(
        maximum_goal_distance - minimum_goal_distance,
        config_.objective_priority_min_spread);

      // Select a deterministic lagging cow, then retain that choice while it
      // stays within the switch margin of the furthest cow. This prevents
      // equal left/right objectives from exchanging priority every frame.
      std::size_t primary_index = 0;
      for (std::size_t index = 1;
        index < current_push_objectives.size();
        ++index)
      {
        const auto& candidate = current_push_objectives[index];
        const auto& selected = current_push_objectives[primary_index];
        const bool candidate_is_farther =
          candidate.cow_goal_distance >
          selected.cow_goal_distance + config_.potential_descent_epsilon;
        const bool equal_distance_with_stable_order =
          std::abs(
            candidate.cow_goal_distance -
            selected.cow_goal_distance) <= config_.potential_descent_epsilon &&
          (
            candidate.global_point.x < selected.global_point.x ||
            (
              candidate.global_point.x == selected.global_point.x &&
              candidate.global_point.y < selected.global_point.y));
        if (candidate_is_farther || equal_distance_with_stable_order) {
          primary_index = index;
        }
      }

      if (has_remembered_primary_push_objective) {
        double nearest_distance =
          config_.primary_objective_association_radius;
        std::size_t associated_index = current_push_objectives.size();
        for (std::size_t index = 0;
          index < current_push_objectives.size();
          ++index)
        {
          const auto& objective = current_push_objectives[index];
          const double distance = std::hypot(
            objective.global_point.x -
              remembered_primary_push_objective.x,
            objective.global_point.y -
              remembered_primary_push_objective.y);
          if (distance <= nearest_distance) {
            nearest_distance = distance;
            associated_index = index;
          }
        }

        if (
          associated_index < current_push_objectives.size() &&
          current_push_objectives[associated_index].cow_goal_distance >=
            maximum_goal_distance - config_.primary_objective_switch_margin)
        {
          primary_index = associated_index;
        }
      }

      for (std::size_t index = 0;
        index < current_push_objectives.size();
        ++index)
      {
        auto& objective = current_push_objectives[index];
        const double normalized_priority = std::clamp(
          (maximum_goal_distance - objective.cow_goal_distance) /
          normalization_span,
          0.0,
          1.0);
        objective.boundary_potential =
          config_.max_near_goal_objective_potential *
          std::pow(
            normalized_priority,
            config_.objective_priority_curve_exponent);

        if (index == primary_index) {
          objective.boundary_potential = 0.0;
        } else if (
          objective.cow_goal_distance >=
            maximum_goal_distance - config_.primary_objective_switch_margin)
        {
          objective.boundary_potential =
            config_.max_near_goal_objective_potential;
        }
      }

      current_primary_push_objective =
        current_push_objectives[primary_index].global_point;
      has_current_primary_push_objective = true;

      // Reserve the best clipped/free cells for lagging cows when multiple
      // objectives project to the same part of the moving local grid.
      std::sort(
        current_push_objectives.begin(),
        current_push_objectives.end(),
        [this](const PushObjective& lhs, const PushObjective& rhs) {
          if (
            std::abs(lhs.cow_goal_distance - rhs.cow_goal_distance) >
            config_.potential_descent_epsilon)
          {
            return lhs.cow_goal_distance > rhs.cow_goal_distance;
          }
          if (
            std::abs(lhs.global_point.x - rhs.global_point.x) >
            config_.potential_descent_epsilon)
          {
            return lhs.global_point.x < rhs.global_point.x;
          }
          return lhs.global_point.y < rhs.global_point.y;
        });
    }

    // Search and pushing coexist: build positive scores for both groups.
    std::vector<grid_visualizer_pkg::SearchPoint> peers, cow_locations;
    for (const auto& peer : peer_drone_poses) {
      if (peer) {peers.push_back({peer->position.x, peer->position.y});}
    }
    if (cows) {for (const auto& cow : cows->poses) {
      cow_locations.push_back({cow.position.x, cow.position.y});
    }}
    auto blocked = [&](grid_visualizer_pkg::SearchPoint p) {
      if (!is_inside_global_map(p.x, p.y)) {return true;}
      for (auto cow : cow_locations) {
        if (grid_visualizer_pkg::distance(p, cow) <= cow_exclusion_radius_) {return true;}
      }
      for (auto peer : peers) {
        if (grid_visualizer_pkg::distance(p, peer) <= config_.cell_size) {return true;}
      }
      return false;
    };
    const double sim_now = now().seconds();
    failed_views_.erase(std::remove_if(failed_views_.begin(), failed_views_.end(),
      [sim_now](const auto& failure) {return sim_now >= failure.second;}), failed_views_.end());
    auto retired = [&](auto p) {return std::any_of(failed_views_.begin(), failed_views_.end(),
      [&](const auto& failure) {return grid_visualizer_pkg::distance(p,failure.first)<config_.cell_size;});};
    // Keep local obstacle/field updates at 5 Hz; global destination selection is
    // much more expensive and only needs to run at the observation cadence.
    if (has_own_observation_ && (latest_search_views_.empty() || !search_plan_initialized_ || sim_now < last_search_plan_ ||
      sim_now-last_search_plan_ >= search_planning_period_)) {
      if (latest_search_views_.empty() || blocked(latest_search_views_.front().position) ||
        retired(latest_search_views_.front().position) || coverage_->patch_remaining(
          latest_search_views_.front().look_at,search_near_,search_far_,search_fov_)==0) {
        latest_search_views_ = coverage_->views(
        {grid_center.x, grid_center.y}, peers, blocked, search_near_, search_far_, search_fov_,
        1, retired, config_.search_border_first,
        first_search_selected_ ? nullptr : &startup_direction_);
        if(!latest_search_views_.empty()) {
          first_search_selected_=true;
          const auto& view=latest_search_views_.front();
          RCLCPP_INFO(get_logger(),
            "search viewpoint: patch=(%.2f,%.2f) attraction=(%.2f,%.2f) from=(%.2f,%.2f)",
            view.look_at.x,view.look_at.y,view.position.x,view.position.y,grid_center.x,grid_center.y);
        }
      }
      last_search_plan_ = sim_now;
      search_plan_initialized_ = true;
    }
    auto search_views = latest_search_views_;
    // Moving cows/peers and newly covered/retired targets invalidate cached
    // destinations immediately, without waiting for the next global plan.
    search_views.erase(std::remove_if(search_views.begin(), search_views.end(),
      [&](const auto& view) {
        return blocked(view.position) || retired(view.position) || coverage_->patch_remaining(view.look_at,search_near_,search_far_,search_fov_)==0;
      }), search_views.end());
    active_search_max_depth_ = search_views.empty() ? std::numeric_limits<double>::infinity() :
      search_views.front().max_border_depth;
    // Fixed configured split; unused attraction budget goes to the available task.
    const bool searching = !search_views.empty();
    const bool herding = !current_push_objectives.empty();
    search_weight_ = searching ? (herding ? config_.search_attraction_percent/100.0 : 1.0) : 0.0;
    herd_weight_ = herding ? (searching ? config_.cow_attraction_percent/100.0 : 1.0) : 0.0;
    double search_sum = 0, herd_sum = 0;
    for (const auto& view : search_views) {search_sum += view.score;}
    for (const auto& objective : current_push_objectives) {
      double priority = config_.max_near_goal_objective_potential > 0 ?
        1-objective.boundary_potential/config_.max_near_goal_objective_potential : 1;
      herd_sum += (0.1+0.9*priority)/(1+cv::norm(objective.global_point-grid_center)/search_far_);
    }
    std::vector<PushObjective> active_push_objectives;
    for (auto objective : current_push_objectives) {
      double priority = config_.max_near_goal_objective_potential > 0 ?
        1-objective.boundary_potential/config_.max_near_goal_objective_potential : 1;
      double score = (0.1+0.9*priority)/(1+cv::norm(objective.global_point-grid_center)/search_far_);
      objective.strength = herd_sum > 0 ? herd_weight_*score/herd_sum : 0;
      active_push_objectives.push_back(objective);
    }
    latest_search_views_ = search_views;
    for (const auto& view : search_views) {
      PushObjective objective;
      objective.global_point = cv::Point2f(view.position.x, view.position.y);
      objective.strength = search_sum > 0 ? search_weight_*view.score/search_sum : 0;
      objective.is_search = true;
      objective.look_at = cv::Point2f(view.look_at.x, view.look_at.y);
      active_push_objectives.push_back(objective);
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "coverage=%.1f%% search=%.2f herd=%.2f objectives=%zu/%zu",
      100*coverage_->fraction(), search_weight_, herd_weight_, search_views.size(), current_push_objectives.size());

    // Project each global destination into the current local window without
    // modifying the cow map or the real global push destination. This temporary
    // edge objective is regenerated when the drone searches somewhere else.
    std::vector<grid_visualizer_pkg::RecoveryTarget> recovery_targets;
    for (const auto& push_objective : active_push_objectives) {
      // A disabled attraction group must not seed recovery or a phantom objective.
      if(push_objective.strength<=0) {continue;}
      cv::Point2f clipped_push_point;
      if (!clip_target_to_valid_region(
          grid_center,
          push_objective.global_point,
          clipped_push_point))
      {
        continue;
      }

      // A clipped intermediate waypoint is navigation, not a claimed observation.
      const bool is_final_view = cv::norm(clipped_push_point-push_objective.global_point) < config_.cell_size;
      if (push_objective.is_search && is_final_view &&
        coverage_->gain({clipped_push_point.x, clipped_push_point.y},
          std::atan2(push_objective.look_at.y-clipped_push_point.y, push_objective.look_at.x-clipped_push_point.x),
          search_near_, search_far_, search_fov_) == 0) {continue;}
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
      desired_row = std::clamp(desired_row, 1, config_.grid_size - 2);
      desired_col = std::clamp(desired_col, 1, config_.grid_size - 2);

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

      if (push_objective.is_search && is_final_view) {
        auto placed = grid_to_global(objective_row,objective_col,grid_center);
        if (coverage_->gain({placed.x,placed.y},
          std::atan2(push_objective.look_at.y-placed.y,push_objective.look_at.x-placed.x),
          search_near_,search_far_,search_fov_) == 0) {continue;}
      }
      const double budget = 32.0*std::pow(config_.cell_size/0.5, 2);
      absorption.at<double>(objective_row, objective_col) = std::min(2*budget,
        absorption.at<double>(objective_row, objective_col)+budget*push_objective.strength);
      grid[objective_row][objective_col] = push_objective.is_search ? SEARCH_OBJECTIVE : PUSH_OBJECTIVE;
      objective_source_cost.at<double>(objective_row, objective_col) =
        config_.objective_priority_fallback_bias*(1-std::min(1.0,
          absorption.at<double>(objective_row, objective_col)/budget));
      objective_mask.at<uchar>(objective_row, objective_col) = 1;
      grid_visualizer_pkg::RecoveryTarget target;
      target.position={push_objective.global_point.x,push_objective.global_point.y};
      target.identity={push_objective.cow_identity.x,push_objective.cow_identity.y};
      target.search=push_objective.is_search;
      target.patch_id=target.search ? coverage_->index(
        {push_objective.look_at.x,push_objective.look_at.y}) : -1;
      target.cell=objective_row*config_.grid_size+objective_col;
      target.strength=push_objective.strength;
      target.source_cost=config_.objective_priority_fallback_bias*(1-std::min(1.0,target.strength));
      recovery_targets.push_back(target);
    }

    const bool has_push_objective =
      has_own_observation_ && cv::countNonZero(objective_mask) > 0;

    // Laplace between targets, positive absorption at targets: one scalar solve.
    bool converged = false;
    if (has_push_objective) {
      for (int iter = 0; iter < config_.relaxation_max_iterations; ++iter) {
        double max_change = 0.0;
        for (int r = 1; r < config_.grid_size - 1; ++r) {
          for (int c = 1; c < config_.grid_size - 1; ++c) {
            if (is_fixed.at<uchar>(r, c) == 1) {
              continue;
            }

            const double old_value = potential_grid.at<double>(r, c);
            const double neighbor_average = grid_visualizer_pkg::absorbing_value(
              potential_grid.at<double>(r, c - 1) +
              potential_grid.at<double>(r, c + 1) +
              potential_grid.at<double>(r - 1, c) +
              potential_grid.at<double>(r + 1, c), absorption.at<double>(r,c));
            const double new_value = old_value +
              config_.sor_relaxation_factor * (neighbor_average - old_value);

            potential_grid.at<double>(r, c) = new_value;
            max_change = std::max(max_change, std::abs(new_value - old_value));
          }
        }

        if (max_change < config_.relaxation_tolerance) {
          double residual = 0;
          for (int r=1; r<config_.grid_size-1; ++r) {for (int c=1; c<config_.grid_size-1; ++c) {
            if (is_fixed.at<uchar>(r,c)) {continue;}
            double sum=potential_grid.at<double>(r-1,c)+potential_grid.at<double>(r+1,c)+
              potential_grid.at<double>(r,c-1)+potential_grid.at<double>(r,c+1);
            residual=std::max(residual, std::abs(potential_grid.at<double>(r,c)-
              grid_visualizer_pkg::absorbing_value(sum,absorption.at<double>(r,c))));
          }}
          converged = residual < config_.relaxation_tolerance;
          if (converged) {break;}
        }
      }
    }

    // Store unit movement directions. When the centered gradient cancels at a
    // saddle, descend to the steepest lower neighbor in the same potential grid.
    std::vector<std::vector<cv::Point2f>> vector_grid(
      config_.grid_size,
      std::vector<cv::Point2f>(
        config_.grid_size, cv::Point2f(0.0f, 0.0f)));
    cv::Mat saddle_escape_grid(
      config_.grid_size, config_.grid_size, CV_8UC1, cv::Scalar(0));
    const cv::Mat goal_distance_grid = compute_goal_distance_grid(
      is_fixed, objective_mask, objective_source_cost);

    if (has_push_objective) {
      for (int r = 1; r < config_.grid_size - 1; ++r) {
        for (int c = 1; c < config_.grid_size - 1; ++c) {
          if (is_fixed.at<uchar>(r, c) == 1) {
            continue;
          }

          const double left = potential_grid.at<double>(r, c - 1);
          const double right = potential_grid.at<double>(r, c + 1);
          const double up = potential_grid.at<double>(r - 1, c);
          const double down = potential_grid.at<double>(r + 1, c);
          const double vector_x =
            (left - right) / (2.0 * config_.cell_size);
          const double vector_y =
            (down - up) / (2.0 * config_.cell_size);
          const double magnitude = std::hypot(vector_x, vector_y);

          if (magnitude >= config_.low_gradient_threshold) {
            vector_grid[r][c] = cv::Point2f(
              static_cast<float>(vector_x / magnitude),
              static_cast<float>(vector_y / magnitude));
            continue;
          }

          if (!config_.enable_saddle_neighbor_descent) {
            if (!config_.enable_geodesic_field_fallback) {
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
                  neighbor_distance + config_.potential_descent_epsilon <
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
          double best_descent = config_.potential_descent_epsilon;
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
              const double neighbor_distance = config_.cell_size *
                std::hypot(
                  static_cast<double>(row_offset),
                  static_cast<double>(col_offset));
              const double descent =
                (current_potential - neighbor_potential) /
                neighbor_distance;

              const bool stronger_descent =
                descent >
                best_descent + config_.potential_descent_epsilon;
              const bool deterministic_tie =
                std::abs(descent - best_descent) <=
                  config_.potential_descent_epsilon &&
                descent > config_.potential_descent_epsilon &&
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
      has_push_objective && converged &&
      usable_vector_count > 0 &&
      cv::checkRange(potential_grid);

    // Commit this cycle only: never continue toward obsolete search/cow targets.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_primary_push_objective_ = has_current_primary_push_objective;
      if (has_current_primary_push_objective) {latest_primary_push_objective_ = current_primary_push_objective;}
      latest_effective_goal_radius_ = effective_goal_radius;
      latest_recovery_targets_ = std::move(recovery_targets);
      latest_vector_grid_ = vector_grid;
      latest_cell_grid_ = grid;
      latest_is_fixed_grid_ = is_fixed.clone();
      latest_saddle_escape_grid_ = saddle_escape_grid.clone();
      latest_grid_center_ = grid_center;
      has_vector_grid_ = candidate_field_is_valid;
      has_no_objectives_ = !has_push_objective;
    }

    // Render this cycle at its current global center.
    if (config_.show_occupancy_grid) {
      compute_and_display_occupancy_map(grid, grid_center);
    }
    if (config_.show_vector_grid) {
      compute_and_display_vector_grid(
        grid, vector_grid, is_fixed, saddle_escape_grid, grid_center);
    }
  }

  void update_coverage_map_window()
  {
    // Read the parameter each cycle so each drone can toggle its view at runtime.
    if (!get_parameter("show_coverage_map").as_bool()) {
      if (coverage_window_open_) {
        cv::destroyWindow(coverage_window_name_);
        cv::waitKey(1);
        coverage_window_open_ = false;
      }
      return;
    }

    const auto display_now = std::chrono::steady_clock::now();
    if (coverage_window_open_ && display_now-last_coverage_display_ < std::chrono::seconds(1)) {
      cv::waitKey(1);  // Keep the window responsive between image updates.
      return;
    }
    last_coverage_display_ = display_now;
    const cv::Vec3b visited_color(190, 225, 190);
    if (coverage_cells_.empty() || rendered_coverage_count_ != coverage_->count()) {
      coverage_cells_ = cv::Mat(coverage_->rows(), coverage_->cols(), CV_8UC3, cv::Scalar(255, 255, 255));
      for (int row = 0; row < coverage_cells_.rows; ++row) {
        for (int col = 0; col < coverage_cells_.cols; ++col) {
          if (coverage_->seen_cell(row, col)) {
            // Coverage rows increase in world Y; display north at the top.
            coverage_cells_.at<cv::Vec3b>(coverage_cells_.rows - 1 - row, col) = visited_color;
          }
        }
      }
      rendered_coverage_count_ = coverage_->count();
    }

    constexpr int max_extent = 600;
    const double scale = max_extent / std::max(global_map_width_, global_map_height_);
    const int width = std::max(1, cvRound(global_map_width_ * scale));
    const int height = std::max(1, cvRound(global_map_height_ * scale));
    constexpr int margin = 16;
    constexpr int top = 58;
    cv::Mat image(height + top + 64, std::max(500, width + 2 * margin),
      CV_8UC3, cv::Scalar(245, 245, 245));
    const cv::Rect plot(margin, top, width, height);
    cv::Mat scaled_cells;
    cv::resize(coverage_cells_, scaled_cells, plot.size(), 0, 0, cv::INTER_NEAREST);
    scaled_cells.copyTo(image(plot));
    cv::rectangle(image, plot, cv::Scalar(80, 80, 80), 1);
    if (!latest_search_views_.empty()) {
      const auto& view=latest_search_views_.front();
      auto pixel=[&](double x,double y) {
        return cv::Point(plot.x+cvRound((x-global_map_min_x_)/global_map_width_*(width-1)),
          plot.y+cvRound((global_map_max_y_-y)/global_map_height_*(height-1)));
      };
      const int step=coverage_->patch_step(search_near_,search_far_,search_fov_);
      const int id=coverage_->index(view.look_at);
      if(id>=0) {
        const int row=(id/coverage_->cols()/step)*step,col=(id%coverage_->cols()/step)*step;
        const double x=global_map_min_x_+col*config_.cell_size;
        const double y=global_map_min_y_+row*config_.cell_size;
        cv::rectangle(image,pixel(x,y),pixel(std::min(global_map_max_x_,x+step*config_.cell_size),
          std::min(global_map_max_y_,y+step*config_.cell_size)),cv::Scalar(255,120,0),2);
        cv::circle(image,pixel(view.position.x,view.position.y),3,cv::Scalar(255,120,0),cv::FILLED);
      }
    }
    // These are detector/tracker positions, not Gazebo ground-truth cows.
    geometry_msgs::msg::PoseArray::SharedPtr detected_cows;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      detected_cows = latest_cows_;
    }
    if (detected_cows) {
      for (const auto& cow : detected_cows->poses) {
        if (!is_inside_global_map(cow.position.x, cow.position.y)) {continue;}
        const int x = plot.x + cvRound(
          (cow.position.x - global_map_min_x_) / global_map_width_ * (width - 1));
        const int y = plot.y + cvRound(
          (global_map_max_y_ - cow.position.y) / global_map_height_ * (height - 1));
        cv::circle(image, cv::Point(x, y), 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
      }
    }
    cv::putText(image,
      cv::format("%s | visited %.1f%%", drone_namespace_.c_str(), 100 * coverage_->fraction()),
      cv::Point(margin, 23), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(30, 30, 30), 1, cv::LINE_AA);
    cv::putText(image, cv::format("Whole map %.1f x %.1f m | +Y up, +X right",
        global_map_width_, global_map_height_),
      cv::Point(margin, 44), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
    cv::putText(image, cv::format("X [%.1f, %.1f]  Y [%.1f, %.1f]",
        global_map_min_x_, global_map_max_x_, global_map_min_y_, global_map_max_y_),
      cv::Point(margin, top + height + 22), cv::FONT_HERSHEY_SIMPLEX, 0.4,
      cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
    const int legend_y = top + height + 37;
    cv::rectangle(image, cv::Rect(margin, legend_y, 14, 14), cv::Scalar(190, 225, 190), cv::FILLED);
    cv::putText(image, "Visited (own + received)", cv::Point(margin + 20, legend_y + 12),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(30, 30, 30), 1, cv::LINE_AA);
    cv::rectangle(image, cv::Rect(margin + 230, legend_y, 14, 14), cv::Scalar(255, 255, 255), cv::FILLED);
    cv::putText(image, "Unvisited", cv::Point(margin + 250, legend_y + 12),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(30, 30, 30), 1, cv::LINE_AA);
    cv::circle(image, cv::Point(margin + 357, legend_y + 7), 4,
      cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    cv::putText(image, "Detected cows", cv::Point(margin + 370, legend_y + 12),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(30, 30, 30), 1, cv::LINE_AA);

    cv::imshow(coverage_window_name_, image);
    coverage_window_open_ = true;
    cv::waitKey(1);
  }

  void draw_off_map_cows(cv::Mat& image, const cv::Point2f& grid_center)
  {
    geometry_msgs::msg::PoseArray::SharedPtr cows;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cows = latest_cows_;
    }
    if (!cows) {return;}
    for (const auto& cow : cows->poses) {
      const cv::Point2f global_cow(cow.position.x, cow.position.y);
      const cv::Point2f offset = global_cow - grid_center;
      if (std::abs(offset.x) <= config_.map_half_width &&
        std::abs(offset.y) <= config_.map_half_width) {continue;}
      cv::Point2f intermediate;
      if (!clip_target_to_valid_region(grid_center, global_cow, intermediate)) {continue;}
      const cv::Point2f local = intermediate - grid_center;
      const cv::Point marker(
        cvRound((local.x + config_.map_half_width) / (2 * config_.map_half_width) * (image.cols - 1)),
        cvRound((config_.map_half_width - local.y) / (2 * config_.map_half_width) * (image.rows - 1)));
      // A hollow marker is only a visual proxy. It is not a cow obstacle at
      // this position, and never replaces the cow's true global coordinates.
      cv::circle(image, marker, 5, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
      const std::string label = cv::format("OFF COW %.1fm", cv::norm(offset));
      const double font_scale = image.cols >= 600 ? 0.35 : 0.30;
      int baseline = 0;
      const cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
        font_scale, 1, &baseline);
      const cv::Point origin(
        std::clamp(marker.x + 8, 2, std::max(2, image.cols - text_size.width - 2)),
        std::clamp(marker.y - 8, text_size.height + 2,
          std::max(text_size.height + 2, image.rows - baseline - 2)));
      cv::putText(image, label, origin, cv::FONT_HERSHEY_SIMPLEX,
        font_scale, cv::Scalar(0, 0, 210), 1, cv::LINE_AA);
    }
  }

  void draw_final_goal(
    cv::Mat& image,
    const cv::Point2f& grid_center) const
  {
    const double local_x = config_.goal_position.x - grid_center.x;
    const double local_y = config_.goal_position.y - grid_center.y;
    const bool goal_is_inside_local_grid =
      local_x >= -config_.map_half_width &&
      local_x <= config_.map_half_width &&
      local_y >= -config_.map_half_width &&
      local_y <= config_.map_half_width;

    // A moving local grid cannot show the true position of an off-screen
    // global goal. Keep a marker on the map edge along the ray toward it so
    // the final destination remains visible on every drone map.
    double displayed_local_x = local_x;
    double displayed_local_y = local_y;
    if (!goal_is_inside_local_grid) {
      const double marker_extent = config_.map_half_width - config_.cell_size;
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
        (displayed_local_x + config_.map_half_width) /
        (2.0 * config_.map_half_width) *
        (image.cols - 1))),
      0,
      image.cols - 1);
    const int pixel_y = std::clamp(
      static_cast<int>(std::lround(
        (config_.map_half_width - displayed_local_y) /
        (2.0 * config_.map_half_width) *
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
    cv::Mat display_img(config_.grid_size, config_.grid_size, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int r = 0; r < config_.grid_size; ++r) {
      for (int c = 0; c < config_.grid_size; ++c) {
        if (grid[r][c] == SEARCHED) {
          display_img.at<cv::Vec3b>(r,c) = cv::Vec3b(235,235,215);
        } else if (grid[r][c] == SEARCH_OBJECTIVE) {
          display_img.at<cv::Vec3b>(r,c) = cv::Vec3b(0,180,255);
        } else if (grid[r][c] == DRONE) {
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
    draw_off_map_cows(enlarged_img, grid_center);

    // config_.grid_size is even, so draw the drone at the exact geometric center
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
    const float cell_size = static_cast<float>(CANVAS_SIZE) / config_.grid_size;

    cv::Mat vector_img(CANVAS_SIZE, CANVAS_SIZE, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int r = 0; r < config_.grid_size; ++r) {
      for (int c = 0; c < config_.grid_size; ++c) {
        float center_x = (c + 0.5f) * cell_size;
        float center_y = (r + 0.5f) * cell_size;
        cv::Point2f center(center_x, center_y);

        if (grid[r][c] == SEARCHED || grid[r][c] == SEARCH_OBJECTIVE) {
          cv::rectangle(vector_img, cv::Point(c*cell_size,r*cell_size),
            cv::Point((c+1)*cell_size,(r+1)*cell_size),
            grid[r][c] == SEARCH_OBJECTIVE ? cv::Scalar(0,180,255) : cv::Scalar(235,235,215), cv::FILLED);
        }
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
    draw_off_map_cows(vector_img, grid_center);

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

    auto pixel=[&](double x,double y) {
        return cv::Point(cvRound((x-grid_center.x+config_.map_half_width)/(2*config_.map_half_width)*CANVAS_SIZE),
          cvRound((config_.map_half_width-y+grid_center.y)/(2*config_.map_half_width)*CANVAS_SIZE));
      };
    const auto phase=distance_recovery_.phase();
    const char* label=phase==grid_visualizer_pkg::RecoveryPhase::recovering ? "RECOVERY" :
      (phase==grid_visualizer_pkg::RecoveryPhase::watching ? "HANDOFF WATCH" : "RELAXATION");
    const auto& target=distance_recovery_.target();
    const cv::Scalar command_color(180,0,180);
    if(cv::norm(commanded_movement_)>1e-6) {
      cv::arrowedLine(vector_img,pixel(command_origin_.x,command_origin_.y),
        pixel(command_origin_.x+commanded_movement_.x,command_origin_.y+commanded_movement_.y),
        command_color,3,cv::LINE_AA,0,0.4);
    }
    if(target) {
      cv::Point2f displayed;
      if(clip_target_to_valid_region(grid_center,
          {static_cast<float>(target->position.x),static_cast<float>(target->position.y)},displayed)) {
        cv::circle(vector_img,pixel(displayed.x,displayed.y),8,command_color,2,cv::LINE_AA);
      }
    }
    cv::putText(vector_img,cv::format("%s%s | magenta: command / retained target",label,
        target ? (target->search ? " SEARCH" : " COW") : ""),
      cv::Point(8,CANVAS_SIZE-10),cv::FONT_HERSHEY_SIMPLEX,0.40,command_color,1,cv::LINE_AA);
    cv::imshow(vector_window_name_, vector_img);
    cv::waitKey(1);
  }

  // -------------------------------------------------------------
  // 2Hz TIMER CALLBACK (MOVEMENT & FOCUS CONTROL)
  // -------------------------------------------------------------

  void timer_2hz_callback()
  {
    if (stopped_) {return;}
    geometry_msgs::msg::Pose::SharedPtr drone_pose;
    geometry_msgs::msg::PoseArray::SharedPtr cows;
    std::vector<std::vector<cv::Point2f>> vector_grid;
    cv::Mat is_fixed_grid;
    std::vector<grid_visualizer_pkg::RecoveryTarget> recovery_targets;
    cv::Point2f grid_center;
    bool has_drone = false;
    bool has_grid = false;
    bool has_no_objectives = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      drone_pose = latest_drone_pose_;
      cows = latest_cows_;
      vector_grid = latest_vector_grid_;
      is_fixed_grid = latest_is_fixed_grid_.clone();
      recovery_targets = latest_recovery_targets_;
      grid_center = latest_grid_center_;
      has_drone = received_drone_;
      has_grid = has_vector_grid_;
      has_no_objectives = has_no_objectives_;
    }

    if (!has_drone || !drone_pose) {
      return;
    }

    double drone_x = drone_pose->position.x;
    double drone_y = drone_pose->position.y;

    if (!startup_heading_initialized_) {goto_pub_->publish(*drone_pose);return;}
    if (startup_heading_pending_) {
      double yaw=std::atan2(2*(drone_pose->orientation.w*drone_pose->orientation.z+
        drone_pose->orientation.x*drone_pose->orientation.y),
        1-2*(drone_pose->orientation.y*drone_pose->orientation.y+
          drone_pose->orientation.z*drone_pose->orientation.z));
      double desired=std::atan2(startup_direction_.y,startup_direction_.x);
      if(std::abs(grid_visualizer_pkg::angle_difference(desired,yaw))>0.15) {
        geometry_msgs::msg::Pose focus=*drone_pose;
        focus.position.x+=10*startup_direction_.x;focus.position.y+=10*startup_direction_.y;
        focusin_pub_->publish(focus);goto_pub_->publish(*drone_pose);return;
      }
      startup_heading_pending_=false;
    }

    // Pose and all field data above belong to one locked snapshot. The drone
    // may have moved away from that snapshot's geometric center in the meantime.
    const auto frame=recovery_grid(is_fixed_grid,grid_center);
    const grid_visualizer_pkg::RecoveryPoint position{drone_x,drone_y};
    const int index=frame.valid() ? frame.index(position) : -1;
    const bool safe_cell=index>=0 && frame.free(index/frame.cols,index%frame.cols);
    cv::Point2f normal(0,0);
    if(safe_cell && vector_grid.size()==static_cast<std::size_t>(frame.rows) &&
      vector_grid[index/frame.cols].size()==static_cast<std::size_t>(frame.cols)) {
      normal=vector_grid[index/frame.cols][index%frame.cols];
    }
    const double normal_length=std::hypot(normal.x,normal.y);
    if(!std::isfinite(normal_length) || (normal_length>0.5 &&
      !grid_visualizer_pkg::recovery_segment_free(frame,position,
        {drone_x+normal.x*config_.max_goto_movement/normal_length,
          drone_y+normal.y*config_.max_goto_movement/normal_length}))) {normal={0,0};}
    // Only search viewpoints intentionally hold for an observation. Reaching
    // a cow-pushing waypoint does not complete herding: the field must keep
    // controlling motion, and a non-moving cow approach must remain stall-eligible.
    const bool intentional_search_hold=std::any_of(recovery_targets.begin(),recovery_targets.end(),
      [&](const auto& target) {
        return target.search && grid_visualizer_pkg::recovery_distance(position,target.position)<=config_.recovery.arrival_m;
      });
    const double sim_now=now().seconds();
    const bool stalled=stall_recovery_.update(sim_now,drone_x,drone_y,
      !has_no_objectives && !intentional_search_hold &&
      distance_recovery_.phase()!=grid_visualizer_pkg::RecoveryPhase::recovering,
      config_.stall_window_s,config_.stall_motion_m);
    grid_visualizer_pkg::RecoveryResult recovery;
    if(has_no_objectives || !config_.enable_geodesic_field_fallback) {
      stall_recovery_.reset();distance_recovery_.reset();
    } else {
      const auto previous_target=distance_recovery_.target();
      recovery=distance_recovery_.update(sim_now,position,frame,recovery_targets,
        intentional_search_hold ? grid_visualizer_pkg::RecoveryPoint{0,0} :
        grid_visualizer_pkg::RecoveryPoint{normal.x,normal.y},
        has_grid,stalled || (!has_grid && !intentional_search_hold),config_.recovery);
      for(auto event:recovery.events) {
        const bool previous=event==grid_visualizer_pkg::RecoveryEvent::invalidated ||
          event==grid_visualizer_pkg::RecoveryEvent::arrived ||
          event==grid_visualizer_pkg::RecoveryEvent::watch_ended ||
          event==grid_visualizer_pkg::RecoveryEvent::clock_reset;
        const auto& target=previous ? previous_target : distance_recovery_.target();
        const char* trigger=recovery.reason ? recovery.reason :
          (stalled ? "stall" : (!has_grid ? "invalid_field" : "progress"));
        RCLCPP_INFO(get_logger(),
          "distance recovery: event=%s trigger=%s target=%s(%.2f,%.2f) direction_available=%d",
          grid_visualizer_pkg::recovery_event_name(event),trigger,
          target ? (target->search ? "search" : "cow") : "none",
          target ? target->position.x : 0.0,target ? target->position.y : 0.0,recovery.has_step);
        if(event==grid_visualizer_pkg::RecoveryEvent::arrived ||
          event==grid_visualizer_pkg::RecoveryEvent::handed_off) {stall_recovery_.reset();}
      }
    }

    cv::Point2f movement(0,0);
    const double bounded_x=std::clamp(drone_x,global_map_min_x_,global_map_max_x_);
    const double bounded_y=std::clamp(drone_y,global_map_min_y_,global_map_max_y_);
    const cv::Point2f boundary_recovery(bounded_x-drone_x,bounded_y-drone_y);
    const double boundary_distance=cv::norm(boundary_recovery);
    if(boundary_distance>1e-6 && !has_no_objectives) {
      movement=boundary_recovery*static_cast<float>(std::min(1.0,config_.max_goto_movement/boundary_distance));
    } else if(!has_no_objectives && safe_cell) {
      if(recovery.active) {
        if(recovery.has_step) {movement={static_cast<float>(recovery.movement.x),static_cast<float>(recovery.movement.y)};}
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
          "distance recovery: active=1 direction_available=%d",recovery.has_step);
      } else if(has_grid && !intentional_search_hold) {
        float magnitude=std::hypot(normal.x,normal.y);
        // Preserve the ordinary low-gradient behavior outside retained recovery.
        if(magnitude<=0.5f && find_local_recovery_direction(drone_x,drone_y,
            index/frame.cols,index%frame.cols,vector_grid,is_fixed_grid,grid_center,normal)) {
          magnitude=1;
        }
        if(magnitude>0.5f) {movement=normal*static_cast<float>(config_.max_goto_movement/magnitude);}
      }
    }
    if(boundary_distance<=1e-6 && !grid_visualizer_pkg::recovery_segment_free(frame,position,
        {drone_x+movement.x,drone_y+movement.y})) {movement={0,0};}
    geometry_msgs::msg::Pose goto_msg=*drone_pose;
    goto_msg.position.x=std::clamp(drone_x+movement.x,global_map_min_x_,global_map_max_x_);
    goto_msg.position.y=std::clamp(drone_y+movement.y,global_map_min_y_,global_map_max_y_);
    // Never let the final boundary clamp turn a hold into an unbounded jump.
    if(boundary_distance>1e-6) {
      goto_msg.position.x=drone_x+movement.x;goto_msg.position.y=drone_y+movement.y;
    }
    command_origin_={static_cast<float>(drone_x),static_cast<float>(drone_y)};
    commanded_movement_={static_cast<float>(goto_msg.position.x-drone_x),static_cast<float>(goto_msg.position.y-drone_y)};
    goto_pub_->publish(goto_msg);

    // Camera priority is independent of navigation's 60/40 attraction split.
    // Compare the actual search attraction with the eligible local cow center.
    // Search attention looks at the patch, not the navigation viewpoint.
    double center_x=0,center_y=0;
    std::size_t local_cows=0;
    if(cows) {for(const auto& cow:cows->poses) {
      const double x=cow.position.x,y=cow.position.y;
      if(!std::isfinite(x) || !std::isfinite(y) || !is_inside_global_map(x,y)) {continue;}
      if(std::abs(x-drone_x)>config_.map_half_width ||
        std::abs(y-drone_y)>config_.map_half_width) {continue;}
      if(std::hypot(x-config_.goal_position.x,y-config_.goal_position.y)<=
        config_.final_goal_reached_radius) {continue;}
      center_x+=x;center_y+=y;++local_cows;
    }}
    geometry_msgs::msg::Pose focus=*drone_pose;
    const bool search_closer=!latest_search_views_.empty() &&
      (local_cows==0 || std::hypot(latest_search_views_.front().position.x-drone_x,
          latest_search_views_.front().position.y-drone_y)<
        std::hypot(center_x/local_cows-drone_x,center_y/local_cows-drone_y));
    if(search_closer) {
      focus.position.x=latest_search_views_.front().look_at.x;
      focus.position.y=latest_search_views_.front().look_at.y;
    } else if(local_cows>0) {
      focus.position.x=center_x/local_cows;
      focus.position.y=center_y/local_cows;
    } else {
      // Preserve heading when neither a local cow nor a search patch exists.
      const double yaw=std::atan2(2*(drone_pose->orientation.w*drone_pose->orientation.z+
        drone_pose->orientation.x*drone_pose->orientation.y),
        1-2*(drone_pose->orientation.y*drone_pose->orientation.y+
          drone_pose->orientation.z*drone_pose->orientation.z));
      focus.position.x=drone_x+10*std::cos(yaw);
      focus.position.y=drone_y+10*std::sin(yaw);
    }
    focusin_pub_->publish(focus);
  }

  // Subscriptions, Publishers, Timers & Mutex
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr drone_sub_;
  std::vector<
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr>
    peer_drone_subs_;
  rclcpp::Subscription<cow_mapping_interfaces::msg::CowObservation>::SharedPtr cow_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goto_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr focusin_pub_;

  rclcpp::TimerBase::SharedPtr timer_5hz_;
  rclcpp::TimerBase::SharedPtr timer_2hz_;

  GridConfig config_;
  std::unique_ptr<grid_visualizer_pkg::GoalQuietZone> goal_quiet_zone_;
  std::unique_ptr<grid_visualizer_pkg::SearchCoverage> coverage_;
  std::unique_ptr<grid_visualizer_pkg::CowEvidence> cow_evidence_;
  std::vector<grid_visualizer_pkg::SearchView> latest_search_views_;
  std::vector<std::pair<grid_visualizer_pkg::SearchPoint,double>> failed_views_;
  std::map<int,int> failed_observations_;
  std::size_t last_observation_gain_{0};
  bool has_own_observation_{false};
  double search_near_{2},search_far_{10},search_fov_{2.09};
  double search_planning_period_{1},last_search_plan_{0};
  bool search_plan_initialized_{false};
  std::vector<grid_visualizer_pkg::RecoveryTarget> latest_recovery_targets_;
  grid_visualizer_pkg::StallRecovery stall_recovery_;
  grid_visualizer_pkg::DistanceRecovery distance_recovery_;
  cv::Point2f commanded_movement_{0,0};
  cv::Point2f command_origin_{0,0};
  double search_weight_{0},herd_weight_{0};
  bool startup_heading_initialized_{false},startup_heading_pending_{false},first_search_selected_{false};
  grid_visualizer_pkg::SearchPoint startup_direction_{0,0};
  std::map<int,std::size_t> patch_previous_remaining_;
  double active_search_max_depth_{std::numeric_limits<double>::infinity()};
  bool stopped_{false};
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr observation_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr> coverage_subs_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr searched_pub_;
  double latest_effective_goal_radius_{0.0};
  int drone_index_{-1};
  int total_drones_{0};
  double global_map_center_x_{0.0};
  double global_map_center_y_{0.0};
  double global_map_width_{0.0};
  double global_map_height_{0.0};
  double global_map_min_x_{0.0};
  double global_map_max_x_{0.0};
  double global_map_min_y_{0.0};
  double global_map_max_y_{0.0};
  double cow_exclusion_radius_{0.0};
  double push_point_margin_{0.0};
  std::string drone_namespace_prefix_;
  std::string cow_observations_topic_;
  cv::Point2f global_map_center_{0.0f, 0.0f};
  cv::Point2f latest_grid_center_{0.0f, 0.0f};
  cv::Point2f latest_grid_offset_{0.0f, 0.0f};
  cv::Point2f latest_primary_push_objective_{0.0f, 0.0f};
  bool has_primary_push_objective_{false};
  std::string drone_namespace_;
  std::string grid_name_;
  std::string occupancy_window_name_;
  std::string vector_window_name_;
  std::string coverage_window_name_;
  bool coverage_window_open_{false};
  cv::Mat coverage_cells_;
  std::size_t rendered_coverage_count_{0};
  std::chrono::steady_clock::time_point last_coverage_display_{};

  std::mutex mutex_;
  geometry_msgs::msg::Pose::SharedPtr latest_drone_pose_;
  std::vector<geometry_msgs::msg::Pose::SharedPtr>
    latest_peer_drone_poses_;
  geometry_msgs::msg::PoseArray::SharedPtr latest_cows_;
  std::vector<std::vector<cv::Point2f>> latest_vector_grid_;
  std::vector<std::vector<CellState>> latest_cell_grid_;
  cv::Mat latest_is_fixed_grid_;
  cv::Mat latest_saddle_escape_grid_;
  bool received_drone_{false};
  bool received_cows_{false};
  bool has_vector_grid_{false};
  bool has_no_objectives_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OccupancyGridVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
