#include "navdog_runtime/navdog_runtime_node.hpp"

#include <json/json.h>
#include <ros/master.h>
#include <tf/transform_datatypes.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace navdog_runtime
{

navdog::NavdogConfig NavdogRuntimeNode::loadNavdogConfig(ros::NodeHandle& nh)
{
  navdog::NavdogConfig config{};

  // Runtime
  nh.param("control_rate_hz", config.runtime.control_rate_hz,
           config.runtime.control_rate_hz);
  nh.param("status_rate_hz", config.runtime.status_rate_hz,
           config.runtime.status_rate_hz);

  // Task
  nh.param("task/default_max_vx", config.task.default_max_vx,
           config.task.default_max_vx);
  nh.param("task/min_max_vx", config.task.min_max_vx,
           config.task.min_max_vx);
  nh.param("task/max_max_vx", config.task.max_max_vx,
           config.task.max_max_vx);

  // Start align
  nh.param("start_align/enter_deg", config.start_align.enter_deg,
           config.start_align.enter_deg);
  nh.param("start_align/exit_deg", config.start_align.exit_deg,
           config.start_align.exit_deg);
  nh.param("start_align/max_hold_sec", config.start_align.max_hold_sec,
           config.start_align.max_hold_sec);
  nh.param("start_align/kp_yaw", config.start_align.kp_yaw,
           config.start_align.kp_yaw);
  nh.param("start_align/max_yaw_rate", config.start_align.max_yaw_rate,
           config.start_align.max_yaw_rate);
  nh.param("start_align/target_min_dist_m",
           config.start_align.target_min_dist_m,
           config.start_align.target_min_dist_m);

  // Route progress
  nh.param("route_progress/min_segment_length_m",
           config.route_progress.min_segment_length_m,
           config.route_progress.min_segment_length_m);
  nh.param("route_progress/max_forward_search_m",
           config.route_progress.max_forward_search_m,
           config.route_progress.max_forward_search_m);
  nh.param("route_progress/on_route_lateral_tolerance_m",
           config.route_progress.on_route_lateral_tolerance_m,
           config.route_progress.on_route_lateral_tolerance_m);

  // Route corridor
  nh.param("route_corridor/lookahead_distance_m",
           config.route_corridor.lookahead_distance_m,
           config.route_corridor.lookahead_distance_m);

  // Route corridor observation
  nh.param("route_corridor_observation/map_timeout_sec",
           config.route_corridor_observation.map_timeout_sec,
           config.route_corridor_observation.map_timeout_sec);
  nh.param("route_corridor_observation/max_progress_lag_m",
           config.route_corridor_observation.max_progress_lag_m,
           config.route_corridor_observation.max_progress_lag_m);

  // Planner
  nh.param("planner/planning_timeout_sec",
           config.planner.planning_timeout_sec,
           config.planner.planning_timeout_sec);

  // Safety
  nh.param("safety/slow_down_front", config.safety.slow_down_front,
           config.safety.slow_down_front);
  nh.param("safety/emergency_stop", config.safety.emergency_stop,
           config.safety.emergency_stop);
  nh.param("safety/odom_timeout_sec", config.safety.odom_timeout_sec,
           config.safety.odom_timeout_sec);
  nh.param("safety/obstacle_timeout_sec",
           config.safety.obstacle_timeout_sec,
           config.safety.obstacle_timeout_sec);
  nh.param("safety/planner_cmd_timeout_sec",
           config.safety.planner_cmd_timeout_sec,
           config.safety.planner_cmd_timeout_sec);
  nh.param("safety/future_tolerance_sec",
           config.safety.future_tolerance_sec,
           config.safety.future_tolerance_sec);

  // Limits
  nh.param("limits/max_vx", config.limits.max_vx, config.limits.max_vx);
  nh.param("limits/max_vy", config.limits.max_vy, config.limits.max_vy);
  nh.param("limits/max_yaw_rate", config.limits.max_yaw_rate,
           config.limits.max_yaw_rate);
  nh.param("limits/max_accel_x", config.limits.max_accel_x,
           config.limits.max_accel_x);
  nh.param("limits/max_accel_y", config.limits.max_accel_y,
           config.limits.max_accel_y);
  nh.param("limits/max_accel_yaw", config.limits.max_accel_yaw,
           config.limits.max_accel_yaw);

  // Navigation mode
  nh.param("navigation_mode/avoid_enter_distance_m",
           config.navigation_mode.avoid_enter_distance_m,
           config.navigation_mode.avoid_enter_distance_m);
  nh.param("navigation_mode/avoid_immediate_distance_m",
           config.navigation_mode.avoid_immediate_distance_m,
           config.navigation_mode.avoid_immediate_distance_m);
  nh.param("navigation_mode/avoid_block_confirm_sec",
           config.navigation_mode.avoid_block_confirm_sec,
           config.navigation_mode.avoid_block_confirm_sec);
  nh.param("navigation_mode/local_avoid_min_hold_sec",
           config.navigation_mode.local_avoid_min_hold_sec,
           config.navigation_mode.local_avoid_min_hold_sec);
  nh.param("navigation_mode/route_clear_confirm_sec",
           config.navigation_mode.route_clear_confirm_sec,
           config.navigation_mode.route_clear_confirm_sec);
  nh.param("navigation_mode/rejoin_lateral_tolerance_m",
           config.navigation_mode.rejoin_lateral_tolerance_m,
           config.navigation_mode.rejoin_lateral_tolerance_m);
  nh.param("navigation_mode/rejoin_heading_tolerance_rad",
           config.navigation_mode.rejoin_heading_tolerance_rad,
           config.navigation_mode.rejoin_heading_tolerance_rad);
  nh.param("navigation_mode/rejoin_confirm_sec",
           config.navigation_mode.rejoin_confirm_sec,
           config.navigation_mode.rejoin_confirm_sec);

  // Route follower
  nh.param("route_follower/lookahead_distance_m",
           config.route_follower.lookahead_distance_m,
           config.route_follower.lookahead_distance_m);
  nh.param("route_follower/kp_x", config.route_follower.kp_x,
           config.route_follower.kp_x);
  nh.param("route_follower/kp_y", config.route_follower.kp_y,
           config.route_follower.kp_y);
  nh.param("route_follower/kp_yaw", config.route_follower.kp_yaw,
           config.route_follower.kp_yaw);
  nh.param("route_follower/heading_turn_only_threshold_rad",
           config.route_follower.heading_turn_only_threshold_rad,
           config.route_follower.heading_turn_only_threshold_rad);
  nh.param("route_follower/max_vx", config.route_follower.max_vx,
           config.route_follower.max_vx);

  // Trajectory follower
  nh.param("trajectory_follower/time_forward_sec",
           config.trajectory_follower.time_forward_sec,
           config.trajectory_follower.time_forward_sec);
  nh.param("trajectory_follower/kp_pos",
           config.trajectory_follower.kp_pos,
           config.trajectory_follower.kp_pos);
  nh.param("trajectory_follower/kp_yaw",
           config.trajectory_follower.kp_yaw,
           config.trajectory_follower.kp_yaw);
  nh.param("trajectory_follower/heading_turn_only_threshold_rad",
           config.trajectory_follower.heading_turn_only_threshold_rad,
           config.trajectory_follower.heading_turn_only_threshold_rad);

  // Rejoin target
  nh.param("rejoin_target/default_forward_distance_m",
           config.rejoin_target.default_forward_distance_m,
           config.rejoin_target.default_forward_distance_m);
  nh.param("rejoin_target/min_forward_distance_m",
           config.rejoin_target.min_forward_distance_m,
           config.rejoin_target.min_forward_distance_m);
  nh.param("rejoin_target/max_forward_distance_m",
           config.rejoin_target.max_forward_distance_m,
           config.rejoin_target.max_forward_distance_m);
  nh.param("rejoin_target/route_yaw_tolerance_rad",
           config.rejoin_target.route_yaw_tolerance_rad,
           config.rejoin_target.route_yaw_tolerance_rad);

  // Goal controller
  nh.param("goal_controller/near_goal_switch_dist",
           config.goal_controller.near_goal_switch_dist,
           config.goal_controller.near_goal_switch_dist);
  nh.param("goal_controller/near_goal_kp_v",
           config.goal_controller.near_goal_kp_v,
           config.goal_controller.near_goal_kp_v);
  nh.param("goal_controller/near_goal_min_v",
           config.goal_controller.near_goal_min_v,
           config.goal_controller.near_goal_min_v);
  nh.param("goal_controller/near_goal_max_v",
           config.goal_controller.near_goal_max_v,
           config.goal_controller.near_goal_max_v);
  nh.param("goal_controller/near_goal_turn_only_deg",
           config.goal_controller.near_goal_turn_only_deg,
           config.goal_controller.near_goal_turn_only_deg);
  nh.param("goal_controller/near_goal_kp_w",
           config.goal_controller.near_goal_kp_w,
           config.goal_controller.near_goal_kp_w);
  nh.param("goal_controller/near_goal_max_w",
           config.goal_controller.near_goal_max_w,
           config.goal_controller.near_goal_max_w);
  nh.param("goal_controller/obstacle_finish_timeout_sec",
           config.goal_controller.obstacle_finish_timeout_sec,
           config.goal_controller.obstacle_finish_timeout_sec);
  nh.param("goal_controller/finish_dist",
           config.goal_controller.finish_dist,
           config.goal_controller.finish_dist);
  nh.param("goal_controller/finish_yaw_tolerance_deg",
           config.goal_controller.finish_yaw_tolerance_deg,
           config.goal_controller.finish_yaw_tolerance_deg);

  // Planner trigger
  nh.param("planner_trigger/replan_retry_interval_sec",
           config.planner_trigger.replan_retry_interval_sec,
           config.planner_trigger.replan_retry_interval_sec);
  nh.param("planner_trigger/trajectory_expiry_margin_sec",
           config.planner_trigger.trajectory_expiry_margin_sec,
           config.planner_trigger.trajectory_expiry_margin_sec);
  nh.param("planner_trigger/min_remaining_duration_sec",
           config.planner_trigger.min_remaining_duration_sec,
           config.planner_trigger.min_remaining_duration_sec);
  nh.param("planner_trigger/trajectory_source_max_age_sec",
           config.planner_trigger.trajectory_source_max_age_sec,
           config.planner_trigger.trajectory_source_max_age_sec);
  nh.param("planner_trigger/trajectory_future_tolerance_sec",
           config.planner_trigger.trajectory_future_tolerance_sec,
           config.planner_trigger.trajectory_future_tolerance_sec);
  nh.param("planner_trigger/target_change_threshold_m",
           config.planner_trigger.target_change_threshold_m,
           config.planner_trigger.target_change_threshold_m);

  return config;
}

NavdogRuntimeNode::NavdogRuntimeNode(
    ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(std::move(nh)), private_nh_(std::move(private_nh)),
      coordinator_(loadNavdogConfig(private_nh_))
{
}

NavdogRuntimeNode::~NavdogRuntimeNode()
{
  if (initialized_) publishZeroFiveTimes();
  if (mqtt_) mqtt_->stop();
}

bool NavdogRuntimeNode::initialize()
{
  const auto& config = coordinator_.config();

  // Validate runtime config
  if (!std::isfinite(config.runtime.control_rate_hz) ||
      config.runtime.control_rate_hz <= 0.0)
  {
    ROS_ERROR("invalid control_rate_hz: %.3f", config.runtime.control_rate_hz);
    return false;
  }
  if (!std::isfinite(config.runtime.status_rate_hz) ||
      config.runtime.status_rate_hz <= 0.0)
  {
    ROS_ERROR("invalid status_rate_hz: %.3f", config.runtime.status_rate_hz);
    return false;
  }

  // Validate safety timeouts
  if (!std::isfinite(config.safety.odom_timeout_sec) ||
      config.safety.odom_timeout_sec <= 0.0)
  {
    ROS_ERROR("invalid safety/odom_timeout_sec: %.3f",
              config.safety.odom_timeout_sec);
    return false;
  }
  if (!std::isfinite(config.safety.obstacle_timeout_sec) ||
      config.safety.obstacle_timeout_sec <= 0.0)
  {
    ROS_ERROR("invalid safety/obstacle_timeout_sec: %.3f",
              config.safety.obstacle_timeout_sec);
    return false;
  }
  if (!std::isfinite(config.safety.planner_cmd_timeout_sec) ||
      config.safety.planner_cmd_timeout_sec <= 0.0)
  {
    ROS_ERROR("invalid safety/planner_cmd_timeout_sec: %.3f",
              config.safety.planner_cmd_timeout_sec);
    return false;
  }

  // Validate planner timeout
  if (!std::isfinite(config.planner.planning_timeout_sec) ||
      config.planner.planning_timeout_sec <= 0.0)
  {
    ROS_ERROR("invalid planner/planning_timeout_sec: %.3f",
              config.planner.planning_timeout_sec);
    return false;
  }

  // Validate limits
  if (!std::isfinite(config.limits.max_vx) || config.limits.max_vx <= 0.0)
  {
    ROS_ERROR("invalid limits/max_vx: %.3f", config.limits.max_vx);
    return false;
  }
  if (!std::isfinite(config.limits.max_vy) || config.limits.max_vy <= 0.0)
  {
    ROS_ERROR("invalid limits/max_vy: %.3f", config.limits.max_vy);
    return false;
  }
  if (!std::isfinite(config.limits.max_yaw_rate) ||
      config.limits.max_yaw_rate <= 0.0)
  {
    ROS_ERROR("invalid limits/max_yaw_rate: %.3f",
              config.limits.max_yaw_rate);
    return false;
  }
  if (!std::isfinite(config.limits.max_accel_x) ||
      config.limits.max_accel_x <= 0.0)
  {
    ROS_ERROR("invalid limits/max_accel_x: %.3f", config.limits.max_accel_x);
    return false;
  }
  if (!std::isfinite(config.limits.max_accel_y) ||
      config.limits.max_accel_y <= 0.0)
  {
    ROS_ERROR("invalid limits/max_accel_y: %.3f", config.limits.max_accel_y);
    return false;
  }
  if (!std::isfinite(config.limits.max_accel_yaw) ||
      config.limits.max_accel_yaw <= 0.0)
  {
    ROS_ERROR("invalid limits/max_accel_yaw: %.3f",
              config.limits.max_accel_yaw);
    return false;
  }

  // Validate navigation_mode distance thresholds
  if (!std::isfinite(config.navigation_mode.avoid_enter_distance_m) ||
      config.navigation_mode.avoid_enter_distance_m <= 0.0)
  {
    ROS_ERROR("invalid navigation_mode/avoid_enter_distance_m: %.3f",
              config.navigation_mode.avoid_enter_distance_m);
    return false;
  }
  if (!std::isfinite(config.navigation_mode.avoid_immediate_distance_m) ||
      config.navigation_mode.avoid_immediate_distance_m <= 0.0)
  {
    ROS_ERROR("invalid navigation_mode/avoid_immediate_distance_m: %.3f",
              config.navigation_mode.avoid_immediate_distance_m);
    return false;
  }

  // Validate navigation_mode time thresholds
  if (!std::isfinite(config.navigation_mode.avoid_block_confirm_sec) ||
      config.navigation_mode.avoid_block_confirm_sec <= 0.0)
  {
    ROS_ERROR("invalid navigation_mode/avoid_block_confirm_sec: %.3f",
              config.navigation_mode.avoid_block_confirm_sec);
    return false;
  }
  if (!std::isfinite(config.navigation_mode.route_clear_confirm_sec) ||
      config.navigation_mode.route_clear_confirm_sec <= 0.0)
  {
    ROS_ERROR("invalid navigation_mode/route_clear_confirm_sec: %.3f",
              config.navigation_mode.route_clear_confirm_sec);
    return false;
  }
  if (!std::isfinite(config.navigation_mode.rejoin_confirm_sec) ||
      config.navigation_mode.rejoin_confirm_sec <= 0.0)
  {
    ROS_ERROR("invalid navigation_mode/rejoin_confirm_sec: %.3f",
              config.navigation_mode.rejoin_confirm_sec);
    return false;
  }

  control_rate_hz_ = config.runtime.control_rate_hz;
  status_rate_hz_ = config.runtime.status_rate_hz;

  private_nh_.param<std::string>("odom_topic", odom_topic_, "/quad_0/body_pose");
  private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_,
                                 "/navdog/route_cmd_vel");
  private_nh_.param("native_scan_takeover", native_scan_takeover_, true);
  private_nh_.param("odom_twist_in_world_frame", odom_twist_in_world_frame_, true);
  private_nh_.param("native_scan_rejoin/min_forward_distance_m",
                    native_scan_rejoin_min_forward_distance_m_, 0.8);
  private_nh_.param("native_scan_rejoin/preferred_forward_distance_m",
                    native_scan_rejoin_preferred_forward_distance_m_, 1.2);
  private_nh_.param("native_scan_rejoin/max_forward_distance_m",
                    native_scan_rejoin_max_forward_distance_m_, 2.0);
  private_nh_.param("native_scan_rejoin/no_progress_timeout_sec",
                    native_scan_rejoin_no_progress_timeout_sec_, 3.0);
  private_nh_.param("native_scan_rejoin/min_progress_improvement_m",
                    native_scan_rejoin_min_progress_improvement_m_, 0.10);
  private_nh_.param("native_scan_rejoin/max_republish_count",
                    native_scan_rejoin_max_republish_count_, 2);
  private_nh_.param("native_scan_rejoin/release_front_clearance_m",
                    native_scan_release_front_clearance_m_, 2.0);
  private_nh_.param("native_scan_rejoin/release_side_clearance_m",
                    native_scan_release_side_clearance_m_, 0.5);
  if (!std::isfinite(native_scan_rejoin_min_forward_distance_m_) ||
      !std::isfinite(native_scan_rejoin_preferred_forward_distance_m_) ||
      !std::isfinite(native_scan_rejoin_max_forward_distance_m_) ||
      !std::isfinite(native_scan_rejoin_no_progress_timeout_sec_) ||
      !std::isfinite(native_scan_rejoin_min_progress_improvement_m_) ||
      !std::isfinite(native_scan_release_front_clearance_m_) ||
      !std::isfinite(native_scan_release_side_clearance_m_) ||
      native_scan_rejoin_min_forward_distance_m_ <= 0.0 ||
      native_scan_rejoin_preferred_forward_distance_m_ <
          native_scan_rejoin_min_forward_distance_m_ ||
      native_scan_rejoin_max_forward_distance_m_ <
          native_scan_rejoin_preferred_forward_distance_m_ ||
      native_scan_rejoin_no_progress_timeout_sec_ <= 0.0 ||
      native_scan_rejoin_min_progress_improvement_m_ <= 0.0 ||
      native_scan_release_front_clearance_m_ <= 0.0 ||
      native_scan_release_side_clearance_m_ <= 0.0 ||
      native_scan_rejoin_max_republish_count_ < 0)
  {
    ROS_ERROR("invalid native_scan_rejoin parameters");
    return false;
  }
  ROS_INFO("NATIVE_SCAN_RELEASE_POLICY front_clearance=%.3f "
           "side_clearance=%.3f require_corridor_clear=1 "
           "require_rejoin_alignment=1",
           native_scan_release_front_clearance_m_,
           native_scan_release_side_clearance_m_);

  planner_manager_ = std::make_shared<scan_planner::SCANPlannerManager>();
  auto planning_visualization =
      std::make_shared<scan_planner::PlanningVisualization>(private_nh_);
  planner_manager_->initPlanModules(private_nh_, planning_visualization);
  if (!planner_manager_->grid_map_)
  {
    ROS_ERROR("SCANPlannerManager did not create GridMap");
    return false;
  }
  grid_query_ = std::make_shared<navdog_scan_adapter::ScanGridMapQuery>(
      planner_manager_->grid_map_);
  occupancy_query_ = std::make_shared<navdog_scan_adapter::OccupancyQueryAdapter>(grid_query_);
  if (!native_scan_takeover_)
  {
    local_planner_adapter_.reset(new navdog_scan_adapter::ScanLocalPlannerAdapter(
        coordinator_.config().planner_trigger, grid_query_, planner_manager_));
  }
  corridor_evaluator_.reset(new navdog_scan_adapter::ScanRouteCorridorEvaluator3D(
      coordinator_.config().route_corridor, grid_query_));
  obstacle_evaluator_.reset(new navdog_scan_adapter::ScanObstacleSummaryEvaluator3D(
      navdog_scan_adapter::ScanObstacleSummaryEvaluator3D::Config{}, grid_query_));
  coordinator_.setLocalPlannerAdapter(local_planner_adapter_.get());
  coordinator_.setOccupancyQuery(occupancy_query_.get());

  MqttBridge::Config mqtt_config{};
  private_nh_.param("mqtt/enabled", mqtt_config.enabled, true);
  private_nh_.param<std::string>("mqtt/host", mqtt_config.host, "127.0.0.1");
  private_nh_.param("mqtt/port", mqtt_config.port, 1883);
  private_nh_.param("mqtt/keepalive_sec", mqtt_config.keepalive_sec, 30);
  private_nh_.param<std::string>("mqtt/client_id", mqtt_config.client_id, "navdog_runtime");
  private_nh_.param("mqtt/qos", mqtt_config.qos, 1);
  private_nh_.param<std::string>("mqtt/task_topic", mqtt_config.task_topic,
                                 mqtt_config.task_topic);
  private_nh_.param<std::string>("mqtt/pause_topic", mqtt_config.pause_topic,
                                 mqtt_config.pause_topic);
  private_nh_.param<std::string>("mqtt/status_topic", mqtt_config.status_topic,
                                 mqtt_config.status_topic);
  int max_queue_size_int = static_cast<int>(mqtt_config.max_queue_size);
  private_nh_.param("mqtt/max_queue_size", max_queue_size_int, max_queue_size_int);
  mqtt_config.max_queue_size = static_cast<std::size_t>(max_queue_size_int);
  private_nh_.param("default_route_z", mqtt_config.default_route_z, 0.3);
  private_nh_.param("default_max_vx", mqtt_config.default_max_vx, 0.4);

  // Validate MQTT config
  if (mqtt_config.qos < 0 || mqtt_config.qos > 2)
  {
    ROS_ERROR("invalid mqtt/qos: %d (must be 0-2)", mqtt_config.qos);
    return false;
  }
  if (mqtt_config.port < 1 || mqtt_config.port > 65535)
  {
    ROS_ERROR("invalid mqtt/port: %d (must be 1-65535)", mqtt_config.port);
    return false;
  }
  if (mqtt_config.keepalive_sec <= 0)
  {
    ROS_ERROR("invalid mqtt/keepalive_sec: %d (must be > 0)",
              mqtt_config.keepalive_sec);
    return false;
  }
  if (mqtt_config.max_queue_size == 0)
  {
    ROS_ERROR("invalid mqtt/max_queue_size: %lu (must be > 0)",
              static_cast<unsigned long>(mqtt_config.max_queue_size));
    return false;
  }
  if (mqtt_config.task_topic.empty())
  {
    ROS_ERROR("mqtt/task_topic must not be empty");
    return false;
  }
  if (mqtt_config.pause_topic.empty())
  {
    ROS_ERROR("mqtt/pause_topic must not be empty");
    return false;
  }
  if (mqtt_config.status_topic.empty())
  {
    ROS_ERROR("mqtt/status_topic must not be empty");
    return false;
  }

  mqtt_.reset(new MqttBridge(mqtt_config));
  if (!mqtt_->start()) ROS_WARN("MQTT unavailable at startup; navigation runtime remains active");

  odom_subscriber_ = nh_.subscribe(odom_topic_, 10,
      &NavdogRuntimeNode::odomCallback, this);
  cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
  route_publisher_ = nh_.advertise<nav_msgs::Path>("/navdog/global_route", 1, true);
  state_publisher_ = nh_.advertise<std_msgs::UInt8>("/navdog/state", 1);
  mode_publisher_ = nh_.advertise<std_msgs::UInt8>("/navdog/navigation_mode", 1);
  final_cmd_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("/navdog/final_cmd", 1);
  control_owner_publisher_ =
      nh_.advertise<std_msgs::UInt8>("/navdog/control_owner", 1, true);
  native_scan_path_publisher_ =
      nh_.advertise<nav_msgs::Path>("/native_scan/initial_path", 1, false);
  native_scan_reset_publisher_ =
      nh_.advertise<std_msgs::Empty>("/native_scan/reset", 1);
  setControlOwner(0);
  control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_),
      &NavdogRuntimeNode::controlCallback, this);

  // Check cmd_vel publisher uniqueness once after registration
  hasUniqueCmdVelPublisher();
  // Periodic check at 1.0 Hz (not in control loop)
  publisher_check_timer_ = nh_.createTimer(ros::Duration(1.0),
      &NavdogRuntimeNode::publisherCheckCallback, this);

  initialized_ = true;
  return true;
}

void NavdogRuntimeNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  navdog::RobotState robot{};
  robot.x = msg->pose.pose.position.x;
  robot.y = msg->pose.pose.position.y;
  robot.z = msg->pose.pose.position.z;
  robot.yaw = tf::getYaw(msg->pose.pose.orientation);
  robot.vx = msg->twist.twist.linear.x;
  robot.vy = msg->twist.twist.linear.y;
  if (!odom_twist_in_world_frame_)
  {
    const double x = robot.vx;
    const double y = robot.vy;
    robot.vx = std::cos(robot.yaw) * x - std::sin(robot.yaw) * y;
    robot.vy = std::sin(robot.yaw) * x + std::cos(robot.yaw) * y;
  }
  robot.yaw_rate = msg->twist.twist.angular.z;
  robot.stamp_sec = msg->header.stamp.toSec();
  robot.valid = std::isfinite(robot.x) && std::isfinite(robot.y) &&
      std::isfinite(robot.z) && std::isfinite(robot.yaw) &&
      std::isfinite(robot.vx) && std::isfinite(robot.vy) &&
      std::isfinite(robot.yaw_rate) && std::isfinite(robot.stamp_sec);
  std::lock_guard<std::mutex> lock(odom_mutex_);
  robot_ = robot;
}

void NavdogRuntimeNode::processEvents()
{
  navdog::NavigationEvent event{};
  while (mqtt_ && mqtt_->popEvent(event))
  {
    const auto result = coordinator_.handleEvent(event);
    if (result == navdog::TaskHandleResult::STARTED)
    {
      resetNativeScanTakeover(true);
      setControlOwner(1);
      last_route_progress_ = navdog::RouteProgress{};
      navdog::NavigationTask active_task{};
      if (coordinator_.copyActiveTask(active_task))
      {
        publishRoute(active_task);
        double estimated_total_length = 0.0;
        std::size_t distinct_point_count = active_task.points.empty() ? 0 : 1;
        std::size_t last_distinct_index = 0;
        for (std::size_t i = 1; i < active_task.points.size(); ++i)
        {
          const double segment_length = std::hypot(
              active_task.points[i].x - active_task.points[i - 1].x,
              active_task.points[i].y - active_task.points[i - 1].y);
          estimated_total_length += segment_length;
          const double distinct_distance = std::hypot(
              active_task.points[i].x - active_task.points[last_distinct_index].x,
              active_task.points[i].y - active_task.points[last_distinct_index].y);
          if (distinct_distance >= 0.01)
          {
            ++distinct_point_count;
            last_distinct_index = i;
          }
        }
        if (!active_task.points.empty())
        {
          const auto& start = active_task.points.front();
          const auto& goal = active_task.points.back();
          ROS_INFO("TASK_ROUTE_DIAG task_sequence=%lu point_count=%lu "
                   "distinct_point_count=%lu estimated_total_length=%.3f "
                   "start=(%.3f,%.3f) goal=(%.3f,%.3f)",
                   static_cast<unsigned long>(active_task.sequence),
                   static_cast<unsigned long>(active_task.points.size()),
                   static_cast<unsigned long>(distinct_point_count),
                   estimated_total_length, start.x, start.y, goal.x, goal.y);
        }
        if (distinct_point_count <= 1 || estimated_total_length < 0.01)
        {
          ROS_WARN("TASK_ROUTE_DEGENERATE task_sequence=%lu point_count=%lu "
                   "distinct_point_count=%lu fallback=POINT_GOAL_FOLLOW",
                   static_cast<unsigned long>(active_task.sequence),
                   static_cast<unsigned long>(active_task.points.size()),
                   static_cast<unsigned long>(distinct_point_count));
        }
      }
      ROS_INFO("task started: task_sequence=%lu",
               static_cast<unsigned long>(active_task.sequence));
    }
    else if (result == navdog::TaskHandleResult::CANCELLED)
    {
      setControlOwner(0);
      resetNativeScanTakeover(true);
      last_route_progress_ = navdog::RouteProgress{};
      pending_planner_feedback_ = navdog::PlannerFeedback{};
      ROS_INFO("task cancelled");
    }
    else if (result == navdog::TaskHandleResult::PAUSED)
    {
      setControlOwner(0);
      resetNativeScanTakeover(true);
      ROS_INFO("navigation paused");
    }
    else if (result == navdog::TaskHandleResult::RESUMED)
    {
      setControlOwner(scan_takeover_active_ ? 2 : 1);
      ROS_INFO("navigation resumed");
    }
    else if (result != navdog::TaskHandleResult::NONE)
      ROS_WARN("navigation event result=%u", static_cast<unsigned>(result));
  }
}

void NavdogRuntimeNode::processPlannerAction(
    const navdog::PlannerAction& action, double now_sec)
{
  if (action.type == navdog::PlannerActionType::SET_ROUTE)
  {
    pending_planner_feedback_ = feedbackForAction(action, now_sec);
  }
  else if (action.type == navdog::PlannerActionType::CANCEL)
    pending_planner_feedback_ = navdog::PlannerFeedback{};
}

navdog::PlannerFeedback NavdogRuntimeNode::feedbackForAction(
    const navdog::PlannerAction& action, double now_sec)
{
  navdog::PlannerFeedback feedback{};
  if (action.type != navdog::PlannerActionType::SET_ROUTE ||
      action.task.sequence == 0 || !std::isfinite(now_sec)) return feedback;
  feedback.state = navdog::PlannerState::READY;
  feedback.trajectory_id = action.task.sequence;
  feedback.stamp_sec = now_sec;
  feedback.valid = true;
  return feedback;
}

void NavdogRuntimeNode::publisherCheckCallback(const ros::TimerEvent&)
{
  // A temporary ROS Master outage must never block the single-threaded
  // control/MQTT callback queue or latch a false cmd_vel conflict.
  if (!ros::master::check())
  {
    ROS_WARN_THROTTLE(
        5.0,
        "ROS Master temporarily unavailable; "
        "skip cmd_vel publisher check");
    return;
  }

  if (!hasUniqueCmdVelPublisher())
  {
    if (!cmd_vel_conflict_latched_)
    {
      cmd_vel_conflict_latched_ = true;
      ROS_ERROR("multiple %s publishers detected; latching fault",
                cmd_vel_topic_.c_str());
    }
  }
}

void NavdogRuntimeNode::controlCallback(const ros::TimerEvent&)
{
  const double now_sec = ros::Time::now().toSec();
  processEvents();
  navdog::CoreInput input{};
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    input.robot = robot_;
  }
  if (obstacle_evaluator_) input.obstacles = obstacle_evaluator_->evaluate(input.robot, now_sec);
  navdog::NavigationTask task{};
  if (coordinator_.copyActiveTask(task) && input.robot.valid &&
      last_route_progress_.valid && grid_query_ && grid_query_->ready())
  {
    input.route_corridor_observation = corridor_evaluator_->evaluate(
        task, last_route_progress_, input.robot, now_sec);
  }
  if (coordinator_.state() != navdog::NavState::PAUSED)
  {
    input.planner = pending_planner_feedback_;
    pending_planner_feedback_ = navdog::PlannerFeedback{};
  }
  navdog::CoreOutput output = coordinator_.update(input, now_sec);
  processPlannerAction(output.planner_action, now_sec);
  if (output.route_progress.valid) last_route_progress_ = output.route_progress;
  updateControlOwner(output, input.robot, input.obstacles);

  // Compute effective command: start from final_cmd, override on conflict
  effective_command_ = output.final_cmd;
  if (output.final_cmd.source == navdog::CommandSource::TRACKING_STOP &&
      output.navigation_mode.mode == navdog::NavigationMode::ROUTE_FOLLOW)
  {
    const char* reason = "INTERPOLATION_FAILED";
    if (task.points.empty()) reason = "EMPTY_TASK";
    else if (!input.robot.valid) reason = "INVALID_ROBOT";
    else if (!output.route_progress.valid) reason = "INVALID_PROGRESS";
    else if (!output.navigation_mode.corridor_available)
      reason = "CORRIDOR_UNAVAILABLE";
    else if (output.navigation_mode.route_blocked)
      reason = "ROUTE_BLOCKED";
    ROS_WARN_THROTTLE(
        1.0,
        "ROUTE_FOLLOW_STOP task_sequence=%lu reason=%s point_count=%lu "
        "total_length=%.3f progress_valid=%d robot_valid=%d arc_length=%.3f "
        "remaining_distance=%.3f",
        static_cast<unsigned long>(output.task_sequence), reason,
        static_cast<unsigned long>(task.points.size()),
        output.route_progress.total_length_m, output.route_progress.valid,
        input.robot.valid, output.route_progress.arc_length_m,
        output.route_progress.remaining_distance_m);
  }
  cmd_vel_conflict_ = false;
  if (cmd_vel_conflict_latched_)
  {
    effective_command_.vx = 0.0;
    effective_command_.vy = 0.0;
    effective_command_.yaw_rate = 0.0;
    effective_command_.valid = true;
    effective_command_.source = navdog::CommandSource::SAFETY_STOP;
    effective_command_.stamp_sec = now_sec;
    cmd_vel_conflict_ = true;
  }

  // Publish effective command to /cmd_vel
  geometry_msgs::Twist command = toTwist(effective_command_);
  cmd_vel_publisher_.publish(command);

  publishDebug(output, now_sec, effective_command_);
  if (last_status_publish_.isZero() ||
      (ros::Time::now() - last_status_publish_).toSec() >= 1.0 / status_rate_hz_)
  {
    publishMqttStatus(output);
    last_status_publish_ = ros::Time::now();
  }
  if (output.state != last_logged_state_ ||
      output.navigation_mode.mode != last_logged_mode_)
  {
    ROS_INFO("task_sequence=%lu state=%u mode=%u source=%u vx=%.3f vy=%.3f w=%.3f",
        static_cast<unsigned long>(output.task_sequence),
        static_cast<unsigned>(output.state),
        static_cast<unsigned>(output.navigation_mode.mode),
        static_cast<unsigned>(effective_command_.source),
        effective_command_.vx, effective_command_.vy,
        effective_command_.yaw_rate);
    last_logged_state_ = output.state;
    last_logged_mode_ = output.navigation_mode.mode;
  }
  last_output_ = output;
}

geometry_msgs::Twist NavdogRuntimeNode::toTwist(const navdog::VelocityCommand& command)
{
  geometry_msgs::Twist result;
  if (command.valid)
  {
    result.linear.x = command.vx;
    result.linear.y = command.vy;
    result.angular.z = command.yaw_rate;
  }
  return result;
}

void NavdogRuntimeNode::publishDebug(const navdog::CoreOutput& output,
                                     double now_sec,
                                     const navdog::VelocityCommand& effective_cmd)
{
  std_msgs::UInt8 state; state.data = static_cast<std::uint8_t>(output.state);
  std_msgs::UInt8 mode; mode.data = static_cast<std::uint8_t>(output.navigation_mode.mode);
  state_publisher_.publish(state);
  mode_publisher_.publish(mode);
  geometry_msgs::TwistStamped command;
  command.header.stamp.fromSec(now_sec);
  command.twist = toTwist(effective_cmd);
  final_cmd_publisher_.publish(command);
}

void NavdogRuntimeNode::publishRoute(const navdog::NavigationTask& task)
{
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";
  for (const auto& point : task.points)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(point.has_yaw ? point.yaw : 0.0);
    path.poses.push_back(pose);
  }
  route_publisher_.publish(path);
}

bool NavdogRuntimeNode::publishNativeScanTakeoverPath(
    const navdog::NavigationTask& task,
    const navdog::RobotState& robot,
    const navdog::RouteProgress& progress)
{
  if (!robot.valid || !progress.valid || task.points.empty() ||
      progress.task_sequence != task.sequence ||
      progress.segment_index >= task.points.size())
  {
    ROS_ERROR("cannot build native SCAN takeover path for task_sequence=%lu",
              static_cast<unsigned long>(task.sequence));
    return false;
  }

  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";

  const auto append_point = [&path, &robot](double x, double y)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = robot.z;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  };

  append_point(robot.x, robot.y);
  const std::size_t next_index = std::min(
      progress.segment_index + 1, task.points.size() - 1);
  for (std::size_t i = next_index; i < task.points.size(); ++i)
    append_point(task.points[i].x, task.points[i].y);

  const auto& goal = task.points.back();
  if (path.poses.back().pose.position.x != goal.x ||
      path.poses.back().pose.position.y != goal.y)
    append_point(goal.x, goal.y);

  native_scan_path_publisher_.publish(path);
  ROS_WARN("NATIVE_SCAN_PATH_PUBLISHED task_sequence=%lu point_count=%lu owner=SCAN",
           static_cast<unsigned long>(task.sequence),
           static_cast<unsigned long>(path.poses.size()));
  return true;
}

bool NavdogRuntimeNode::publishNativeScanRejoinPath(
    const navdog::NavigationTask& task,
    const navdog::RobotState& robot,
    const navdog::RouteProgress& progress)
{
  if (!robot.valid || !progress.valid || task.points.size() < 2 ||
      progress.task_sequence != task.sequence || !occupancy_query_ ||
      !occupancy_query_->ready())
  {
    ROS_ERROR("cannot build native SCAN rejoin path for task_sequence=%lu",
              static_cast<unsigned long>(task.sequence));
    return false;
  }

  std::vector<double> cumulative(task.points.size(), 0.0);
  for (std::size_t i = 1; i < task.points.size(); ++i)
  {
    cumulative[i] = cumulative[i - 1] + std::hypot(
        task.points[i].x - task.points[i - 1].x,
        task.points[i].y - task.points[i - 1].y);
  }
  const double total_length = cumulative.back();
  const double min_arc = std::min(
      total_length,
      progress.arc_length_m + native_scan_rejoin_min_forward_distance_m_);
  const double max_arc = std::min(
      total_length,
      progress.arc_length_m + native_scan_rejoin_max_forward_distance_m_);
  const double preferred_arc = std::max(
      min_arc,
      std::min(max_arc,
               progress.arc_length_m +
                   native_scan_rejoin_preferred_forward_distance_m_));
  if (max_arc <= progress.arc_length_m + 1e-6)
  {
    ROS_ERROR("native SCAN rejoin has no forward route for task_sequence=%lu",
              static_cast<unsigned long>(task.sequence));
    return false;
  }

  const auto interpolate = [&](double arc, double& x, double& y,
                               double& yaw, std::size_t& segment_end) -> bool
  {
    for (std::size_t i = 1; i < task.points.size(); ++i)
    {
      const double length = cumulative[i] - cumulative[i - 1];
      if (length <= 1e-9 || cumulative[i] + 1e-9 < arc)
        continue;
      const double ratio = std::max(
          0.0, std::min(1.0, (arc - cumulative[i - 1]) / length));
      x = task.points[i - 1].x +
          ratio * (task.points[i].x - task.points[i - 1].x);
      y = task.points[i - 1].y +
          ratio * (task.points[i].y - task.points[i - 1].y);
      yaw = std::atan2(task.points[i].y - task.points[i - 1].y,
                       task.points[i].x - task.points[i - 1].x);
      segment_end = i;
      return true;
    }
    return false;
  };

  double target_x = 0.0;
  double target_y = 0.0;
  double target_yaw = 0.0;
  double target_arc = preferred_arc;
  std::size_t target_segment_end = 0;
  bool found_target = false;
  const double search_step = 0.10;
  const int search_count = static_cast<int>(
      std::ceil((max_arc - min_arc) / search_step));
  for (int i = 0; i <= search_count && !found_target; ++i)
  {
    const double offset = static_cast<double>(i) * search_step;
    const double candidates[2] = {preferred_arc + offset,
                                  preferred_arc - offset};
    for (const double candidate_arc : candidates)
    {
      if (candidate_arc < min_arc - 1e-9 ||
          candidate_arc > max_arc + 1e-9)
        continue;
      std::size_t segment_end = 0;
      double x = 0.0;
      double y = 0.0;
      double yaw = 0.0;
      if (interpolate(candidate_arc, x, y, yaw, segment_end) &&
          occupancy_query_->isFree(x, y, robot.z, yaw))
      {
        target_x = x;
        target_y = y;
        target_yaw = yaw;
        target_arc = candidate_arc;
        target_segment_end = segment_end;
        found_target = true;
        break;
      }
    }
  }
  if (!found_target)
  {
    ROS_ERROR("no free forward rejoin point for task_sequence=%lu arc=[%.3f,%.3f]",
              static_cast<unsigned long>(task.sequence), min_arc, max_arc);
    return false;
  }

  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";
  const auto append = [&](double x, double y, double yaw)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = robot.z;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    path.poses.push_back(pose);
  };
  append(robot.x, robot.y, robot.yaw);
  append(target_x, target_y, target_yaw);
  for (std::size_t i = target_segment_end; i < task.points.size(); ++i)
  {
    if (cumulative[i] <= target_arc + 0.01)
      continue;
    const double yaw = i > 0
        ? std::atan2(task.points[i].y - task.points[i - 1].y,
                     task.points[i].x - task.points[i - 1].x)
        : target_yaw;
    append(task.points[i].x, task.points[i].y, yaw);
  }
  native_scan_path_publisher_.publish(path);
  ROS_WARN("NATIVE_SCAN_ACTIVE_REJOIN task_sequence=%lu robot=(%.3f,%.3f) "
           "progress_arc=%.3f lateral_error=%.3f target=(%.3f,%.3f) "
           "remaining_points=%lu",
           static_cast<unsigned long>(task.sequence), robot.x, robot.y,
           progress.arc_length_m, progress.lateral_error_m,
           target_x, target_y,
           static_cast<unsigned long>(path.poses.size() - 1));
  return true;
}

void NavdogRuntimeNode::setControlOwner(std::uint8_t owner)
{
  if (owner > 2) owner = 0;
  if (control_owner_ != owner)
  {
    static const char* names[] = {"STOP", "ROUTE", "SCAN"};
    ROS_WARN("CONTROL_OWNER %s -> %s", names[control_owner_], names[owner]);
    control_owner_ = owner;
  }
  std_msgs::UInt8 message;
  message.data = control_owner_;
  control_owner_publisher_.publish(message);
}

void NavdogRuntimeNode::resetNativeScanTakeover(bool publish_reset)
{
  scan_takeover_active_ = false;
  scan_rejoin_complete_pending_ = false;
  scan_takeover_task_sequence_ = 0;
  resetRejoinMonitoring();
  if (publish_reset)
    native_scan_reset_publisher_.publish(std_msgs::Empty{});
}

void NavdogRuntimeNode::resetRejoinMonitoring() noexcept
{
  best_rejoin_lateral_error_ = 0.0;
  rejoin_enter_stamp_sec_ = 0.0;
  last_rejoin_republish_stamp_sec_ = 0.0;
  rejoin_republish_count_ = 0;
}

void NavdogRuntimeNode::releaseNativeScanToRoute(
    const navdog::NavigationTask& task,
    const navdog::RobotState& robot,
    const navdog::CoreOutput& output,
    const navdog::ObstacleSummary& obstacles)
{
  const navdog::RouteProgress& progress = output.route_progress;
  const double heading_error = progress.valid && robot.valid
      ? std::atan2(std::sin(robot.yaw - progress.route_yaw),
                   std::cos(robot.yaw - progress.route_yaw))
      : 0.0;
  ROS_WARN("NATIVE_SCAN_RELEASE_TO_ROUTE task_sequence=%lu "
           "lateral_error=%.3f route_yaw=%.3f robot_yaw=%.3f "
           "heading_error=%.3f front_clearance=%.3f left_clearance=%.3f "
           "right_clearance=%.3f reason=REJOIN_COMPLETE_AND_SURROUNDING_CLEAR",
           static_cast<unsigned long>(task.sequence),
           progress.lateral_error_m, progress.route_yaw, robot.yaw,
           heading_error, obstacles.front_min, obstacles.left_min,
           obstacles.right_min);
  resetNativeScanTakeover(true);
  setControlOwner(1);
}

void NavdogRuntimeNode::updateControlOwner(
    const navdog::CoreOutput& output,
    const navdog::RobotState& robot,
    const navdog::ObstacleSummary& obstacles)
{
  if (output.state == navdog::NavState::PAUSED)
  {
    setControlOwner(0);
    if (scan_takeover_active_ || scan_takeover_task_sequence_ != 0)
      resetNativeScanTakeover(true);
    return;
  }

  if (output.state == navdog::NavState::IDLE ||
      output.state == navdog::NavState::SUCCEEDED ||
      output.state == navdog::NavState::FAILED ||
      output.state == navdog::NavState::EMERGENCY_STOP)
  {
    setControlOwner(0);
    if (scan_takeover_active_ || scan_takeover_task_sequence_ != 0)
      resetNativeScanTakeover(true);
    return;
  }

  navdog::NavigationTask task{};
  const bool have_task = coordinator_.copyActiveTask(task);
  if (native_scan_takeover_ && !scan_takeover_active_ && have_task &&
      output.navigation_mode.mode == navdog::NavigationMode::LOCAL_AVOID)
  {
    scan_takeover_active_ = true;
    scan_rejoin_complete_pending_ = false;
    scan_takeover_task_sequence_ = task.sequence;

    // Switch ownership first.  The mux clears the previous owner's cached
    // velocity so zero velocity is held until SCAN produces its first cmd.
    // Do NOT publish /native_scan/reset here — SCAN is already clean from
    // the task-boundary reset, and publishing reset concurrently with the
    // path below creates a race where reset can arrive after the path and
    // clear have_target_, leaving the robot permanently stopped.
    setControlOwner(2);

    if (!publishNativeScanTakeoverPath(task, robot, last_route_progress_))
    {
      ROS_ERROR("NATIVE_SCAN_TAKEOVER failed task_sequence=%lu",
                static_cast<unsigned long>(task.sequence));

      resetNativeScanTakeover(false);
      setControlOwner(0);
      return;
    }

    ROS_WARN("ROUTE_CORRIDOR_BLOCKED task_sequence=%lu distance_ahead=%.3f "
             "route_arc=%.3f mode=LOCAL_AVOID",
             static_cast<unsigned long>(task.sequence),
             output.route_corridor.first_blocked_distance_ahead_m,
             output.route_corridor.first_blocked_arc_length_m);

    ROS_WARN("NATIVE_SCAN_TAKEOVER task_sequence=%lu blocked_distance=%.3f",
             static_cast<unsigned long>(task.sequence),
             output.route_corridor.first_blocked_distance_ahead_m);

    return;
  }

  if (scan_takeover_active_)
  {
    if (!have_task || task.sequence != scan_takeover_task_sequence_)
    {
      setControlOwner(0);
      resetNativeScanTakeover(true);
    }
    else if (output.navigation_mode.mode == navdog::NavigationMode::LOCAL_AVOID)
    {
      scan_rejoin_complete_pending_ = false;
      if (output.navigation_mode.transitioned &&
          output.navigation_mode.previous_mode ==
              navdog::NavigationMode::ROUTE_REJOIN)
        resetRejoinMonitoring();
      setControlOwner(2);
    }
    else if (output.navigation_mode.mode == navdog::NavigationMode::ROUTE_REJOIN)
    {
      scan_rejoin_complete_pending_ = false;
      const double now_sec = ros::Time::now().toSec();
      if (output.navigation_mode.transitioned &&
          output.navigation_mode.previous_mode ==
              navdog::NavigationMode::LOCAL_AVOID)
      {
        // Keep SCAN ownership while replacing the completed avoidance
        // trajectory with a forward path that actively rejoins the route.
        setControlOwner(2);
        best_rejoin_lateral_error_ = output.route_progress.lateral_error_m;
        rejoin_enter_stamp_sec_ = now_sec;
        last_rejoin_republish_stamp_sec_ = now_sec;
        rejoin_republish_count_ = 0;
        // Do NOT publish reset before the rejoin path — SCAN's pathCallback()
        // can already transition to REPLAN_TRAJ from EXEC_TRAJ without a
        // reset, and publishing both topics in the same callback races.
        publishNativeScanRejoinPath(task, robot, output.route_progress);
        const double heading_error = output.route_progress.valid && robot.valid
            ? std::atan2(std::sin(robot.yaw - output.route_progress.route_yaw),
                         std::cos(robot.yaw - output.route_progress.route_yaw))
            : 0.0;
        ROS_WARN("ROUTE_CORRIDOR_CLEAR_CONFIRMED task_sequence=%lu clear_elapsed=%.3f",
                 static_cast<unsigned long>(task.sequence),
                 coordinator_.config().navigation_mode.route_clear_confirm_sec);
        ROS_WARN("NATIVE_SCAN_REJOIN task_sequence=%lu lateral_error=%.3f "
                 "heading_error=%.3f",
                 static_cast<unsigned long>(task.sequence),
                 output.route_progress.lateral_error_m, heading_error);
      }
      else if (output.route_progress.valid &&
               std::isfinite(output.route_progress.lateral_error_m))
      {
        const double lateral_error = output.route_progress.lateral_error_m;
        if (lateral_error <= best_rejoin_lateral_error_ -
                                 native_scan_rejoin_min_progress_improvement_m_)
        {
          best_rejoin_lateral_error_ = lateral_error;
          rejoin_enter_stamp_sec_ = now_sec;
        }
        const double stalled_elapsed = now_sec - std::max(
            rejoin_enter_stamp_sec_, last_rejoin_republish_stamp_sec_);
        if (rejoin_enter_stamp_sec_ > 0.0 &&
            stalled_elapsed >= native_scan_rejoin_no_progress_timeout_sec_)
        {
          ROS_WARN("NATIVE_SCAN_REJOIN_STALLED task_sequence=%lu "
                   "lateral_error=%.3f best_lateral_error=%.3f elapsed=%.3f "
                   "republish_count=%d",
                   static_cast<unsigned long>(task.sequence), lateral_error,
                   best_rejoin_lateral_error_, stalled_elapsed,
                   rejoin_republish_count_);
          if (rejoin_republish_count_ <
              native_scan_rejoin_max_republish_count_)
          {
            // Do NOT publish reset before the rejoin path — same race as the
            // initial rejoin handoff: reset arriving after the path would
            // clear have_target_ and leave SCAN without a trajectory.
            publishNativeScanRejoinPath(task, robot, output.route_progress);
            ++rejoin_republish_count_;
          }
          else
          {
            ROS_ERROR("NATIVE_SCAN_REJOIN_STALLED maximum republish count "
                      "reached; retaining SCAN ownership");
          }
          best_rejoin_lateral_error_ = lateral_error;
          rejoin_enter_stamp_sec_ = now_sec;
          last_rejoin_republish_stamp_sec_ = now_sec;
        }
      }
      setControlOwner(2);
    }
    else if (output.navigation_mode.valid &&
             output.navigation_mode.mode == navdog::NavigationMode::ROUTE_FOLLOW)
    {
      if (output.navigation_mode.reason ==
          navdog::NavigationModeReason::REJOIN_COMPLETE)
      {
        scan_rejoin_complete_pending_ = true;
      }

      const auto clearanceSatisfied = [](double distance,
                                         double required) -> bool
      {
        return !std::isnan(distance) && distance >= required;
      };
      const bool corridor_clear =
          output.navigation_mode.corridor_available &&
          !output.navigation_mode.route_blocked;
      const double heading_error = output.route_progress.valid && robot.valid
          ? std::atan2(
                std::sin(robot.yaw - output.route_progress.route_yaw),
                std::cos(robot.yaw - output.route_progress.route_yaw))
          : std::numeric_limits<double>::infinity();
      const bool alignment_ok = output.route_progress.valid && robot.valid &&
          std::isfinite(output.route_progress.lateral_error_m) &&
          std::fabs(output.route_progress.lateral_error_m) <=
              coordinator_.config().navigation_mode
                  .rejoin_lateral_tolerance_m &&
          std::isfinite(heading_error) &&
          std::fabs(heading_error) <=
              coordinator_.config().navigation_mode
                  .rejoin_heading_tolerance_rad;
      const bool surroundings_clear = obstacles.valid &&
          clearanceSatisfied(obstacles.front_min,
                             native_scan_release_front_clearance_m_) &&
          clearanceSatisfied(obstacles.left_min,
                             native_scan_release_side_clearance_m_) &&
          clearanceSatisfied(obstacles.right_min,
                             native_scan_release_side_clearance_m_);

      if (scan_rejoin_complete_pending_ && corridor_clear && alignment_ok &&
          surroundings_clear)
      {
        releaseNativeScanToRoute(task, robot, output, obstacles);
      }
      else
      {
        ROS_WARN_THROTTLE(
            1.0,
            "NATIVE_SCAN_RELEASE_WAIT task_sequence=%lu "
            "rejoin_complete=%d corridor_clear=%d alignment_ok=%d "
            "lateral_error=%.3f heading_error=%.3f obstacle_valid=%d "
            "front=%.3f/%.3f left=%.3f/%.3f right=%.3f/%.3f",
            static_cast<unsigned long>(task.sequence),
            scan_rejoin_complete_pending_ ? 1 : 0,
            corridor_clear ? 1 : 0, alignment_ok ? 1 : 0,
            output.route_progress.lateral_error_m, heading_error,
            obstacles.valid ? 1 : 0,
            obstacles.front_min, native_scan_release_front_clearance_m_,
            obstacles.left_min, native_scan_release_side_clearance_m_,
            obstacles.right_min, native_scan_release_side_clearance_m_);
        setControlOwner(2);
      }
    }
    else
    {
      // A nonterminal, non-release state must not silently hand control back
      // to RouteFollower before NavigationModeManager confirms the rejoin.
      setControlOwner(2);
    }
    return;
  }

  setControlOwner(have_task ? 1 : 0);
}

void NavdogRuntimeNode::statusForOutput(
    const navdog::CoreOutput& output, bool protocol_error, int& status, int& error)
{
  status = 0; error = protocol_error ? 1 : 0;

  // FAILED / EMERGENCY_STOP -> status=0, error=2
  if (output.state == navdog::NavState::FAILED ||
      output.state == navdog::NavState::EMERGENCY_STOP)
  {
    status = 0; error = 2;
    return;
  }

  if (output.state == navdog::NavState::PAUSED) status = 5;
  else if (output.state == navdog::NavState::PLANNING ||
           output.state == navdog::NavState::START_ALIGN ||
           output.state == navdog::NavState::TRACKING ||
           output.state == navdog::NavState::GOAL_ALIGN ||
           output.state == navdog::NavState::RECOVERY) status = 1;
}

void NavdogRuntimeNode::publishMqttStatus(const navdog::CoreOutput& output)
{
  if (!mqtt_) return;
  int status = 0, error = 0;
  statusForOutput(output, mqtt_->consumeProtocolError() > 0, status, error);
  Json::Value root;
  root["status"] = status; root["error"] = error;
  root["velocity"]["vx"] = effective_command_.vx;
  root["velocity"]["vy"] = effective_command_.vy;
  root["velocity"]["yaw_rate"] = effective_command_.yaw_rate;
  Json::StreamWriterBuilder writer; writer["indentation"] = "";
  mqtt_->publishStatus(Json::writeString(writer, root));
}

bool NavdogRuntimeNode::hasUniqueCmdVelPublisher()
{
  const std::string resolved = nh_.resolveName(cmd_vel_topic_);
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();
  if (!ros::master::execute("getSystemState", args, result, payload, false) ||
      payload.getType() != XmlRpc::XmlRpcValue::TypeArray || payload.size() < 1)
    return false;
  const auto& publishers = payload[0];
  for (int i = 0; i < publishers.size(); ++i)
  {
    if (static_cast<std::string>(publishers[i][0]) == resolved)
      return publishers[i][1].size() == 1 &&
          static_cast<std::string>(publishers[i][1][0]) == ros::this_node::getName();
  }
  return false;
}

bool NavdogRuntimeNode::hasUniqueCmdVelPublisherCached() const
{
  return !cmd_vel_conflict_latched_;
}

void NavdogRuntimeNode::publishZeroFiveTimes()
{
  geometry_msgs::Twist zero;
  for (int i = 0; i < 5; ++i)
  {
    cmd_vel_publisher_.publish(zero);
    ros::WallDuration(0.02).sleep();
  }
}

}  // namespace navdog_runtime
