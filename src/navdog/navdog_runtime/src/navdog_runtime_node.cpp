#include "navdog_runtime/navdog_runtime_node.hpp"

#include <json/json.h>
#include <ros/master.h>
#include <tf/transform_datatypes.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <utility>

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
      if (coordinator_.copyActiveTask(active_task)) publishRoute(active_task);
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
  updateControlOwner(output, input.robot);

  // Compute effective command: start from final_cmd, override on conflict
  effective_command_ = output.final_cmd;
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

  // Latch ownership before handing the route to SCAN. The mux emits a zero
  // frame on this transition, so Navdog cannot keep driving while SCAN plans.
  scan_takeover_active_ = true;
  scan_takeover_task_sequence_ = task.sequence;
  setControlOwner(2);
  native_scan_path_publisher_.publish(path);
  ROS_WARN("NATIVE_SCAN_TAKEOVER task_sequence=%lu remaining_points=%lu "
           "start=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f)",
           static_cast<unsigned long>(task.sequence),
           static_cast<unsigned long>(path.poses.size()),
           robot.x, robot.y, robot.z, goal.x, goal.y, robot.z);
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
  scan_takeover_task_sequence_ = 0;
  if (publish_reset)
    native_scan_reset_publisher_.publish(std_msgs::Empty{});
}

void NavdogRuntimeNode::updateControlOwner(
    const navdog::CoreOutput& output,
    const navdog::RobotState& robot)
{
  if (output.state == navdog::NavState::PAUSED)
  {
    setControlOwner(0);
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
    if (!publishNativeScanTakeoverPath(task, robot, last_route_progress_))
    {
      setControlOwner(0);
    }
    return;
  }

  if (scan_takeover_active_)
  {
    // Ownership is latched for the rest of this task. ROUTE_REJOIN and
    // ROUTE_FOLLOW transitions must never return velocity control to Navdog.
    if (!have_task || task.sequence != scan_takeover_task_sequence_)
    {
      setControlOwner(0);
      resetNativeScanTakeover(true);
    }
    else
    {
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
  if (!ros::master::execute("getSystemState", args, result, payload, true) ||
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
