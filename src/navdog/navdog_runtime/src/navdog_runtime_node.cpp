#include "navdog_runtime/navdog_runtime_node.hpp"
#include "navdog_runtime/ros1_config_loader.hpp"

#include <navdog_protocol/mqtt_codec.hpp>
#include <plan_env/grid_map.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <utility>

namespace navdog_runtime
{

navdog::NavdogConfig NavdogRuntimeNode::loadNavdogConfig(ros::NodeHandle& nh)
{ return Ros1ConfigLoader::load(nh).core; }

NavdogRuntimeNode::NavdogRuntimeNode(
    ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(std::move(nh)), private_nh_(std::move(private_nh)),
      application_config_(Ros1ConfigLoader::load(private_nh_)),
      coordinator_(new navdog::NavigationCoordinator(
          application_config_.core, application_config_.task))
{}

NavdogRuntimeNode::~NavdogRuntimeNode()
{ if (mqtt_) mqtt_->stop(); }

bool NavdogRuntimeNode::initialize()
{
  const auto& io = application_config_.runtime_io;
  if (!std::isfinite(io.control_rate_hz) || io.control_rate_hz <= 0.0 ||
      !std::isfinite(io.status_rate_hz) || io.status_rate_hz <= 0.0)
  {
    ROS_ERROR("invalid runtime rates");
    return false;
  }

  // Create a standalone GridMap for corridor/obstacle evaluation.
  // The native SCAN node (scan_planner_dmq_node) owns its own GridMap.
  auto standalone_grid_map = std::make_shared<GridMap>();
  standalone_grid_map->initMap(private_nh_);
  grid_query_ = std::make_shared<navdog_scan_adapter::ScanGridMapQuery>(
      standalone_grid_map);
  corridor_evaluator_.reset(
      new navdog_scan_adapter::ScanRouteCorridorEvaluator3D(
          application_config_.core.route_corridor, grid_query_));
  obstacle_evaluator_.reset(
      new navdog_scan_adapter::ScanObstacleSummaryEvaluator3D(
          navdog_scan_adapter::ScanObstacleSummaryEvaluator3D::Config{},
          grid_query_));

  private_nh_.param("grid_map/body_height", body_height_, 0.3);

  mqtt_.reset(new navdog_protocol::MqttBridge(application_config_.mqtt));
  if (!mqtt_->start())
    ROS_WARN("MQTT unavailable at startup; local navigation remains active");

  odom_subscriber_ = nh_.subscribe(io.odom_topic, 10,
      &NavdogRuntimeNode::odomCallback, this,
      ros::TransportHints().tcpNoDelay());
  route_publisher_ =
      nh_.advertise<nav_msgs::Path>("/navdog/global_route", 1, true);
  native_scan_path_publisher_ =
      nh_.advertise<nav_msgs::Path>("/native_scan/initial_path", 1, true);
  native_scan_reset_publisher_ =
      nh_.advertise<std_msgs::Empty>(
          "/native_scan/reset", 1, false);
  state_publisher_ = nh_.advertise<std_msgs::UInt8>("/navdog/state", 1);
  mode_publisher_ =
      nh_.advertise<std_msgs::UInt8>("/navdog/navigation_mode", 1);
  final_cmd_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>(
      io.final_cmd_topic, 1);
  control_timer_ = nh_.createTimer(ros::Duration(1.0 / io.control_rate_hz),
      &NavdogRuntimeNode::controlCallback, this);

  const auto& nm = application_config_.core.navigation_mode;
  ROS_INFO("NAV_MODE_CONFIG enter_dist=%.3f enter_confirm=%.3f "
           "immediate_dist=%.3f min_avoid_hold=%.3f "
           "exit_confirm=%.3f exit_front=%.3f exit_left=%.3f exit_right=%.3f",
      nm.enter_blocked_distance_m, nm.enter_confirm_sec,
      nm.immediate_enter_distance_m, nm.min_local_avoid_hold_sec,
      nm.exit_clear_confirm_sec, nm.exit_front_clearance_m,
      nm.exit_left_clearance_m, nm.exit_right_clearance_m);

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
  if (!application_config_.runtime_io.odom_twist_in_world_frame)
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
  navdog_task::NavigationEvent event{};
  while (mqtt_ && mqtt_->popEvent(event))
  {
    const auto result = coordinator_->handleEvent(std::move(event));
    if (result == navdog_task::TaskHandleResult::STARTED)
    {
      resetNativeScan("TASK_STARTED");
      last_route_progress_ = navdog::RouteProgress{};
      publishRoute();
      ROS_INFO("navigation task started: sequence=%lu",
          static_cast<unsigned long>(coordinator_->taskSession().sequence));
    }
    else if (result == navdog_task::TaskHandleResult::CANCELLED)
    {
      resetNativeScan("TASK_CANCELLED");
      last_route_progress_ = navdog::RouteProgress{};
      pending_planner_feedback_ = navdog::PlannerFeedback{};
      route_publisher_.publish(nav_msgs::Path{});
      ROS_INFO("navigation task cancelled");
    }
  }
}

void NavdogRuntimeNode::processPlannerAction(
    const navdog::PlannerAction& action, double now_sec)
{
  if (action.type == navdog::PlannerActionType::SET_ROUTE)
    pending_planner_feedback_ = feedbackForAction(action, now_sec);
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

void NavdogRuntimeNode::controlCallback(const ros::TimerEvent&)
{
  const double now_sec = ros::Time::now().toSec();
  processEvents();
  navdog::CoreInput input{};
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    input.robot = robot_;
  }
  if (obstacle_evaluator_)
    input.obstacles = obstacle_evaluator_->evaluate(input.robot, now_sec);
  if (coordinator_->routeManager().hasRoute() && input.robot.valid &&
      last_route_progress_.valid && grid_query_ && grid_query_->ready())
  {
    input.route_corridor_observation = corridor_evaluator_->evaluate(
        coordinator_->routeManager().taskView(), last_route_progress_,
        input.robot, now_sec);
  }
  input.planner = pending_planner_feedback_;
  pending_planner_feedback_ = navdog::PlannerFeedback{};

  const navdog::CoreOutput output = coordinator_->update(input, now_sec);
  logNavigationChanges(output, input);
  if (output.navigation_mode.transitioned)
  {
    if (output.navigation_mode.mode == navdog::NavigationMode::LOCAL_AVOID)
    {
      resetNativeScan("LOCAL_AVOID_ENTER");
      pending_native_scan_path_ = true;
    }
    else if (output.navigation_mode.previous_mode ==
                 navdog::NavigationMode::LOCAL_AVOID)
    {
      resetNativeScan("LOCAL_AVOID_EXIT");
    }
  }
  // Deferred native scan reference path: ensure reset arrives before path
  scheduleNativeScanReferencePath();
  processPlannerAction(output.planner_action, now_sec);
  if (output.route_progress.valid) last_route_progress_ = output.route_progress;
  publishOutput(output, now_sec);
  if (last_status_publish_.isZero() ||
      (ros::Time::now() - last_status_publish_).toSec() >=
          1.0 / application_config_.runtime_io.status_rate_hz)
  {
    publishMqttStatus(output);
    last_status_publish_ = ros::Time::now();
  }
}

void NavdogRuntimeNode::logNavigationChanges(
    const navdog::CoreOutput& output, const navdog::CoreInput& input)
{
  if (!log_state_initialized_ || output.state != last_logged_state_)
  {
    ROS_INFO("NAV_STATE prev=%s next=%s seq=%lu",
        log_state_initialized_ ? navdog::navStateName(last_logged_state_) : "UNKNOWN",
        navdog::navStateName(output.state),
        static_cast<unsigned long>(output.task_sequence));
    last_logged_state_ = output.state;
    log_state_initialized_ = true;
  }
  if (!log_mode_initialized_ || output.navigation_mode.mode != last_logged_mode_)
  {
    ROS_INFO("NAV_MODE prev=%s next=%s seq=%lu reason=%s blocked_forward=%.3f",
        log_mode_initialized_ ? navdog::navigationModeName(last_logged_mode_) : "NONE",
        navdog::navigationModeName(output.navigation_mode.mode),
        static_cast<unsigned long>(output.task_sequence),
        navdog::navigationModeReasonName(output.navigation_mode.reason),
        input.route_corridor_observation.first_blocked_distance_ahead_m);
    last_logged_mode_ = output.navigation_mode.mode;
    log_mode_initialized_ = true;
  }
}

geometry_msgs::Twist NavdogRuntimeNode::toTwist(
    const navdog::VelocityCommand& command)
{
  geometry_msgs::Twist result;
  if (command.valid && std::isfinite(command.vx) &&
      std::isfinite(command.vy) && std::isfinite(command.yaw_rate))
  {
    result.linear.x = command.vx;
    result.linear.y = command.vy;
    result.angular.z = command.yaw_rate;
  }
  return result;
}

void NavdogRuntimeNode::publishOutput(
    const navdog::CoreOutput& output, double now_sec)
{
  geometry_msgs::TwistStamped command;
  command.header.stamp.fromSec(now_sec);
  command.twist = toTwist(output.final_cmd);
  final_cmd_publisher_.publish(command);
  std_msgs::UInt8 state;
  state.data = static_cast<std::uint8_t>(output.state);
  state_publisher_.publish(state);
  std_msgs::UInt8 mode;
  mode.data = static_cast<std::uint8_t>(output.navigation_mode.mode);
  mode_publisher_.publish(mode);
}

void NavdogRuntimeNode::publishRoute()
{
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";
  for (const auto& point : coordinator_->routeManager().route())
  {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(point.has_yaw ? point.yaw : 0.0);
    path.poses.push_back(pose);
  }
  route_publisher_.publish(path);
}

void NavdogRuntimeNode::resetNativeScan(const char* reason)
{
  native_scan_reset_publisher_.publish(std_msgs::Empty{});

  pending_native_scan_path_ = false;
  native_scan_reset_time_ = ros::Time::now();

  ROS_WARN("NATIVE_SCAN_RESET reason=%s",
      reason ? reason : "UNKNOWN");
}

void NavdogRuntimeNode::scheduleNativeScanReferencePath()
{
  if (pending_native_scan_path_ &&
      (ros::Time::now() - native_scan_reset_time_).toSec() >= 0.02)
  {
    publishNativeScanReferencePath(last_route_progress_);
    pending_native_scan_path_ = false;
  }
}

void NavdogRuntimeNode::publishNativeScanReferencePath(
    const navdog::RouteProgress& progress)
{
  if (!coordinator_ || !coordinator_->routeManager().hasRoute())
    return;

  const auto& route = coordinator_->routeManager().route();
  if (route.size() < 2)
    return;

  // Determine the first remaining route index from progress.
  // Never re-send waypoints the robot has already passed.
  std::size_t first_remaining_index = 0;
  if (progress.valid &&
      progress.task_sequence ==
          coordinator_->taskSession().sequence)
  {
    first_remaining_index =
        std::min(
            progress.segment_index + 1,
            route.size() - 1);
  }

  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";

  // First point: current robot position (lowered to ground reference)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = robot_.x;
    pose.pose.position.y = robot_.y;
    pose.pose.position.z = robot_.z - body_height_;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_.yaw);
    path.poses.push_back(pose);
  }

  // Remaining points: only waypoints after the current progress segment.
  // Z coordinate: route_point.z - body_height_ (convert body height to ground reference)
  for (std::size_t i = first_remaining_index; i < route.size(); ++i)
  {
    const auto& point = route[i];
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z - body_height_;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(
        point.has_yaw ? point.yaw : 0.0);
    path.poses.push_back(pose);
  }

  native_scan_path_publisher_.publish(path);
  ROS_INFO("NATIVE_SCAN_PATH_PUBLISHED points=%lu "
           "first_remaining_index=%lu",
      static_cast<unsigned long>(path.poses.size()),
      static_cast<unsigned long>(first_remaining_index));
}

void NavdogRuntimeNode::statusForOutput(const navdog::CoreOutput& output,
    bool protocol_error, int& status, int& error)
{
  status = 0;
  error = protocol_error ? 1 : 0;
  if (output.state == navdog::NavState::FAILED ||
      output.state == navdog::NavState::EMERGENCY_STOP)
  { status = 0; error = 2; return; }
  if (output.state == navdog::NavState::PAUSED) status = 5;
  else if (output.state == navdog::NavState::PLANNING ||
           output.state == navdog::NavState::START_ALIGN ||
           output.state == navdog::NavState::TRACKING ||
           output.state == navdog::NavState::RECOVERY) status = 1;
}

void NavdogRuntimeNode::publishMqttStatus(const navdog::CoreOutput& output)
{
  if (!mqtt_) return;
  int status = 0;
  int error = 0;
  statusForOutput(output, mqtt_->consumeProtocolError() > 0, status, error);
  mqtt_->publishStatus(navdog_protocol::MqttCodec::encodeStatus(status, error));
}

}  // namespace navdog_runtime
