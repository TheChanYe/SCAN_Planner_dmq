#include "navdog_runtime/navdog_runtime_node.hpp"
#include "navdog_runtime/ros1_config_loader.hpp"

#include <navdog_protocol/mqtt_codec.hpp>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <utility>

namespace navdog_runtime
{

namespace
{
const char* navigationModeName(navdog::NavigationMode mode) noexcept
{
  switch (mode)
  {
    case navdog::NavigationMode::ROUTE_FOLLOW:
      return "ROUTE_FOLLOW";
    case navdog::NavigationMode::LOCAL_AVOID:
      return "LOCAL_AVOID";
    default:
      return "NONE";
  }
}
}  // namespace

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

  planner_manager_ = std::make_shared<scan_planner::SCANPlannerManager>();
  auto visualization =
      std::make_shared<scan_planner::PlanningVisualization>(private_nh_);
  planner_manager_->initPlanModules(private_nh_, visualization);
  if (!planner_manager_->grid_map_)
  {
    ROS_ERROR("SCANPlannerManager did not create GridMap");
    return false;
  }
  grid_query_ = std::make_shared<navdog_scan_adapter::ScanGridMapQuery>(
      planner_manager_->grid_map_);
  occupancy_query_ =
      std::make_shared<navdog_scan_adapter::OccupancyQueryAdapter>(grid_query_);
  // The adapter is unconditional: Core alone decides when a local plan is needed.
  local_planner_adapter_.reset(new navdog_scan_adapter::ScanLocalPlannerAdapter(
      application_config_.scan_adapter.planner_trigger,
      grid_query_, planner_manager_));
  corridor_evaluator_.reset(
      new navdog_scan_adapter::ScanRouteCorridorEvaluator3D(
          application_config_.core.route_corridor, grid_query_));
  obstacle_evaluator_.reset(
      new navdog_scan_adapter::ScanObstacleSummaryEvaluator3D(
          navdog_scan_adapter::ScanObstacleSummaryEvaluator3D::Config{},
          grid_query_));
  coordinator_->setLocalPlannerAdapter(local_planner_adapter_.get());
  coordinator_->setOccupancyQuery(occupancy_query_.get());

  mqtt_.reset(new navdog_protocol::MqttBridge(application_config_.mqtt));
  if (!mqtt_->start())
    ROS_WARN("MQTT unavailable at startup; local navigation remains active");

  odom_subscriber_ = nh_.subscribe(io.odom_topic, 10,
      &NavdogRuntimeNode::odomCallback, this,
      ros::TransportHints().tcpNoDelay());
  route_publisher_ =
      nh_.advertise<nav_msgs::Path>("/navdog/global_route", 1, true);
  local_trajectory_publisher_ =
      nh_.advertise<nav_msgs::Path>("/navdog/local_trajectory", 1, true);
  state_publisher_ = nh_.advertise<std_msgs::UInt8>("/navdog/state", 1);
  mode_publisher_ =
      nh_.advertise<std_msgs::UInt8>("/navdog/navigation_mode", 1);
  final_cmd_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>(
      io.final_cmd_topic, 1);
  control_timer_ = nh_.createTimer(ros::Duration(1.0 / io.control_rate_hz),
      &NavdogRuntimeNode::controlCallback, this);
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
      last_route_progress_ = navdog::RouteProgress{};
      publishRoute();
      ROS_INFO("navigation task started: sequence=%lu",
          static_cast<unsigned long>(coordinator_->taskSession().sequence));
    }
    else if (result == navdog_task::TaskHandleResult::CANCELLED)
    {
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
  if (output.navigation_mode.transitioned)
  {
    if (output.navigation_mode.mode == navdog::NavigationMode::LOCAL_AVOID)
    {
      ROS_INFO("NAV_MODE %s -> %s blocked_forward=%.3f blocked_lateral=0.600",
          navigationModeName(output.navigation_mode.previous_mode),
          navigationModeName(output.navigation_mode.mode),
          input.route_corridor_observation.first_blocked_distance_ahead_m);
    }
    else if (output.navigation_mode.previous_mode ==
                 navdog::NavigationMode::LOCAL_AVOID)
    {
      ROS_INFO("NAV_MODE %s -> %s front=2.500 left=0.600 right=0.600 clear=true",
          navigationModeName(output.navigation_mode.previous_mode),
          navigationModeName(output.navigation_mode.mode));
    }
  }
  processPlannerAction(output.planner_action, now_sec);
  if (output.route_progress.valid) last_route_progress_ = output.route_progress;
  publishOutput(output, now_sec);
  publishLocalTrajectory(output);
  if (last_status_publish_.isZero() ||
      (ros::Time::now() - last_status_publish_).toSec() >=
          1.0 / application_config_.runtime_io.status_rate_hz)
  {
    publishMqttStatus(output);
    last_status_publish_ = ros::Time::now();
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

void NavdogRuntimeNode::publishLocalTrajectory(
    const navdog::CoreOutput& output)
{
  const navdog::LocalTrajectory& trajectory =
      coordinator_->activeLocalTrajectory();
  if (output.navigation_mode.mode != navdog::NavigationMode::LOCAL_AVOID ||
      !trajectory.valid)
  {
    if (local_trajectory_visible_)
    {
      nav_msgs::Path empty;
      empty.header.stamp = ros::Time::now();
      empty.header.frame_id = "world";
      local_trajectory_publisher_.publish(empty);
      local_trajectory_visible_ = false;
      published_local_plan_sequence_ = 0;
    }
    return;
  }

  if (trajectory.plan_sequence == published_local_plan_sequence_)
    return;

  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";
  for (const auto& point : trajectory.points)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(
        point.has_yaw ? point.yaw : 0.0);
    path.poses.push_back(pose);
  }
  local_trajectory_publisher_.publish(path);
  ROS_INFO("LOCAL_PLAN_PROMOTED task=%lu plan=%lu points=%lu duration=%.3f",
      static_cast<unsigned long>(trajectory.task_sequence),
      static_cast<unsigned long>(trajectory.plan_sequence),
      static_cast<unsigned long>(trajectory.points.size()),
      trajectory.duration_sec);
  published_local_plan_sequence_ = trajectory.plan_sequence;
  local_trajectory_visible_ = true;
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
