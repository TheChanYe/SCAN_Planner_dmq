#include "navdog_runtime/navdog_runtime_node.hpp"

#include <json/json.h>
#include <ros/master.h>
#include <tf/transform_datatypes.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <cmath>
#include <utility>

namespace navdog_runtime
{

navdog::NavdogConfig NavdogRuntimeNode::loadNavdogConfig(ros::NodeHandle& nh)
{
  navdog::NavdogConfig config{};
  nh.param("limits/max_vx", config.limits.max_vx, config.limits.max_vx);
  nh.param("limits/max_vy", config.limits.max_vy, config.limits.max_vy);
  nh.param("limits/max_yaw_rate", config.limits.max_yaw_rate,
           config.limits.max_yaw_rate);
  nh.param("safety/odom_timeout_sec", config.safety.odom_timeout_sec,
           config.safety.odom_timeout_sec);
  nh.param("safety/obstacle_timeout_sec", config.safety.obstacle_timeout_sec,
           config.safety.obstacle_timeout_sec);
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
  private_nh_.param("control_rate_hz", control_rate_hz_, 50.0);
  private_nh_.param("status_rate_hz", status_rate_hz_, 10.0);
  private_nh_.param<std::string>("odom_topic", odom_topic_, "/quad_0/body_pose");
  private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
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
  local_planner_adapter_.reset(new navdog_scan_adapter::ScanLocalPlannerAdapter(
      coordinator_.config().planner_trigger, grid_query_, planner_manager_));
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
  private_nh_.param("default_route_z", mqtt_config.default_route_z, 0.3);
  private_nh_.param("default_max_vx", mqtt_config.default_max_vx, 0.4);
  mqtt_.reset(new MqttBridge(mqtt_config));
  if (!mqtt_->start()) ROS_WARN("MQTT unavailable at startup; navigation runtime remains active");

  odom_subscriber_ = nh_.subscribe(odom_topic_, 10,
      &NavdogRuntimeNode::odomCallback, this);
  cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
  route_publisher_ = nh_.advertise<nav_msgs::Path>("/navdog/global_route", 1, true);
  state_publisher_ = nh_.advertise<std_msgs::UInt8>("/navdog/state", 1);
  mode_publisher_ = nh_.advertise<std_msgs::UInt8>("/navdog/navigation_mode", 1);
  final_cmd_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("/navdog/final_cmd", 1);
  control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_),
      &NavdogRuntimeNode::controlCallback, this);
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
      last_route_progress_ = navdog::RouteProgress{};
      navdog::NavigationTask task{};
      if (coordinator_.copyActiveTask(task)) publishRoute(task);
      ROS_INFO("task started: task_sequence=%lu", static_cast<unsigned long>(event.task.sequence));
    }
    else if (result == navdog::TaskHandleResult::CANCELLED)
    {
      last_route_progress_ = navdog::RouteProgress{};
      pending_planner_feedback_ = navdog::PlannerFeedback{};
      ROS_INFO("task cancelled");
    }
    else if (result == navdog::TaskHandleResult::PAUSED)
      ROS_INFO("navigation paused");
    else if (result == navdog::TaskHandleResult::RESUMED)
      ROS_INFO("navigation resumed");
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

  geometry_msgs::Twist command = toTwist(output.final_cmd);
  if (!hasUniqueCmdVelPublisher())
  {
    ROS_ERROR_THROTTLE(2.0, "multiple %s publishers detected; forcing zero", cmd_vel_topic_.c_str());
    command = geometry_msgs::Twist{};
  }
  cmd_vel_publisher_.publish(command);
  publishDebug(output, now_sec);
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
        static_cast<unsigned>(output.final_cmd.source), output.final_cmd.vx,
        output.final_cmd.vy, output.final_cmd.yaw_rate);
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

void NavdogRuntimeNode::publishDebug(const navdog::CoreOutput& output, double now_sec)
{
  std_msgs::UInt8 state; state.data = static_cast<std::uint8_t>(output.state);
  std_msgs::UInt8 mode; mode.data = static_cast<std::uint8_t>(output.navigation_mode.mode);
  state_publisher_.publish(state);
  mode_publisher_.publish(mode);
  geometry_msgs::TwistStamped command;
  command.header.stamp.fromSec(now_sec);
  command.twist = toTwist(output.final_cmd);
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

void NavdogRuntimeNode::statusForOutput(
    const navdog::CoreOutput& output, bool protocol_error, int& status, int& error)
{
  status = 0; error = protocol_error ? 1 : 0;
  if (output.state == navdog::NavState::PAUSED) status = 5;
  else if (output.state == navdog::NavState::PLANNING ||
           output.state == navdog::NavState::START_ALIGN ||
           output.state == navdog::NavState::TRACKING ||
           output.state == navdog::NavState::GOAL_ALIGN ||
           output.state == navdog::NavState::RECOVERY) status = 1;
  if (output.state == navdog::NavState::FAILED ||
      output.state == navdog::NavState::EMERGENCY_STOP ||
      output.final_cmd.source == navdog::CommandSource::SAFETY_STOP)
  { status = 0; error = 2; }
}

void NavdogRuntimeNode::publishMqttStatus(const navdog::CoreOutput& output)
{
  if (!mqtt_) return;
  int status = 0, error = 0;
  statusForOutput(output, mqtt_->consumeProtocolError() > 0, status, error);
  Json::Value root;
  root["status"] = status; root["error"] = error;
  root["velocity"]["vx"] = output.final_cmd.vx;
  root["velocity"]["vy"] = output.final_cmd.vy;
  root["velocity"]["yaw_rate"] = output.final_cmd.yaw_rate;
  Json::StreamWriterBuilder writer; writer["indentation"] = "";
  mqtt_->publishStatus(Json::writeString(writer, root));
}

bool NavdogRuntimeNode::hasUniqueCmdVelPublisher() const
{
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();
  if (!ros::master::execute("getSystemState", args, result, payload, true) ||
      payload.getType() != XmlRpc::XmlRpcValue::TypeArray || payload.size() < 1)
    return false;
  const auto& publishers = payload[0];
  for (int i = 0; i < publishers.size(); ++i)
  {
    if (static_cast<std::string>(publishers[i][0]) == cmd_vel_topic_)
      return publishers[i][1].size() == 1 &&
          static_cast<std::string>(publishers[i][1][0]) == ros::this_node::getName();
  }
  return false;
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
