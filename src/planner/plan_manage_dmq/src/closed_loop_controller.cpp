#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <Eigen/Eigen>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <tf/tf.h>

#include "bspline_opt/uniform_bspline.h"
#include "plan_manage_dmq/route_path_tracker.h"
#include "scan_planner/Bspline.h"

namespace
{
using scan_planner::UniformBspline;

constexpr double kMaxVYawLimit = 1.0;
constexpr double kTakeoverTimestampToleranceSec = 0.10;
constexpr std::uint8_t kModeRouteFollow = 1;
constexpr std::uint8_t kModeLocalAvoid = 2;

ros::Publisher cmd_vel_pub;
ros::Publisher execution_frozen_pub;
ros::Publisher takeover_ready_pub;
ros::Subscriber bspline_sub;
ros::Subscriber odom_sub;
ros::Subscriber reset_sub;
ros::Subscriber takeover_sync_sub;
ros::Subscriber route_path_sub;
ros::Subscriber navigation_mode_sub;
ros::Timer cmd_timer;

bool receive_traj = false;
bool have_odom = false;
bool waiting_takeover_trajectory = false;
bool stationary_trajectory = false;
double takeover_anchor_tolerance = 0.25;
ros::Time takeover_sync_time;
std::vector<UniformBspline> traj;
double traj_duration = 0.0;
int traj_id = 0;

Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;

double exec_time = 0.0;
ros::Time last_update_time;

double time_forward;

double kp_pos;
double kp_yaw;
double max_vx;
double max_vy;
double max_vyaw;
double finish_dist;
std::string body_pose_topic;
std::uint8_t navigation_mode = 0;
scan_planner_dmq::RoutePathTrackerConfig route_tracker_config;
std::unique_ptr<scan_planner_dmq::RoutePathTracker> route_tracker;

bool loadRequiredParam(const ros::NodeHandle &nh, const std::string &name, double &value)
{
  if (nh.getParam(name, value))
    return true;

  ROS_ERROR_STREAM("[closed_loop_controller_dmq] missing required private parameter ~" << name);
  return false;
}

bool loadParams(const ros::NodeHandle &nh)
{
  bool ok = true;

  ros::param::param<std::string>(
      "/body_pose_topic",
      body_pose_topic,
      std::string("/quad_0/body_pose"));

  ok &= loadRequiredParam(
      nh,
      "time_forward",
      time_forward);

  ok &= loadRequiredParam(
      nh,
      "kp_pos",
      kp_pos);

  ok &= loadRequiredParam(
      nh,
      "kp_yaw",
      kp_yaw);

  ok &= loadRequiredParam(
      nh,
      "max_vx",
      max_vx);
  nh.param("speed_limits/local_avoid_linear_mps", max_vx, max_vx);

  ok &= loadRequiredParam(
      nh,
      "max_vy",
      max_vy);

  ok &= loadRequiredParam(
      nh,
      "max_vyaw",
      max_vyaw);

  ok &= loadRequiredParam(
      nh,
      "finish_dist",
      finish_dist);
  nh.param("takeover_anchor_tolerance", takeover_anchor_tolerance, 0.25);
  nh.param("route_tracking/lookahead_distance_m",
      route_tracker_config.lookahead_distance_m,
      route_tracker_config.lookahead_distance_m);
  nh.param("route_tracking/heading_lookahead_m",
      route_tracker_config.heading_lookahead_m,
      route_tracker_config.heading_lookahead_m);
  nh.param("route_tracking/max_forward_search_m",
      route_tracker_config.max_forward_search_m,
      route_tracker_config.max_forward_search_m);
  nh.param("route_tracking/near_goal_slowdown_m",
      route_tracker_config.near_goal_slowdown_m,
      route_tracker_config.near_goal_slowdown_m);
  nh.param("route_tracking/kp_position",
      route_tracker_config.kp_position,
      route_tracker_config.kp_position);
  nh.param("route_tracking/kp_yaw",
      route_tracker_config.kp_yaw,
      route_tracker_config.kp_yaw);
  nh.param("route_tracking/max_vx",
      route_tracker_config.max_vx,
      route_tracker_config.max_vx);
  nh.param("speed_limits/route_follow_linear_mps",
      route_tracker_config.max_vx,
      route_tracker_config.max_vx);
  nh.param("route_tracking/max_yaw_rate",
      route_tracker_config.max_yaw_rate,
      route_tracker_config.max_yaw_rate);

  if (!ok)
    return false;

  if (!std::isfinite(time_forward) ||
      time_forward < 0.0)
  {
    ROS_ERROR(
        "[closed_loop_controller_dmq] "
        "time_forward must be finite and >= 0");
    ok = false;
  }

  if (!std::isfinite(kp_pos) ||
      kp_pos < 0.0)
  {
    ROS_ERROR(
        "[closed_loop_controller_dmq] "
        "kp_pos must be finite and >= 0");
    ok = false;
  }

  if (!std::isfinite(kp_yaw) ||
      kp_yaw < 0.0)
  {
    ROS_ERROR(
        "[closed_loop_controller_dmq] "
        "kp_yaw must be finite and >= 0");
    ok = false;
  }

  if (!std::isfinite(max_vx) ||
      max_vx <= 0.0 ||
      !std::isfinite(max_vy) ||
      max_vy <= 0.0 ||
      !std::isfinite(max_vyaw) ||
      max_vyaw <= 0.0)
  {
    ROS_ERROR(
        "[closed_loop_controller_dmq] "
        "max_vx/max_vy/max_vyaw "
        "must be finite and > 0");
    ok = false;
  }

  if (!std::isfinite(finish_dist) ||
      finish_dist < 0.0)
  {
    ROS_ERROR(
        "[closed_loop_controller_dmq] "
        "finish_dist must be finite and >= 0");
    ok = false;
  }
  if (!std::isfinite(takeover_anchor_tolerance) ||
      takeover_anchor_tolerance < 0.0)
  {
    ROS_ERROR("[closed_loop_controller_dmq] takeover_anchor_tolerance must be finite and >= 0");
    ok = false;
  }
  if (!std::isfinite(route_tracker_config.lookahead_distance_m) ||
      route_tracker_config.lookahead_distance_m <= 0.0 ||
      !std::isfinite(route_tracker_config.max_vx) ||
      route_tracker_config.max_vx <= 0.0 ||
      !std::isfinite(route_tracker_config.max_yaw_rate) ||
      route_tracker_config.max_yaw_rate <= 0.0)
  {
    ROS_ERROR("[closed_loop_controller_dmq] invalid route_tracking configuration");
    ok = false;
  }

  if (!ok)
    return false;

  if (max_vyaw > kMaxVYawLimit)
  {
    ROS_WARN(
        "[closed_loop_controller_dmq] "
        "cap max_vyaw %.3f to %.3f rad/s.",
        max_vyaw,
        kMaxVYawLimit);

    max_vyaw = kMaxVYawLimit;
  }

  ROS_INFO(
      "[closed_loop_controller_dmq] "
      "TRACK_CONFIG "
      "time_forward=%.3f "
      "kp_pos=%.3f "
      "kp_yaw=%.3f "
      "max_vx=%.3f "
      "max_vy=%.3f "
      "max_vyaw=%.3f",
      time_forward,
      kp_pos,
      kp_yaw,
      max_vx,
      max_vy,
      max_vyaw);

  ROS_INFO(
      "[closed_loop_controller_dmq] ROUTE_TRACK_CONFIG "
      "lookahead=%.3f heading_lookahead=%.3f kp_pos=%.3f kp_yaw=%.3f "
      "max_vx=%.3f max_yaw_rate=%.3f",
      route_tracker_config.lookahead_distance_m,
      route_tracker_config.heading_lookahead_m,
      route_tracker_config.kp_position,
      route_tracker_config.kp_yaw,
      route_tracker_config.max_vx,
      route_tracker_config.max_yaw_rate);

  return true;
}


double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

Eigen::Vector2d clampNorm(
    const Eigen::Vector2d &value,
    double max_norm)
{
  const double norm = value.norm();

  if (norm <= max_norm ||
      norm < 1e-6)
  {
    return value;
  }

  return value / norm * max_norm;
}

double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des)
{
  const double t_look = std::min(traj_duration, t_cur + time_forward);
  Eigen::Vector3d dir = traj[0].evaluateDeBoorT(t_look) - pos_des;

  if (dir.head<2>().squaredNorm() < 1e-4)
  {
    Eigen::Vector3d vel = traj[1].evaluateDeBoorT(t_cur);
    dir = vel;
  }

  if (dir.head<2>().squaredNorm() < 1e-4)
    return odom_yaw;

  return std::atan2(dir(1), dir(0));
}

void publishStop(double vyaw = 0.0)
{
  geometry_msgs::Twist cmd;
  cmd.angular.z = clamp(vyaw, -max_vyaw, max_vyaw);
  cmd_vel_pub.publish(cmd);
}

void publishExecutionFrozen(bool frozen)
{
  std_msgs::Bool msg;
  msg.data = frozen;
  execution_frozen_pub.publish(msg);
}

void publishTakeoverReady(bool ready)
{
  std_msgs::Bool msg;
  msg.data = ready;
  takeover_ready_pub.publish(msg);
}

void resetCallback(const std_msgs::EmptyConstPtr&)
{
  receive_traj = false;
  stationary_trajectory = false;

  traj.clear();
  traj_duration = 0.0;
  traj_id = 0;

  exec_time = 0.0;
  last_update_time = ros::Time::now();

  publishExecutionFrozen(false);
  waiting_takeover_trajectory = false;
  takeover_sync_time = ros::Time();
  publishTakeoverReady(false);
  publishStop();
  if (route_tracker) route_tracker->reset();

  ROS_WARN(
      "[closed_loop_controller_dmq] "
      "NATIVE_SCAN_CONTROLLER_RESET");
}

void routePathCallback(const nav_msgs::PathConstPtr& msg)
{
  if (!msg || msg->poses.size() < 2 || !route_tracker)
  {
    ROS_WARN_THROTTLE(1.0,
        "[closed_loop_controller_dmq] reject empty route path");
    return;
  }

  std::vector<Eigen::Vector3d> points;
  points.reserve(msg->poses.size());
  for (const auto& pose : msg->poses)
  {
    points.emplace_back(
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z);
  }

  if (!route_tracker->setPath(points))
  {
    ROS_ERROR("[closed_loop_controller_dmq] ROUTE_TRACK_PATH_REJECTED points=%zu",
        points.size());
    return;
  }

  ROS_INFO("[closed_loop_controller_dmq] ROUTE_TRACK_PATH_ACCEPTED points=%zu",
      points.size());
}

void navigationModeCallback(const std_msgs::UInt8ConstPtr& msg)
{
  if (!msg) return;
  const std::uint8_t previous_mode = navigation_mode;
  navigation_mode = msg->data;
  if (navigation_mode == kModeRouteFollow &&
      previous_mode != kModeRouteFollow && route_tracker)
  {
    route_tracker->requestReacquire();
    ROS_INFO("[closed_loop_controller_dmq] ROUTE_TRACK_REACQUIRE");
  }
}

void takeoverSyncCallback(const std_msgs::EmptyConstPtr&)
{
  receive_traj = false;
  stationary_trajectory = false;
  waiting_takeover_trajectory = true;
  takeover_sync_time = ros::Time::now();
  traj.clear();
  traj_duration = 0.0;
  traj_id = 0;
  exec_time = 0.0;
  last_update_time = ros::Time::now();
  publishTakeoverReady(false);
  publishStop();
  ROS_INFO("SCAN_TAKEOVER_CONTROLLER_FLUSH");
}

void bsplineCallback(const scan_planner::BsplineConstPtr &msg)
{
  if (!msg || msg->order < 1 || msg->pos_pts.size() < 4 ||
      msg->knots.size() < msg->pos_pts.size())
  {
    ROS_WARN("[closed_loop_controller_dmq] reject invalid bspline");
    return;
  }

  // A queued prewarm trajectory may arrive after the takeover flush. Only a
  // trajectory planned in response to this synchronization may take control.
  if (waiting_takeover_trajectory && !takeover_sync_time.isZero() &&
      msg->start_time + ros::Duration(kTakeoverTimestampToleranceSec) <
          takeover_sync_time)
  {
    ROS_DEBUG("SCAN_TAKEOVER_STALE_TRAJ_SKIPPED traj_id=%d", msg->traj_id);
    return;
  }

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());

  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    if (!std::isfinite(msg->knots[i]))
    {
      ROS_WARN("[closed_loop_controller_dmq] reject non-finite bspline knot");
      return;
    }
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    if (!std::isfinite(msg->pos_pts[i].x) ||
        !std::isfinite(msg->pos_pts[i].y) ||
        !std::isfinite(msg->pos_pts[i].z))
    {
      ROS_WARN("[closed_loop_controller_dmq] reject non-finite bspline control point");
      return;
    }
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  std::vector<UniformBspline> candidate;
  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);
  candidate.push_back(pos_traj);
  candidate.push_back(candidate[0].getDerivative());
  candidate.push_back(candidate[1].getDerivative());
  const double candidate_duration = candidate[0].getTimeSum();
  if (!std::isfinite(candidate_duration) || candidate_duration <= 0.0)
  {
    ROS_WARN("[closed_loop_controller_dmq] reject bspline with invalid duration");
    return;
  }

  const double start_age = std::max(0.0, std::min(candidate_duration,
      (ros::Time::now() - msg->start_time).toSec()));
  double anchor_error = 0.0;
  if (waiting_takeover_trajectory)
  {
    const Eigen::Vector3d anchor = candidate[0].evaluateDeBoorT(start_age);
    anchor_error = (anchor.head<2>() - odom_pos.head<2>()).norm();
    if (anchor_error > takeover_anchor_tolerance)
    {
      ROS_ERROR("SCAN_TAKEOVER_TRAJ_REJECTED traj_id=%d anchor_error=%.3f tolerance=%.3f",
          msg->traj_id, anchor_error, takeover_anchor_tolerance);
      return;
    }
  }

  traj = std::move(candidate);
  traj_duration = candidate_duration;
  traj_id = msg->traj_id;
  stationary_trajectory = true;
  for (int i = 1; i < pos_pts.cols(); ++i)
  {
    if ((pos_pts.col(i) - pos_pts.col(0)).norm() > 1e-6)
    {
      stationary_trajectory = false;
      break;
    }
  }
  exec_time = start_age;
  last_update_time = ros::Time::now();
  receive_traj = true;
  if (waiting_takeover_trajectory)
  {
    waiting_takeover_trajectory = false;
    takeover_sync_time = ros::Time();
    publishTakeoverReady(true);
    ROS_INFO("SCAN_TAKEOVER_TRAJ_READY traj_id=%d anchor_error=%.3f exec_time=%.3f",
        traj_id, anchor_error, exec_time);
  }

  ROS_DEBUG("[closed_loop_controller_dmq] received bspline traj_id=%d duration=%.3f", traj_id, traj_duration);
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;
  odom_yaw = tf::getYaw(msg->pose.pose.orientation);
  have_odom = true;
}

void cmdCallback(const ros::TimerEvent &)
{
  if (!have_odom)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  if (navigation_mode == kModeRouteFollow)
  {
    publishExecutionFrozen(true);
    if (!route_tracker)
    {
      publishStop();
      return;
    }

    const auto route_output = route_tracker->update(odom_pos, odom_yaw);
    if (!route_output.valid)
    {
      ROS_WARN_THROTTLE(1.0,
          "[closed_loop_controller_dmq] ROUTE_TRACK_WAITING_PATH");
      publishStop();
      return;
    }

    geometry_msgs::Twist cmd;
    cmd.linear.x = route_output.vx;
    cmd.linear.y = 0.0;
    cmd.angular.z = route_output.yaw_rate;
    cmd_vel_pub.publish(cmd);
    ROS_DEBUG_THROTTLE(1.0,
        "ROUTE_TRACK_CONTROL progress=%.3f remaining=%.3f "
        "target=(%.3f,%.3f) cmd=(%.3f,0.000,%.3f)",
        route_output.progress_m, route_output.remaining_m,
        route_output.target_x, route_output.target_y,
        route_output.vx, route_output.yaw_rate);
    return;
  }

  if (navigation_mode != kModeLocalAvoid || !receive_traj)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  // EmergencyStop is encoded as a B-spline whose control points are all the
  // same. Position feedback on that spline would otherwise command the robot
  // back toward a stale stop point as odometry drifts.
  if (stationary_trajectory)
  {
    publishExecutionFrozen(false);
    publishStop();
    last_update_time = ros::Time::now();
    return;
  }

  const ros::Time now = ros::Time::now();
  double dt = (now - last_update_time).toSec();
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  const double t_eval = std::min(exec_time, traj_duration);
  Eigen::Vector3d pos_des = traj[0].evaluateDeBoorT(t_eval);
  Eigen::Vector3d vel_des = traj[1].evaluateDeBoorT(t_eval);

  const double yaw_des =
      estimateDesiredYaw(
          t_eval,
          pos_des);

  const double yaw_err =
      normalizeAngle(
          yaw_des -
          odom_yaw);

  const double vyaw_cmd =
      clamp(
          kp_yaw * yaw_err,
          -max_vyaw,
          max_vyaw);

  // SCAN轨迹始终保持执行状态。
  // 机器人可以边沿轨迹移动，边调整自身航向。
  publishExecutionFrozen(false);

  exec_time =
      std::min(
          traj_duration,
          exec_time + dt);

  last_update_time = now;

  pos_des = traj[0].evaluateDeBoorT(exec_time);
  vel_des = traj[1].evaluateDeBoorT(exec_time);

  Eigen::Vector2d pos_err(
      pos_des(0) -
          odom_pos(0),
      pos_des(1) -
          odom_pos(1));

  Eigen::Vector2d vel_ff(
      vel_des(0),
      vel_des(1));

  Eigen::Vector2d vel_world =
      clampNorm(
          vel_ff +
              kp_pos *
              pos_err,
          std::max(
              max_vx,
              max_vy));


  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  geometry_msgs::Twist cmd;
  cmd.linear.x = clamp(c * vel_world(0) + s * vel_world(1), -max_vx, max_vx);
  cmd.linear.y = clamp(-s * vel_world(0) + c * vel_world(1), -max_vy, max_vy);
  cmd.angular.z = vyaw_cmd;

  if (exec_time >= traj_duration && pos_err.norm() < finish_dist)
    cmd = geometry_msgs::Twist();

  ROS_DEBUG_THROTTLE(
    1.0,
    "SCAN_TRACK_CONTROL "
    "traj_id=%d "
    "yaw_des=%.3f "
    "odom_yaw=%.3f "
    "yaw_error=%.3f "
    "exec_time=%.3f "
    "duration=%.3f",
    traj_id,
    yaw_des,
    odom_yaw,
    yaw_err,
    exec_time,
    traj_duration);

  cmd_vel_pub.publish(cmd);
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "closed_loop_controller_dmq");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  if (!loadParams(nh))
    return 1;
  route_tracker.reset(
      new scan_planner_dmq::RoutePathTracker(route_tracker_config));

  bspline_sub = node.subscribe("/native_scan/planning/bspline", 10, bsplineCallback);
  odom_sub = node.subscribe(body_pose_topic, 20, odomCallback, ros::TransportHints().tcpNoDelay());
  reset_sub = node.subscribe("/native_scan/reset", 10, resetCallback);
  takeover_sync_sub = node.subscribe("/native_scan/takeover_sync", 10,
      takeoverSyncCallback);
  route_path_sub = node.subscribe("/native_scan/initial_path", 1,
      routePathCallback);
  navigation_mode_sub = node.subscribe("/navdog/navigation_mode", 10,
      navigationModeCallback);
  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/navdog/scan_cmd", 20);
  execution_frozen_pub = node.advertise<std_msgs::Bool>("/native_scan/planning/go2_execution_frozen", 10);
  takeover_ready_pub = node.advertise<std_msgs::Bool>("/native_scan/takeover_ready", 1, true);
  publishTakeoverReady(false);
  cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  last_update_time = ros::Time::now();
  ROS_WARN("[closed_loop_controller_dmq] ready.");

  ros::spin();
  return 0;
}
