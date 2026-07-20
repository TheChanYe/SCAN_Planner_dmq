#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Eigen>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <tf/tf.h>

#include "bspline_opt/uniform_bspline.h"
#include "scan_planner/Bspline.h"

namespace
{
using scan_planner::UniformBspline;

constexpr double kMaxVYawLimit = 1.0;

ros::Publisher cmd_vel_pub;
ros::Publisher execution_frozen_pub;
ros::Subscriber bspline_sub;
ros::Subscriber odom_sub;
ros::Subscriber reset_sub;
ros::Timer cmd_timer;

bool receive_traj = false;
bool have_odom = false;
std::vector<UniformBspline> traj;
double traj_duration = 0.0;
int traj_id = 0;

Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;

double exec_time = 0.0;
ros::Time last_update_time;

double time_forward;
// 开始降低线速度的航向误差。
double heading_error_threshold;

// 航向误差小于该值后恢复全速。
// 同时用于形成航向控制的平滑区间。
double heading_resume_threshold;

// 大航向误差时保留的最小线速度比例。
double min_heading_speed_scale;

double kp_pos;
double kp_yaw;
double max_vx;
double max_vy;
double max_vyaw;
double finish_dist;
std::string body_pose_topic;

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
      "heading_error_threshold",
      heading_error_threshold);

  ok &= loadRequiredParam(
      nh,
      "heading_resume_threshold",
      heading_resume_threshold);

  ok &= loadRequiredParam(
      nh,
      "min_heading_speed_scale",
      min_heading_speed_scale);

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

  if (!std::isfinite(heading_resume_threshold) ||
      heading_resume_threshold < 0.0)
  {
    ROS_ERROR(
        "[closed_loop_controller_dmq] "
        "heading_resume_threshold must be "
        "finite and >= 0");
    ok = false;
  }

  if (!std::isfinite(heading_error_threshold) ||
      heading_error_threshold <=
          heading_resume_threshold)
  {
    ROS_ERROR(
        "[closed_loop_controller_dmq] "
        "heading_error_threshold must be "
        "finite and greater than "
        "heading_resume_threshold");
    ok = false;
  }

  if (!std::isfinite(min_heading_speed_scale) ||
      min_heading_speed_scale < 0.0 ||
      min_heading_speed_scale > 1.0)
  {
    ROS_ERROR(
        "[closed_loop_controller_dmq] "
        "min_heading_speed_scale must be in [0, 1]");
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
      "heading_resume=%.3f "
      "heading_error=%.3f "
      "min_heading_scale=%.3f "
      "kp_pos=%.3f "
      "kp_yaw=%.3f "
      "max_vx=%.3f "
      "max_vy=%.3f "
      "max_vyaw=%.3f",
      time_forward,
      heading_resume_threshold,
      heading_error_threshold,
      min_heading_speed_scale,
      kp_pos,
      kp_yaw,
      max_vx,
      max_vy,
      max_vyaw);

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

/**
 * @brief 根据避障轨迹航向误差计算线速度缩放比例。
 *
 * 误差较小时保持完整线速度；
 * 误差位于两个阈值之间时线性降低；
 * 误差很大时保留min_heading_speed_scale，
 * 不再完全停止并原地旋转。
 */
double computeHeadingSpeedScale(
    double abs_yaw_error)
{
  if (!std::isfinite(abs_yaw_error))
    return 0.0;

  // 误差很小，完整跟踪轨迹。
  if (abs_yaw_error <=
      heading_resume_threshold)
  {
    return 1.0;
  }

  // 误差已经达到或超过大误差阈值，
  // 仍保留最小线速度，允许边转边走。
  if (abs_yaw_error >=
      heading_error_threshold)
  {
    return min_heading_speed_scale;
  }

  // 在resume阈值和error阈值之间线性插值。
  const double threshold_range =
      heading_error_threshold -
      heading_resume_threshold;

  if (threshold_range <= 1e-6)
    return min_heading_speed_scale;

  const double interpolation =
      clamp(
          (abs_yaw_error -
           heading_resume_threshold) /
              threshold_range,
          0.0,
          1.0);

  return 1.0 -
      interpolation *
          (1.0 -
           min_heading_speed_scale);
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

void resetCallback(const std_msgs::EmptyConstPtr&)
{
  receive_traj = false;

  traj.clear();
  traj_duration = 0.0;
  traj_id = 0;

  exec_time = 0.0;
  last_update_time = ros::Time::now();

  publishExecutionFrozen(false);
  publishStop();

  ROS_WARN(
      "[closed_loop_controller_dmq] "
      "NATIVE_SCAN_CONTROLLER_RESET");
}

void bsplineCallback(const scan_planner::BsplineConstPtr &msg)
{
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());

  for (size_t i = 0; i < msg->knots.size(); ++i)
    knots(i) = msg->knots[i];

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  traj.clear();
  traj.push_back(pos_traj);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  traj_duration = traj[0].getTimeSum();
  traj_id = msg->traj_id;
  exec_time = 0.0;
  last_update_time = ros::Time::now();
  receive_traj = true;

  ROS_WARN("[closed_loop_controller_dmq] received bspline traj_id=%d duration=%.3f", traj_id, traj_duration);
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
  if (!receive_traj || !have_odom)
  {
    publishExecutionFrozen(false);
    publishStop();
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

  const double abs_yaw_err =
      std::abs(
          yaw_err);

  const double vyaw_cmd =
      clamp(
          kp_yaw * yaw_err,
          -max_vyaw,
          max_vyaw);

  // 根据航向误差平滑降低线速度。
  // 不再因为误差超过heading_error_threshold
  // 就冻结SCAN轨迹并原地旋转。
  const double heading_speed_scale =
      computeHeadingSpeedScale(
          abs_yaw_err);

  // SCAN轨迹始终保持执行状态。
  // 机器人可以边沿轨迹移动，边调整自身航向。
  publishExecutionFrozen(false);

  // 航向误差较大时，参考轨迹时间也减速推进。
  // 防止机器人还没有调整好方向，轨迹参考点就快速跑远。
  exec_time =
      std::min(
          traj_duration,
          exec_time +
              dt *
              heading_speed_scale);

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

  // 航向误差越大，世界坐标系中的线速度越低。
  // 最低仍保留min_heading_speed_scale比例，
  // 因此不会再次出现完整原地旋转。
  vel_world *=
      heading_speed_scale;

  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  geometry_msgs::Twist cmd;
  cmd.linear.x = clamp(c * vel_world(0) + s * vel_world(1), -max_vx, max_vx);
  cmd.linear.y = clamp(-s * vel_world(0) + c * vel_world(1), -max_vy, max_vy);
  cmd.angular.z = vyaw_cmd;

  if (exec_time >= traj_duration && pos_err.norm() < finish_dist)
    cmd = geometry_msgs::Twist();

  ROS_INFO_THROTTLE(
    0.5,
    "SCAN_TRACK_CONTROL "
    "traj_id=%d "
    "yaw_des=%.3f "
    "odom_yaw=%.3f "
    "yaw_error=%.3f "
    "heading_scale=%.3f "
    "exec_time=%.3f "
    "duration=%.3f",
    traj_id,
    yaw_des,
    odom_yaw,
    yaw_err,
    heading_speed_scale,
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

  bspline_sub = node.subscribe("/native_scan/planning/bspline", 10, bsplineCallback);
  odom_sub = node.subscribe(body_pose_topic, 20, odomCallback, ros::TransportHints().tcpNoDelay());
  reset_sub = node.subscribe("/native_scan/reset", 10, resetCallback);
  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/navdog/scan_cmd", 20);
  execution_frozen_pub = node.advertise<std_msgs::Bool>("/native_scan/planning/go2_execution_frozen", 10);
  cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  last_update_time = ros::Time::now();
  ROS_WARN("[closed_loop_controller_dmq] ready.");

  ros::spin();
  return 0;
}
