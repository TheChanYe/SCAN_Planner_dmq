#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <Eigen/Eigen>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
#include <tf/tf.h>

#include "bspline_opt/uniform_bspline.h"
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
ros::Publisher takeover_replan_pub;
ros::Subscriber bspline_sub;
ros::Subscriber odom_sub;
ros::Subscriber reset_sub;
ros::Subscriber takeover_sync_sub;
ros::Subscriber navigation_mode_sub;
ros::Timer cmd_timer;

bool receive_traj = false;
bool have_odom = false;
bool waiting_takeover_trajectory = false;
bool stationary_trajectory = false;
double takeover_anchor_tolerance = 0.25;
ros::Time takeover_sync_time;
std::uint32_t takeover_generation = 0;
std::uint32_t ready_generation = 0;
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
double stair_up_linear_speed_mps = 0.40;
double max_vy;
double max_vyaw;
double finish_dist;
std::string body_pose_topic;
std::uint8_t navigation_mode = 0;
bool stair_up_active = false;

// loadRequiredParam：从私有参数服务器读取必需参数，若不存在则打印错误并返回false。
bool loadRequiredParam(const ros::NodeHandle &nh, const std::string &name, double &value)
{
  if (nh.getParam(name, value))
    return true;

  ROS_ERROR_STREAM("[closed_loop_controller_dmq] missing required private parameter ~" << name);
  return false;
}

// loadParams：加载并校验闭环控制器全部参数（前矢时间/比例增益/速度上限/完成距离、
// 接管锚点容差）。每个必需参数缺失或非法均记录ok=false，最后对
// max_vyaw做安全上限截断并输出配置日志。任一项无效则返回false。
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
  nh.param("stair_up/linear_speed_mps", stair_up_linear_speed_mps, 0.40);

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
      !std::isfinite(stair_up_linear_speed_mps) ||
      stair_up_linear_speed_mps <= 0.0 ||
      !std::isfinite(max_vyaw) ||
      max_vyaw <= 0.0)
  {
    ROS_ERROR(
        "[closed_loop_controller_dmq] "
        "max_vx/stair_up_linear_speed_mps/max_vy/max_vyaw "
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
      "stair_vx=%.3f "
      "max_vy=%.3f "
      "max_vyaw=%.3f",
      time_forward,
      kp_pos,
      kp_yaw,
      max_vx,
      stair_up_linear_speed_mps,
      max_vy,
      max_vyaw);

  return true;
}


// normalizeAngle：将角度归一化到[-pi, pi]区间。
double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

// clamp：将value限制在[min_value, max_value]区间内。
double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

// clampNorm：若二维向量value的模超过max_norm，按比例缩放至上限。
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

// estimateDesiredYaw：估计当前期望朝向。优先取前矢时间后的轨迹点与当前期望位置
// 的连线方向；若两者过于接近则回退使用当前时刻的速度方向；若仍接近零则保持
// 当前里程计朝向不变。
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

void publishCommand(const geometry_msgs::Twist& cmd)
{ cmd_vel_pub.publish(cmd); }

// publishStop：发布零线速度、仅保留角速度（限幅后）的停车指令，默认角速度为0。
void publishStop(double vyaw = 0.0)
{
  geometry_msgs::Twist cmd;
  cmd.angular.z = clamp(vyaw, -max_vyaw, max_vyaw);
  publishCommand(cmd);
}

// publishExecutionFrozen：发布Go2执行冻结状态信号（告诉SCAN规划端是否暂停
// 时间推进）。
void publishExecutionFrozen(bool frozen)
{
  std_msgs::Bool msg;
  msg.data = frozen;
  execution_frozen_pub.publish(msg);
}

double activeLinearSpeedLimit()
{
  return stair_up_active ? stair_up_linear_speed_mps : max_vx;
}

// publishTakeoverReady：发布接管就绪generation，0表示未就绪。
void publishTakeoverReady(std::uint32_t generation)
{
  std_msgs::UInt32 msg;
  msg.data = generation;
  ready_generation = generation;
  takeover_ready_pub.publish(msg);
}

// resetCallback：接收外部重置信号，清空当前轨迹/执行状态/接管等全部临时状态，
// 并发布停止指令。
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
  takeover_generation = 0;
  publishTakeoverReady(0);
  publishStop();

  ROS_WARN(
      "[closed_loop_controller_dmq] "
      "NATIVE_SCAN_CONTROLLER_RESET");
}

// navigationModeCallback：订阅导航模式。
void navigationModeCallback(const std_msgs::UInt8ConstPtr& msg)
{
  if (!msg) return;
  navigation_mode = msg->data;
}

void stairUpActiveCallback(const std_msgs::BoolConstPtr& msg)
{
  if (!msg) return;
  stair_up_active = msg->data;
}

// takeoverSyncCallback：接收接管同步信号。清空当前轨迹并进入“等待接管轨迹”
// 状态，记录同步时刻供后续bsplineCallback判断轨迹时效性，并发布停车。
void takeoverSyncCallback(const std_msgs::UInt32ConstPtr& msg)
{
  if (!msg || msg->data == 0)
    return;
  waiting_takeover_trajectory = true;
  takeover_generation = msg->data;
  takeover_sync_time = ros::Time::now();
  publishTakeoverReady(0);
  std_msgs::UInt32 replan;
  replan.data = takeover_generation;
  takeover_replan_pub.publish(replan);
  ROS_INFO("SCAN_TAKEOVER_CONTROLLER_ARM generation=%u", takeover_generation);
}

// bsplineCallback：接收SCAN规划器发布的B样条轨迹消息。
// 步骤：
// 1. 校验阶数/控制点数/节点数基本合法性，不合法则拒绝；
// 2. 若处于等待接管轨迹状态且该轨迹的开始时刻早于同步时刻（超出容差），说明
//    是接管前预热轨迹的迟到消息，不能接管，直接丢弃；
// 3. 逐个校验节点与控制点的有限性，不合法则拒绝；
// 4. 构造位置B样条并求导得到速度/加速度（仅为候选，尚未接受），校验总时长合法；
// 5. 计算该轨迹已经过去的时长start_age（当前时刻相对轨迹发布时刻的延迟）；
// 6. 若处于等待接管状态，校验该轨迹在start_age时刻的位置与当前里程计位置的锚点
//    误差是否在容差内，超出则拒绝该轨迹（防止接管跳变）；
// 7. 接受轨迹，判断是否为静止轨迹（所有控制点重合，即EmergencyStop编码）；
// 8. 若之前处于等待接管状态，标记接管成功并发布就绪信号。
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
    publishTakeoverReady(takeover_generation);
    ROS_INFO("SCAN_TAKEOVER_TRAJ_READY generation=%u traj_id=%d anchor_error=%.3f exec_time=%.3f",
        takeover_generation, traj_id, anchor_error, exec_time);
  }

  ROS_DEBUG("[closed_loop_controller_dmq] received bspline traj_id=%d duration=%.3f", traj_id, traj_duration);
}

// odomCallback：订阅里程计，更新当前位置与朝向。
void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;
  odom_yaw = tf::getYaw(msg->pose.pose.orientation);
  have_odom = true;
}

// cmdCallback：固定频率（默认100Hz）的主控制循环，根据当前导航模式分支处理：
// 1. 无里程计时：不冻结且发布停车；
// 2. ROUTE_FOLLOW模式：SCAN不拥有速度控制权，冻结规划时间并发布零scan_cmd；
// 3. 非LOCAL_AVOID模式或尚未收到轨迹：不冻结且发布停车；
// 4. LOCAL_AVOID且为静止轨迹（EmergencyStop）：不对该轨迹做位置反馈跟踪（防止
//    里程计漂移后被拉回旧停止点），直接发布停车；
// 5. LOCAL_AVOID且为正常B样条轨迹：
//    a. 计算dt并容错归零；
//    b. 先在旧执行时刻t_eval采样位置/速度并估计期望朝向与朝向角速度指令；
//    c. 推进执行时刻exec_time（不超过轨迹总时长），在新时刻重新采样位置/速度；
//    d. 合成世界系目标速度：前馈速度+位置误差比例项，按模限幅；
//    e. 旋转到机体系得到vx/vy并限幅；
//    f. 若已到达轨迹末尾且位置误差小于完成距离，强制输出零指令；
//    g. 发布日志与最终指令。
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
    publishStop();
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

  const double active_max_vx = activeLinearSpeedLimit();
  Eigen::Vector2d vel_world =
      clampNorm(
          vel_ff +
              kp_pos *
              pos_err,
          std::max(
              active_max_vx,
              max_vy));


  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  geometry_msgs::Twist cmd;
  cmd.linear.x = clamp(c * vel_world(0) + s * vel_world(1),
      -active_max_vx, active_max_vx);
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

  publishCommand(cmd);
}
} // namespace

// main：closed_loop_controller_dmq节点入口。加载参数失败则退出；注册全部订阅/
// 发布者（B样条、里程计、重置、接管同步、导航模式）与发布者（速度指令、
// 执行冻结、接管就绪）；创建100Hz控制定时器；
// 最后进入ros::spin()。
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
  takeover_sync_sub = node.subscribe("/native_scan/takeover_sync", 10,
      takeoverSyncCallback);
  navigation_mode_sub = node.subscribe("/navdog/navigation_mode", 10,
      navigationModeCallback);
  ros::Subscriber stair_up_active_sub = node.subscribe(
      "/navdog/stair_up_active", 10, stairUpActiveCallback);
  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/navdog/scan_cmd", 20);
  execution_frozen_pub = node.advertise<std_msgs::Bool>("/native_scan/planning/go2_execution_frozen", 10);
  takeover_replan_pub = node.advertise<std_msgs::UInt32>(
      "/native_scan/takeover_replan", 1, false);
  takeover_ready_pub = node.advertise<std_msgs::UInt32>("/native_scan/takeover_ready", 1, true);
  publishTakeoverReady(0);
  cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  last_update_time = ros::Time::now();
  ROS_WARN("[closed_loop_controller_dmq] ready.");

  ros::spin();
  return 0;
}
