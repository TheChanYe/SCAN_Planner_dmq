#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <navdog_core/types.hpp>
#include <navdog_runtime/cmd_speed_limit.hpp>
#include <navdog_runtime/velocity_slew_limiter.hpp>

#include <cmath>
#include <cstdint>
#include <string>

namespace
{

constexpr double kEpsilon = 1e-9;

// Configurable parameters
double route_cmd_timeout_sec = 0.30;
double scan_cmd_timeout_sec = 0.30;
double publish_rate_hz = 50.0;
double mode_sync_grace_sec = 0.10;
double scan_handoff_hold_sec = 0.10;
double route_follow_linear_speed_mps = 0.70;
double local_avoid_linear_speed_mps = 0.30;
double stair_up_linear_speed_mps = 0.40;
std::string external_stop_topic{"/navdog/external_stop"};
std::string final_cmd_feedback_topic{"/navdog/final_cmd_feedback"};
std::string max_vx_limit_topic{"/navdog/max_vx_limit"};

// Current state
navdog::NavState nav_state_{navdog::NavState::IDLE};
navdog::NavigationMode navigation_mode_{navdog::NavigationMode::NONE};

geometry_msgs::Twist latest_route_cmd_{};
double route_cmd_stamp_sec_{0.0};

geometry_msgs::Twist latest_scan_cmd_{};
double scan_cmd_stamp_sec_{0.0};

// CommandOwner：当前允许写入/cmd_vel的指令来源方。
// ROUTE：route_cmd（StartAlign/Core RouteFollower/GoalAlign）；SCAN：scan_cmd。
enum class CommandOwner
{
  NONE,
  ROUTE,
  SCAN
};

// MotionClass：用于日志分类的输出运动状态枚举。
enum class MotionClass
{
  STOP,
  TURN,
  DRIVE
};

CommandOwner previous_owner_{CommandOwner::NONE};
double owner_change_stamp_sec_{0.0};
double nav_state_change_stamp_sec_{0.0};
double local_avoid_enter_stamp_sec_{0.0};
std::uint32_t expected_takeover_generation_{0};
std::uint32_t ready_takeover_generation_{0};
std::uint32_t logged_handoff_generation_{0};
geometry_msgs::Twist handoff_route_last_cmd_{};
bool external_stop_{false};
bool stair_up_active_{false};
double task_max_vx_{0.0};
bool task_max_vx_valid_{false};

// Velocity slew limiter — single instance for smooth handoff.
navdog_runtime::VelocitySlewLimiter slew_limiter_;
geometry_msgs::Twist last_output_cmd_{};
double last_publish_stamp_sec_{0.0};
bool limiter_initialized_{false};
MotionClass last_logged_motion_{MotionClass::STOP};
bool motion_log_initialized_{false};

ros::Publisher cmd_vel_pub_;
ros::Publisher final_cmd_feedback_pub_;

// ownerName：将CommandOwner枚举转换为可读字符串，供日志使用。
const char* ownerName(CommandOwner owner)
{
  switch (owner)
  {
    case CommandOwner::ROUTE: return "ROUTE";
    case CommandOwner::SCAN: return "SCAN";
    case CommandOwner::NONE:
    default:                  return "NONE";
  }
}

// effectiveOwner：根据当前导航状态与模式确定指令权归属于谁：
// START_ALIGN/GOAL_ALIGN阶段归ROUTE；TRACKING+ROUTE_FOLLOW归ROUTE；
// TRACKING+LOCAL_AVOID归SCAN；TRACKING但模式为NONE时视为过渡期，暂不分配
// 所有权；其余所有非运动状态（IDLE/PLANNING/PAUSED/FAILED/SUCCEEDED/
// RECOVERY/EMERGENCY_STOP）均不分配权限。
CommandOwner effectiveOwner()
{
  switch (nav_state_)
  {
    case navdog::NavState::START_ALIGN:
    case navdog::NavState::GOAL_ALIGN:
      return CommandOwner::ROUTE;

    case navdog::NavState::TRACKING:
      if (navigation_mode_ == navdog::NavigationMode::ROUTE_FOLLOW)
        return CommandOwner::ROUTE;
      if (navigation_mode_ == navdog::NavigationMode::LOCAL_AVOID)
        return CommandOwner::SCAN;
      // mode NONE during TRACKING — grace period.
      return CommandOwner::NONE;

    case navdog::NavState::IDLE:
    case navdog::NavState::PLANNING:
    case navdog::NavState::PAUSED:
    case navdog::NavState::FAILED:
    case navdog::NavState::SUCCEEDED:
    case navdog::NavState::RECOVERY:
    case navdog::NavState::EMERGENCY_STOP:
    default:
      return CommandOwner::NONE;
  }
}

// finiteTwist：校验Twist指令的六个分量是否均为有限数。
bool finiteTwist(const geometry_msgs::Twist& cmd)
{
  return std::isfinite(cmd.linear.x) &&
         std::isfinite(cmd.linear.y) &&
         std::isfinite(cmd.linear.z) &&
         std::isfinite(cmd.angular.x) &&
         std::isfinite(cmd.angular.y) &&
         std::isfinite(cmd.angular.z);
}

// isFresh：判断指定时间戳相对now_sec是否在有效新鲜期内（非负、不超时）。
bool isFresh(double stamp_sec, double now_sec, double timeout_sec)
{
  if (!std::isfinite(stamp_sec) || !std::isfinite(now_sec) ||
      stamp_sec <= 0.0 || timeout_sec <= 0.0)
    return false;

  const double age = now_sec - stamp_sec;
  return age >= 0.0 && age <= timeout_sec;
}

// zeroCommand：返回默认零速度的Twist。
geometry_msgs::Twist zeroCommand()
{
  return geometry_msgs::Twist{};
}

void publishFinalCommand(const geometry_msgs::Twist& command, double now_sec)
{
  cmd_vel_pub_.publish(command);
  geometry_msgs::TwistStamped feedback;
  feedback.header.stamp.fromSec(now_sec);
  feedback.twist = command;
  final_cmd_feedback_pub_.publish(feedback);
}

void limitModeLinearSpeed(geometry_msgs::Twist& command,
                          const char* mode_name)
{
  double effective_limit = 0.0;
  double mode_limit_mps = 0.0;
  if (navigation_mode_ == navdog::NavigationMode::ROUTE_FOLLOW)
  {
    mode_limit_mps = route_follow_linear_speed_mps;
    effective_limit = navdog_runtime::effectiveLinearSpeedLimit(
        route_follow_linear_speed_mps, task_max_vx_, task_max_vx_valid_);
  }
  else if (navigation_mode_ == navdog::NavigationMode::LOCAL_AVOID)
  {
    mode_limit_mps = navdog_runtime::localAvoidLinearSpeedLimit(
        local_avoid_linear_speed_mps,
        stair_up_linear_speed_mps,
        stair_up_active_);
    effective_limit = mode_limit_mps;
  }
  if (effective_limit <= 0.0)
  {
    command.linear.x = 0.0;
    command.linear.y = 0.0;
    return;
  }
  if (navigation_mode_ == navdog::NavigationMode::ROUTE_FOLLOW &&
      task_max_vx_valid_)
  {
    ROS_INFO_THROTTLE(1.0,
        "CMD_SPEED_LIMIT mode=%s task_max_vx=%.3f mode_limit=%.3f effective=%.3f",
        mode_name, task_max_vx_, mode_limit_mps, effective_limit);
  }
  const double requested_linear =
      std::hypot(command.linear.x, command.linear.y);
  const double scale = navdog_runtime::proportionalMotionScale(
      command.linear.x, command.linear.y, effective_limit);
  if (scale <= 0.0)
  {
    command.linear.x = 0.0;
    command.linear.y = 0.0;
    command.angular.z = 0.0;
    return;
  }
  if (scale >= 1.0)
    return;

  command.linear.x *= scale;
  command.linear.y *= scale;
  command.angular.z *= scale;
  ROS_INFO_THROTTLE(1.0,
      "CMD_MOTION_LIMIT mode=%s requested_linear=%.3f limited_linear=%.3f scale=%.3f",
      mode_name, requested_linear, effective_limit, scale);
}

// classifyMotion：根据输出指令粗略判断运动状态，仅用于日志分类。
MotionClass classifyMotion(const geometry_msgs::Twist& cmd)
{
  if (std::hypot(cmd.linear.x, cmd.linear.y) > 0.02)
    return MotionClass::DRIVE;
  if (std::fabs(cmd.angular.z) > 0.015)
    return MotionClass::TURN;
  return MotionClass::STOP;
}

// motionClassName：将MotionClass枚举转换为可读字符串。
const char* motionClassName(const MotionClass value)
{
  switch (value)
  {
    case MotionClass::DRIVE: return "DRIVE";
    case MotionClass::TURN: return "TURN";
    case MotionClass::STOP:
    default: return "STOP";
  }
}

// logOutputCommand：记录最终输出指令日志。运动分类发生变化时打印CMD_OUTPUT详情，
// 并按固定1Hz节流打印CMD_TRACE完整追踪信息（包含rote/scan指令龄期与SCAN接管
// 就绪状态）。
void logOutputCommand(const geometry_msgs::Twist& output,
                      const geometry_msgs::Twist& target,
                      const CommandOwner owner,
                      const bool target_valid,
                      const char* reason,
                      const double now_sec)
{
  const MotionClass motion = classifyMotion(output);
  const double route_age = route_cmd_stamp_sec_ > 0.0
      ? now_sec - route_cmd_stamp_sec_ : -1.0;
  const double scan_age = scan_cmd_stamp_sec_ > 0.0
      ? now_sec - scan_cmd_stamp_sec_ : -1.0;

  if (!motion_log_initialized_ || motion != last_logged_motion_)
  {
    ROS_INFO("CMD_OUTPUT motion=%s owner=%s state=%s mode=%s reason=%s "
             "target_valid=%d target=[%.3f %.3f %.3f] "
             "output=[%.3f %.3f %.3f] route_age=%.3f scan_age=%.3f",
        motionClassName(motion), ownerName(owner),
        navdog::navStateName(nav_state_),
        navdog::navigationModeName(navigation_mode_),
        reason ? reason : "UNKNOWN", target_valid ? 1 : 0,
        target.linear.x, target.linear.y, target.angular.z,
        output.linear.x, output.linear.y, output.angular.z,
        route_age, scan_age);
    last_logged_motion_ = motion;
    motion_log_initialized_ = true;
  }

  ROS_INFO_THROTTLE(1.0,
      "CMD_TRACE motion=%s owner=%s state=%s mode=%s reason=%s "
      "target_valid=%d target=[%.3f %.3f %.3f] "
      "output=[%.3f %.3f %.3f] route_age=%.3f scan_age=%.3f ready=%d",
      motionClassName(motion), ownerName(owner),
      navdog::navStateName(nav_state_),
      navdog::navigationModeName(navigation_mode_),
      reason ? reason : "UNKNOWN", target_valid ? 1 : 0,
      target.linear.x, target.linear.y, target.angular.z,
      output.linear.x, output.linear.y, output.angular.z,
      route_age, scan_age,
      navdog_runtime::takeoverGenerationReady(
          expected_takeover_generation_, ready_takeover_generation_) ? 1 : 0);
}

// routeCmdCallback：订阅Route阶段的速度指令，校验有限性后保存最新指令与时间戳。
void routeCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  if (!finiteTwist(msg->twist))
  {
    ROS_WARN_THROTTLE(1.0, "INVALID_ROUTE_CMD");
    return;
  }
  latest_route_cmd_ = msg->twist;
  route_cmd_stamp_sec_ = msg->header.stamp.toSec();
}

// scanCmdCallback：订阅SCAN（本地避障/跟踪）的速度指令，校验有限性后保存最新
// 指令与接收时刻（使用ros::Time::now()而非消息自身时间戳，避免SCAN时间戳不同步）。
void scanCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  if (!finiteTwist(*msg))
  {
    ROS_WARN_THROTTLE(1.0, "INVALID_SCAN_CMD");
    return;
  }
  latest_scan_cmd_ = *msg;
  scan_cmd_stamp_sec_ = ros::Time::now().toSec();
}

// stateCallback：订阅导航状态，若状态发生变化则记录变化时刻（供模式同步宽限使用）。
void stateCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  const auto previous = nav_state_;
  nav_state_ = static_cast<navdog::NavState>(msg->data);
  if (nav_state_ != previous)
    nav_state_change_stamp_sec_ = ros::Time::now().toSec();
}

// modeCallback：订阅导航模式。进入LOCAL_AVOID时记录进入时刻并重置SCAN接管
// 就绪/前进确认标志（要求重新确认）。
void modeCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  const auto previous = navigation_mode_;
  navigation_mode_ = static_cast<navdog::NavigationMode>(msg->data);
  if (navigation_mode_ == navdog::NavigationMode::LOCAL_AVOID &&
      previous != navdog::NavigationMode::LOCAL_AVOID)
  {
    local_avoid_enter_stamp_sec_ = ros::Time::now().toSec();
  }
}

void scanTakeoverSyncCallback(const std_msgs::UInt32::ConstPtr& msg)
{
  if (!msg) return;
  expected_takeover_generation_ = msg->data;
  logged_handoff_generation_ = 0;
}

// scanTakeoverReadyCallback：订阅Native SCAN接管就绪generation。
void scanTakeoverReadyCallback(const std_msgs::UInt32::ConstPtr& msg)
{ ready_takeover_generation_ = msg ? msg->data : 0; }

void stairUpActiveCallback(const std_msgs::Bool::ConstPtr& msg)
{ stair_up_active_ = msg && msg->data; }

void externalStopCallback(const std_msgs::Bool::ConstPtr& msg)
{ external_stop_ = msg && msg->data; }

void maxVxLimitCallback(const std_msgs::Float64::ConstPtr& msg)
{
  if (!msg || !std::isfinite(msg->data) || msg->data <= 0.0)
  {
    ROS_WARN_THROTTLE(1.0, "INVALID_TASK_MAX_VX_LIMIT");
    return;
  }
  task_max_vx_ = msg->data;
  task_max_vx_valid_ = true;
}

// timerCallback：固定频率（默认50Hz）的主输出循环，是全局唯一向/cmd_vel发布的入口。
// 整体流程：
// 1. 计算effectiveOwner，并在TRACKING且模式为NONE但刚刚发生状态切换时，
//    在mode_sync_grace_sec宽限内沿用上一个权归属（避免因state/mode两个
//    topic到达顺序不一致导致瞬间零速度抖动）；
// 2. 若检测到权归属变化，从上一帧实际发布给机器人的速度重新初始化限速器
//    （而不是限速器内部可能已过时的值），并根据权归属变化情况重置SCAN接管
//    就绪/前进确认标志，打印CMD_OWNER日志；
// 3. 计算硬停车条件（无人拥有或处于IDLE/PAUSED/SUCCEEDED/FAILED/
//    EMERGENCY_STOP等非运动状态），若成立则立即重置限速器并发布零速度（不经
//    过限速处理），直接返回；
// 4. 否则根据权归属方选择目标指令：
//    - ROUTE：只读取route_cmd，若新鲜则使用，否则告警过时；
//    - SCAN：只读取scan_cmd，需要接管就绪且SCAN指令晚于进入LOCAL_AVOID
//      的时刻且新鲜才采用，
//      并对首次接管后的负向纵向速度做安全阻断（等待确认前进轨迹）；
//      若尚未就绪，在短暂交接宽限内沿用上一帧实际输出，超出宽限则输出零速；
//    - NONE：不处理（保持默认零目标）；
// 5. 若目标有效，根据当前模式对线速度做上限限制；
// 6. 计算dt并调用限速器推进得到实际输出；
// 7. 记录日志、发布最终指令，更新last_output_cmd_与时间戳。
void timerCallback(const ros::TimerEvent&)
{
  const double now_sec = ros::Time::now().toSec();
  CommandOwner owner = effectiveOwner();

  // --- Mode sync grace period ---
  // During TRACKING, if mode is NONE but the state change happened very
  // recently (< mode_sync_grace_sec), keep the previous owner to avoid
  // a brief zero-speed blip caused by state/mode topic arrival order.
  if (nav_state_ == navdog::NavState::TRACKING &&
      owner == CommandOwner::NONE &&
      previous_owner_ != CommandOwner::NONE)
  {
    const double since_state_change = now_sec - nav_state_change_stamp_sec_;
    if (since_state_change < mode_sync_grace_sec)
    {
      owner = previous_owner_;  // Keep previous owner during grace period.
    }
  }

  // --- Detect owner change ---
  if (owner != previous_owner_)
  {
    const CommandOwner old_owner = previous_owner_;
    owner_change_stamp_sec_ = now_sec;

    // Every ownership change starts from the last velocity actually sent to
    // the robot, rather than a possibly stale limiter-internal value.
    slew_limiter_.setCurrent(
        last_output_cmd_.linear.x,
        last_output_cmd_.linear.y,
        last_output_cmd_.angular.z);
    limiter_initialized_ = true;

    if (old_owner == CommandOwner::ROUTE && owner == CommandOwner::SCAN)
      handoff_route_last_cmd_ = last_output_cmd_;

    ROS_INFO("CMD_OWNER prev=%s next=%s state=%s mode=%s",
        ownerName(old_owner),
        ownerName(owner),
        navdog::navStateName(nav_state_),
        navdog::navigationModeName(navigation_mode_));

    previous_owner_ = owner;
  }

  // --- Compute target command ---
  geometry_msgs::Twist target_cmd = zeroCommand();
  bool target_valid = false;
  const char* selection_reason = "NO_TARGET";

  // States that require immediate zero — no slew limiting.
  const bool hard_stop =
      owner == CommandOwner::NONE ||
      nav_state_ == navdog::NavState::IDLE ||
      nav_state_ == navdog::NavState::PAUSED ||
      nav_state_ == navdog::NavState::SUCCEEDED ||
      nav_state_ == navdog::NavState::FAILED ||
      nav_state_ == navdog::NavState::EMERGENCY_STOP;

  if (hard_stop)
  {
    // Immediate zero — reset the limiter so it doesn't try to
    // slew from a stale velocity on the next handoff.
    slew_limiter_.reset();
    last_output_cmd_ = zeroCommand();
    logOutputCommand(last_output_cmd_, target_cmd, owner, false,
        "HARD_STOP_STATE", now_sec);
    publishFinalCommand(zeroCommand(), now_sec);
    last_publish_stamp_sec_ = now_sec;
    return;
  }

  if (external_stop_)
  {
    slew_limiter_.reset();
    last_output_cmd_ = zeroCommand();
    logOutputCommand(last_output_cmd_, target_cmd, owner, false,
        "EXTERNAL_STOP", now_sec);
    publishFinalCommand(zeroCommand(), now_sec);
    last_publish_stamp_sec_ = now_sec;
    return;
  }

  switch (owner)
  {
    case CommandOwner::ROUTE:
      if (isFresh(route_cmd_stamp_sec_, now_sec, route_cmd_timeout_sec))
      {
        target_cmd = latest_route_cmd_;
        target_valid = true;
        selection_reason = "ROUTE_FRESH";
      }
      else
      {
        selection_reason = "ROUTE_STALE";
        ROS_WARN_THROTTLE(1.0, "ROUTE_CMD_STALE");
      }
      break;

    case CommandOwner::SCAN:
    {
      const bool scan_cmd_fresh =
          isFresh(scan_cmd_stamp_sec_, now_sec, scan_cmd_timeout_sec);
      const bool scan_cmd_after_handoff =
          scan_cmd_stamp_sec_ > local_avoid_enter_stamp_sec_ + kEpsilon;
      const bool generation_ready = navdog_runtime::takeoverGenerationReady(
          expected_takeover_generation_, ready_takeover_generation_);
      if (generation_ready && scan_cmd_after_handoff && scan_cmd_fresh)
      {
        target_cmd = latest_scan_cmd_;
        target_valid = true;
        selection_reason = "SCAN_FRESH";
        if (logged_handoff_generation_ != expected_takeover_generation_)
        {
          const double gap_ms =
              std::max(0.0, now_sec - local_avoid_enter_stamp_sec_) * 1000.0;
          ROS_INFO("SCAN_HANDOFF_COMPLETE generation=%u gap_ms=%.0f "
                   "route_last=[%.3f %.3f %.3f] scan_first=[%.3f %.3f %.3f] "
                   "output=[%.3f %.3f %.3f]",
              expected_takeover_generation_, gap_ms,
              handoff_route_last_cmd_.linear.x,
              handoff_route_last_cmd_.linear.y,
              handoff_route_last_cmd_.angular.z,
              target_cmd.linear.x, target_cmd.linear.y, target_cmd.angular.z,
              last_output_cmd_.linear.x,
              last_output_cmd_.linear.y,
              last_output_cmd_.angular.z);
          logged_handoff_generation_ = expected_takeover_generation_;
        }
      }
      else if (!generation_ready)
      {
        const double handoff_age = now_sec - local_avoid_enter_stamp_sec_;
        if (handoff_age < scan_handoff_hold_sec)
        {
          // Preserve the actual last output briefly while prewarmed SCAN
          // publishes its first post-handoff command.  Do not reuse route
          // commands beyond this bounded window.
          target_cmd = last_output_cmd_;
          target_valid = true;
          selection_reason = "SCAN_HANDOFF_HOLD";
        }
        else
        {
          target_cmd = zeroCommand();
          target_valid = true;
          selection_reason = "SCAN_WAITING_READY";
        }
        ROS_WARN_THROTTLE(1.0,
            "SCAN_TAKEOVER_WAITING_READY expected_generation=%u ready_generation=%u handoff_age=%.3f",
            expected_takeover_generation_, ready_takeover_generation_,
            handoff_age);
      }
      else if (!scan_cmd_after_handoff)
      {
        selection_reason = "SCAN_WAITING_COMMAND";
        ROS_WARN_THROTTLE(1.0, "SCAN_CMD_WAITING handoff_age=%.3f",
            now_sec - local_avoid_enter_stamp_sec_);
      }
      else
      {
        selection_reason = "SCAN_STALE";
        ROS_WARN_THROTTLE(1.0, "SCAN_CMD_STALE");
      }
      // If SCAN command not ready yet: target remains zero, and we slew
      // down from the current velocity to zero.
      break;
    }

    case CommandOwner::NONE:
    default:
      break;
  }

  if (target_valid && nav_state_ == navdog::NavState::TRACKING)
  {
    if (navigation_mode_ == navdog::NavigationMode::ROUTE_FOLLOW)
      limitModeLinearSpeed(target_cmd, "ROUTE_FOLLOW");
    else if (navigation_mode_ == navdog::NavigationMode::LOCAL_AVOID)
      limitModeLinearSpeed(target_cmd, "LOCAL_AVOID");
  }

  // --- Apply velocity slew limiting ---
  double dt = (last_publish_stamp_sec_ > 0.0)
      ? (now_sec - last_publish_stamp_sec_)
      : 1.0 / publish_rate_hz;

  double out_vx, out_vy, out_yaw;
  slew_limiter_.update(
      target_cmd.linear.x,
      target_cmd.linear.y,
      target_cmd.angular.z,
      dt, out_vx, out_vy, out_yaw);

  geometry_msgs::Twist output;
  output.linear.x = out_vx;
  output.linear.y = out_vy;
  output.angular.z = out_yaw;

  logOutputCommand(output, target_cmd, owner, target_valid,
      selection_reason, now_sec);
  publishFinalCommand(output, now_sec);
  last_output_cmd_ = output;
  last_publish_stamp_sec_ = now_sec;
}

}  // namespace

// main：cmd_vel_owner_mux节点入口。从参数服务器加载超时/频率/限速参数，
// 校验合法性后注册全部订阅者/发布者并创建定时器，最后进入ros::spin()。
// 本节点是全局唯一允许发布/cmd_vel的节点（多路速度指令的统一出口）。
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_owner_mux");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Load configurable parameters
  private_nh.param("route_cmd_timeout", route_cmd_timeout_sec, 0.30);
  private_nh.param("scan_cmd_timeout", scan_cmd_timeout_sec, 0.30);
  private_nh.param("publish_rate_hz", publish_rate_hz, 50.0);
  private_nh.param("mode_sync_grace_sec", mode_sync_grace_sec, 0.10);
  private_nh.param("scan_handoff_hold_sec", scan_handoff_hold_sec, 0.10);
  private_nh.param("external_stop_topic", external_stop_topic,
      std::string("/navdog/external_stop"));
  private_nh.param("final_cmd_feedback_topic", final_cmd_feedback_topic,
      std::string("/navdog/final_cmd_feedback"));
  private_nh.param("max_vx_limit_topic", max_vx_limit_topic,
      std::string("/navdog/max_vx_limit"));
  private_nh.param("speed_limits/route_follow_linear_mps",
      route_follow_linear_speed_mps, 0.70);
  private_nh.param("speed_limits/local_avoid_linear_mps",
      local_avoid_linear_speed_mps, 0.30);
  private_nh.param("stair_up/linear_speed_mps",
      stair_up_linear_speed_mps, 0.40);

  // Slew limiter params
  navdog_runtime::VelocitySlewLimiter::Config slew_config;
  private_nh.param("handoff_accel_x", slew_config.accel_x, 0.50);
  private_nh.param("handoff_decel_x", slew_config.decel_x, 0.80);
  private_nh.param("handoff_accel_y", slew_config.accel_y, 0.25);
  private_nh.param("handoff_decel_y", slew_config.decel_y, 0.50);
  private_nh.param("handoff_accel_yaw", slew_config.accel_yaw, 0.80);
  private_nh.param("handoff_decel_yaw", slew_config.decel_yaw, 1.20);
  slew_limiter_.setConfig(slew_config);

  if (!std::isfinite(route_cmd_timeout_sec) || route_cmd_timeout_sec <= 0.0 ||
      !std::isfinite(scan_cmd_timeout_sec) || scan_cmd_timeout_sec <= 0.0 ||
      !std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0 ||
      !std::isfinite(scan_handoff_hold_sec) || scan_handoff_hold_sec < 0.0 ||
      !std::isfinite(route_follow_linear_speed_mps) ||
      route_follow_linear_speed_mps <= 0.0 ||
      !std::isfinite(local_avoid_linear_speed_mps) ||
      local_avoid_linear_speed_mps <= 0.0 ||
      !std::isfinite(stair_up_linear_speed_mps) ||
      stair_up_linear_speed_mps <= 0.0)
  {
    ROS_FATAL("cmd_vel_owner_mux: invalid configuration");
    return 1;
  }

  // Subscribers
  ros::Subscriber route_sub = nh.subscribe("/navdog/route_cmd", 10,
      routeCmdCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber scan_sub = nh.subscribe("/navdog/scan_cmd", 10,
      scanCmdCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber mode_sub = nh.subscribe("/navdog/navigation_mode", 10,
      modeCallback);
  ros::Subscriber state_sub = nh.subscribe("/navdog/state", 10,
      stateCallback);
  ros::Subscriber scan_takeover_sync_sub = nh.subscribe(
      "/native_scan/takeover_sync", 10, scanTakeoverSyncCallback);
  ros::Subscriber scan_takeover_ready_sub = nh.subscribe(
      "/native_scan/takeover_ready", 10, scanTakeoverReadyCallback);
  ros::Subscriber external_stop_sub = nh.subscribe(
      external_stop_topic, 10, externalStopCallback);
  ros::Subscriber max_vx_limit_sub = nh.subscribe(
      max_vx_limit_topic, 10, maxVxLimitCallback);
  ros::Subscriber stair_up_active_sub = nh.subscribe(
      "/navdog/stair_up_active", 10, stairUpActiveCallback);

  // Publisher — the ONLY node that publishes to /cmd_vel
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  final_cmd_feedback_pub_ = nh.advertise<geometry_msgs::TwistStamped>(
      final_cmd_feedback_topic, 10);

  // Timer
  ros::Timer timer = nh.createTimer(
      ros::Duration(1.0 / publish_rate_hz), timerCallback);

  ROS_INFO("cmd_vel_owner_mux: ready. route_timeout=%.2f scan_timeout=%.2f "
           "rate=%.1f grace=%.2f scan_hold=%.2f "
           "route_linear=%.2f avoid_linear=%.2f stair_linear=%.2f "
           "handoff accel_x=%.2f decel_x=%.2f accel_yaw=%.2f decel_yaw=%.2f",
      route_cmd_timeout_sec, scan_cmd_timeout_sec, publish_rate_hz,
      mode_sync_grace_sec, scan_handoff_hold_sec,
      route_follow_linear_speed_mps, local_avoid_linear_speed_mps,
      stair_up_linear_speed_mps,
      slew_config.accel_x, slew_config.decel_x,
      slew_config.accel_yaw, slew_config.decel_yaw);

  ros::spin();
  return 0;
}
