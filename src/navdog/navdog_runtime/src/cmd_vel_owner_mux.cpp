#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <navdog_core/types.hpp>
#include <navdog_runtime/velocity_slew_limiter.hpp>

#include <cmath>
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

// Current state
navdog::NavState nav_state_{navdog::NavState::IDLE};
navdog::NavigationMode navigation_mode_{navdog::NavigationMode::NONE};

geometry_msgs::Twist latest_route_cmd_{};
double route_cmd_stamp_sec_{0.0};

geometry_msgs::Twist latest_scan_cmd_{};
double scan_cmd_stamp_sec_{0.0};

enum class CommandOwner
{
  NONE,
  ROUTE,
  SCAN
};

CommandOwner previous_owner_{CommandOwner::NONE};
double owner_change_stamp_sec_{0.0};
double nav_state_change_stamp_sec_{0.0};
bool scan_takeover_ready{false};
bool scan_takeover_forward_confirmed{false};

// Velocity slew limiter — single instance for smooth handoff.
navdog_runtime::VelocitySlewLimiter slew_limiter_;
geometry_msgs::Twist last_output_cmd_{};
double last_publish_stamp_sec_{0.0};
bool limiter_initialized_{false};

ros::Publisher cmd_vel_pub_;

const char* ownerName(CommandOwner owner)
{
  switch (owner)
  {
    case CommandOwner::ROUTE: return "ROUTE";
    case CommandOwner::SCAN:  return "SCAN";
    case CommandOwner::NONE:
    default:                  return "NONE";
  }
}

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

bool finiteTwist(const geometry_msgs::Twist& cmd)
{
  return std::isfinite(cmd.linear.x) &&
         std::isfinite(cmd.linear.y) &&
         std::isfinite(cmd.linear.z) &&
         std::isfinite(cmd.angular.x) &&
         std::isfinite(cmd.angular.y) &&
         std::isfinite(cmd.angular.z);
}

bool isFresh(double stamp_sec, double now_sec, double timeout_sec)
{
  if (!std::isfinite(stamp_sec) || !std::isfinite(now_sec) ||
      stamp_sec <= 0.0 || timeout_sec <= 0.0)
    return false;

  const double age = now_sec - stamp_sec;
  return age >= 0.0 && age <= timeout_sec;
}

geometry_msgs::Twist zeroCommand()
{
  return geometry_msgs::Twist{};
}

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

void stateCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  const auto previous = nav_state_;
  nav_state_ = static_cast<navdog::NavState>(msg->data);
  if (nav_state_ != previous)
    nav_state_change_stamp_sec_ = ros::Time::now().toSec();
}

void modeCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  navigation_mode_ = static_cast<navdog::NavigationMode>(msg->data);
}

void scanTakeoverReadyCallback(const std_msgs::Bool::ConstPtr& msg)
{ scan_takeover_ready = msg && msg->data; }

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
    {
      scan_takeover_ready = false;
      scan_takeover_forward_confirmed = false;
    }
    else if (owner != CommandOwner::SCAN)
    {
      scan_takeover_ready = false;
      scan_takeover_forward_confirmed = false;
    }

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
    scan_takeover_ready = false;
    scan_takeover_forward_confirmed = false;
    slew_limiter_.reset();
    last_output_cmd_ = zeroCommand();
    cmd_vel_pub_.publish(zeroCommand());
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
      }
      else ROS_WARN_THROTTLE(1.0, "ROUTE_CMD_STALE");
      break;

    case CommandOwner::SCAN:
    {
      const bool scan_cmd_after_handoff =
          scan_cmd_stamp_sec_ > owner_change_stamp_sec_ + kEpsilon;
      const bool scan_cmd_fresh =
          isFresh(scan_cmd_stamp_sec_, now_sec, scan_cmd_timeout_sec);
      if (scan_takeover_ready && scan_cmd_after_handoff && scan_cmd_fresh)
      {
        target_cmd = latest_scan_cmd_;
        target_valid = true;
        if (!scan_takeover_forward_confirmed)
        {
          if (target_cmd.linear.x >= 0.0)
          {
            scan_takeover_forward_confirmed = true;
            ROS_INFO("SCAN_TAKEOVER_FORWARD_CONFIRMED scan_vx=%.3f",
                target_cmd.linear.x);
          }
          else
          {
            const double raw_scan_vx = target_cmd.linear.x;
            // Until a trajectory regenerated from odom proves it commands
            // forward motion, do not pass a longitudinal reverse command.
            target_cmd.linear.x = 0.0;
            ROS_WARN_THROTTLE(1.0,
                "SCAN_TAKEOVER_NEGATIVE_BLOCKED raw_vx=%.3f action=WAIT_FORWARD_TRAJECTORY",
                raw_scan_vx);
          }
        }
      }
      else if (!scan_takeover_ready)
      {
        const double handoff_age = now_sec - owner_change_stamp_sec_;
        if (handoff_age < scan_handoff_hold_sec)
        {
          // Preserve the actual last output briefly while prewarmed SCAN
          // publishes its first post-handoff command.  Do not reuse route
          // commands beyond this bounded window.
          target_cmd = last_output_cmd_;
          target_valid = true;
        }
        else
        {
          target_cmd = zeroCommand();
          target_valid = true;
        }
        ROS_WARN_THROTTLE(1.0, "SCAN_TAKEOVER_WAITING_READY handoff_age=%.3f", handoff_age);
      }
      else if (!scan_cmd_after_handoff)
        ROS_WARN_THROTTLE(1.0, "SCAN_CMD_WAITING handoff_age=%.3f",
            now_sec - owner_change_stamp_sec_);
      else
        ROS_WARN_THROTTLE(1.0, "SCAN_CMD_STALE");
      // If SCAN command not ready yet: target remains zero, and we slew
      // down from the current velocity to zero.
      break;
    }

    case CommandOwner::NONE:
    default:
      break;
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

  cmd_vel_pub_.publish(output);
  last_output_cmd_ = output;
  last_publish_stamp_sec_ = now_sec;
}

}  // namespace

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
      !std::isfinite(scan_handoff_hold_sec) || scan_handoff_hold_sec < 0.0)
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
  ros::Subscriber scan_takeover_ready_sub = nh.subscribe(
      "/native_scan/takeover_ready", 10, scanTakeoverReadyCallback);

  // Publisher — the ONLY node that publishes to /cmd_vel
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Timer
  ros::Timer timer = nh.createTimer(
      ros::Duration(1.0 / publish_rate_hz), timerCallback);

  ROS_INFO("cmd_vel_owner_mux: ready. route_timeout=%.2f scan_timeout=%.2f "
           "rate=%.1f grace=%.2f scan_hold=%.2f "
           "handoff accel_x=%.2f decel_x=%.2f accel_yaw=%.2f decel_yaw=%.2f",
      route_cmd_timeout_sec, scan_cmd_timeout_sec, publish_rate_hz,
      mode_sync_grace_sec, scan_handoff_hold_sec,
      slew_config.accel_x, slew_config.decel_x,
      slew_config.accel_yaw, slew_config.decel_yaw);

  ros::spin();
  return 0;
}
