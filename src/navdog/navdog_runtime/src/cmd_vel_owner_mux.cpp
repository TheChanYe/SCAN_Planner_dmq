#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/UInt8.h>
#include <navdog_core/types.hpp>

#include <cmath>
#include <string>

namespace
{

constexpr double kEpsilon = 1e-9;

// Configurable parameters
double route_cmd_timeout_sec = 0.30;
double scan_cmd_timeout_sec = 0.30;
double publish_rate_hz = 50.0;

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
bool zero_frame_pending_{false};
double owner_change_stamp_sec_{0.0};

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

const char* navStateName(navdog::NavState s)
{
  switch (s)
  {
    case navdog::NavState::IDLE:          return "IDLE";
    case navdog::NavState::PLANNING:      return "PLANNING";
    case navdog::NavState::START_ALIGN:   return "START_ALIGN";
    case navdog::NavState::TRACKING:      return "TRACKING";
    case navdog::NavState::PAUSED:        return "PAUSED";
    case navdog::NavState::RECOVERY:      return "RECOVERY";
    case navdog::NavState::GOAL_ALIGN:    return "GOAL_ALIGN";
    case navdog::NavState::SUCCEEDED:     return "SUCCEEDED";
    case navdog::NavState::EMERGENCY_STOP: return "EMERGENCY_STOP";
    case navdog::NavState::FAILED:        return "FAILED";
    default:                              return "UNKNOWN";
  }
}

const char* navModeName(navdog::NavigationMode m)
{
  switch (m)
  {
    case navdog::NavigationMode::ROUTE_FOLLOW: return "ROUTE_FOLLOW";
    case navdog::NavigationMode::LOCAL_AVOID:  return "LOCAL_AVOID";
    case navdog::NavigationMode::NONE:
    default:                                   return "NONE";
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
      {
        return CommandOwner::ROUTE;
      }
      if (navigation_mode_ == navdog::NavigationMode::LOCAL_AVOID)
      {
        return CommandOwner::SCAN;
      }
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
    return;
  latest_route_cmd_ = msg->twist;
  route_cmd_stamp_sec_ = msg->header.stamp.toSec();
}

void scanCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  if (!finiteTwist(*msg))
    return;
  latest_scan_cmd_ = *msg;
  scan_cmd_stamp_sec_ = ros::Time::now().toSec();
}

void stateCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  const auto prev = nav_state_;
  nav_state_ = static_cast<navdog::NavState>(msg->data);
  if (nav_state_ != prev)
  {
    ROS_INFO("NAV_STATE %s -> %s", navStateName(prev), navStateName(nav_state_));
  }
}

void modeCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  const auto prev = navigation_mode_;
  navigation_mode_ = static_cast<navdog::NavigationMode>(msg->data);
  if (navigation_mode_ != prev)
  {
    ROS_INFO("NAV_MODE %s -> %s", navModeName(prev), navModeName(navigation_mode_));
  }
}

void timerCallback(const ros::TimerEvent&)
{
  const double now_sec = ros::Time::now().toSec();
  const CommandOwner owner = effectiveOwner();

  // Detect owner change — emit one zero frame and log.
  if (owner != previous_owner_)
  {
    const auto old_owner = previous_owner_;
    previous_owner_ = owner;
    owner_change_stamp_sec_ = now_sec;
    zero_frame_pending_ = true;

    ROS_INFO("CMD_OWNER %s -> %s state=%u mode=%u",
        ownerName(old_owner),
        ownerName(owner),
        static_cast<unsigned>(nav_state_),
        static_cast<unsigned>(navigation_mode_));
  }

  if (zero_frame_pending_)
  {
    cmd_vel_pub_.publish(zeroCommand());
    zero_frame_pending_ = false;
    return;
  }

  geometry_msgs::Twist output = zeroCommand();

  switch (owner)
  {
    case CommandOwner::ROUTE:
      if (isFresh(route_cmd_stamp_sec_, now_sec, route_cmd_timeout_sec))
      {
        output = latest_route_cmd_;
      }
      break;

    case CommandOwner::SCAN:
      // Only accept scan commands stamped after the ownership change.
      if (scan_cmd_stamp_sec_ > owner_change_stamp_sec_ + kEpsilon &&
          isFresh(scan_cmd_stamp_sec_, now_sec, scan_cmd_timeout_sec))
      {
        output = latest_scan_cmd_;
      }
      break;

    case CommandOwner::NONE:
    default:
      break;
  }

  cmd_vel_pub_.publish(output);
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

  if (!std::isfinite(route_cmd_timeout_sec) || route_cmd_timeout_sec <= 0.0 ||
      !std::isfinite(scan_cmd_timeout_sec) || scan_cmd_timeout_sec <= 0.0 ||
      !std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0)
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

  // Publisher — the ONLY node that publishes to /cmd_vel
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Timer
  ros::Timer timer = nh.createTimer(
      ros::Duration(1.0 / publish_rate_hz), timerCallback);

  ROS_INFO("cmd_vel_owner_mux: ready. route_timeout=%.2f scan_timeout=%.2f rate=%.1f",
      route_cmd_timeout_sec, scan_cmd_timeout_sec, publish_rate_hz);

  ros::spin();
  return 0;
}
