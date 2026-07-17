#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/UInt8.h>

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
geometry_msgs::Twist latest_route_cmd_{};
double route_cmd_stamp_sec_{0.0};

geometry_msgs::Twist latest_scan_cmd_{};
double scan_cmd_stamp_sec_{0.0};

std::uint8_t navigation_mode_{0};  // 0=NONE, 1=ROUTE_FOLLOW, 2=LOCAL_AVOID
double mode_change_stamp_sec_{0.0};

ros::Publisher cmd_vel_pub_;

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

void modeCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  const std::uint8_t previous = navigation_mode_;
  navigation_mode_ = msg->data;
  if (navigation_mode_ != previous)
  {
    mode_change_stamp_sec_ = ros::Time::now().toSec();
    const char* prev_name = previous == 1 ? "ROUTE_FOLLOW" :
                            (previous == 2 ? "LOCAL_AVOID" : "NONE");
    const char* new_name = navigation_mode_ == 1 ? "ROUTE_FOLLOW" :
                           (navigation_mode_ == 2 ? "LOCAL_AVOID" : "NONE");
    ROS_INFO("CMD_OWNER %s -> %s", prev_name, new_name);
  }
}

void timerCallback(const ros::TimerEvent&)
{
  const double now_sec = ros::Time::now().toSec();
  geometry_msgs::Twist output = zeroCommand();

  switch (navigation_mode_)
  {
    case 1:  // ROUTE_FOLLOW
    {
      if (isFresh(route_cmd_stamp_sec_, now_sec, route_cmd_timeout_sec))
      {
        output = latest_route_cmd_;
      }
      break;
    }

    case 2:  // LOCAL_AVOID
    {
      // Only accept scan commands stamped after the mode transition.
      if (scan_cmd_stamp_sec_ > mode_change_stamp_sec_ + kEpsilon &&
          isFresh(scan_cmd_stamp_sec_, now_sec, scan_cmd_timeout_sec))
      {
        output = latest_scan_cmd_;
      }
      break;
    }

    default:  // NONE or unknown
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
