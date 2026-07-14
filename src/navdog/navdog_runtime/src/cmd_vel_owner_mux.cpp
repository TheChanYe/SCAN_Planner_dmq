#include <cstdint>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

namespace
{

enum class ControlOwner : std::uint8_t
{
  STOP = 0,
  ROUTE = 1,
  SCAN = 2,
};

const char* ownerName(ControlOwner owner)
{
  switch (owner)
  {
    case ControlOwner::ROUTE: return "ROUTE";
    case ControlOwner::SCAN: return "SCAN";
    case ControlOwner::STOP: return "STOP";
  }
  return "STOP";
}

class CmdVelOwnerMux
{
public:
  CmdVelOwnerMux()
      : nh_(), private_nh_("~")
  {
    private_nh_.param("route_cmd_timeout", route_timeout_sec_, 0.30);
    private_nh_.param("scan_cmd_timeout", scan_timeout_sec_, 0.30);
    private_nh_.param("publish_rate_hz", publish_rate_hz_, 50.0);

    if (route_timeout_sec_ <= 0.0 || scan_timeout_sec_ <= 0.0 ||
        publish_rate_hz_ <= 0.0)
    {
      ROS_FATAL("cmd_vel_owner_mux parameters must be positive");
      ros::shutdown();
      return;
    }

    route_subscriber_ = nh_.subscribe(
        "/navdog/route_cmd_vel", 10, &CmdVelOwnerMux::routeCallback, this,
        ros::TransportHints().tcpNoDelay());
    scan_subscriber_ = nh_.subscribe(
        "/native_scan/cmd_vel", 10, &CmdVelOwnerMux::scanCallback, this,
        ros::TransportHints().tcpNoDelay());
    owner_subscriber_ = nh_.subscribe(
        "/navdog/control_owner", 10, &CmdVelOwnerMux::ownerCallback, this);
    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_hz_),
                             &CmdVelOwnerMux::timerCallback, this);
  }

private:
  void routeCallback(const geometry_msgs::Twist::ConstPtr& message)
  {
    route_command_ = *message;
    route_stamp_ = ros::Time::now();
  }

  void scanCallback(const geometry_msgs::Twist::ConstPtr& message)
  {
    scan_command_ = *message;
    scan_stamp_ = ros::Time::now();
  }

  void ownerCallback(const std_msgs::UInt8::ConstPtr& message)
  {
    ControlOwner requested = ControlOwner::STOP;
    if (message->data == static_cast<std::uint8_t>(ControlOwner::ROUTE))
      requested = ControlOwner::ROUTE;
    else if (message->data == static_cast<std::uint8_t>(ControlOwner::SCAN))
      requested = ControlOwner::SCAN;
    else if (message->data != static_cast<std::uint8_t>(ControlOwner::STOP))
      ROS_ERROR("invalid control owner %u; forcing STOP", message->data);

    if (requested == owner_)
      return;

    ROS_WARN("CONTROL_OWNER %s -> %s", ownerName(owner_), ownerName(requested));
    owner_ = requested;

    // Flush the prior owner's last velocity before forwarding the new owner.
    cmd_vel_publisher_.publish(geometry_msgs::Twist{});
    if (requested == ControlOwner::ROUTE)
    {
      route_command_ = geometry_msgs::Twist{};
      route_stamp_ = ros::Time(0);
    }
    else if (requested == ControlOwner::SCAN)
    {
      scan_command_ = geometry_msgs::Twist{};
      scan_stamp_ = ros::Time(0);
    }
  }

  bool commandFresh(const ros::Time& stamp, double timeout_sec,
                    const ros::Time& now) const
  {
    if (stamp.isZero()) return false;
    const double age = (now - stamp).toSec();
    return age >= 0.0 && age <= timeout_sec;
  }

  void timerCallback(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    geometry_msgs::Twist output;

    if (owner_ == ControlOwner::ROUTE &&
        commandFresh(route_stamp_, route_timeout_sec_, now))
    {
      output = route_command_;
    }
    else if (owner_ == ControlOwner::SCAN &&
             commandFresh(scan_stamp_, scan_timeout_sec_, now))
    {
      output = scan_command_;
    }

    // A stale selected source always stops. It never falls back to the other
    // source, even if that source has a fresh command.
    cmd_vel_publisher_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber route_subscriber_;
  ros::Subscriber scan_subscriber_;
  ros::Subscriber owner_subscriber_;
  ros::Publisher cmd_vel_publisher_;
  ros::Timer timer_;

  ControlOwner owner_{ControlOwner::STOP};
  geometry_msgs::Twist route_command_;
  geometry_msgs::Twist scan_command_;
  ros::Time route_stamp_;
  ros::Time scan_stamp_;
  double route_timeout_sec_{0.30};
  double scan_timeout_sec_{0.30};
  double publish_rate_hz_{50.0};
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_owner_mux");
  CmdVelOwnerMux mux;
  ros::spin();
  return 0;
}
