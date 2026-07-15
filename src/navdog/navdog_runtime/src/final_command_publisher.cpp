#include "navdog_runtime/final_command_publisher.hpp"

#include <cmath>

namespace navdog_runtime
{

FinalCommandPublisher::FinalCommandPublisher(ros::NodeHandle& nh,
    const std::string& input_topic, double timeout, double rate)
    : command_timeout_sec_(timeout)
{
  input_subscriber_ = nh.subscribe(input_topic, 10,
      &FinalCommandPublisher::commandCallback, this,
      ros::TransportHints().tcpNoDelay());
  output_publisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  timer_ = nh.createTimer(ros::Duration(1.0 / rate),
      &FinalCommandPublisher::timerCallback, this);
}

bool FinalCommandPublisher::finite(
    const geometry_msgs::Twist& command) noexcept
{
  return std::isfinite(command.linear.x) &&
      std::isfinite(command.linear.y) &&
      std::isfinite(command.linear.z) &&
      std::isfinite(command.angular.x) &&
      std::isfinite(command.angular.y) &&
      std::isfinite(command.angular.z);
}

geometry_msgs::Twist FinalCommandPublisher::validated(
    const geometry_msgs::Twist& command) noexcept
{ return finite(command) ? command : geometry_msgs::Twist{}; }

bool FinalCommandPublisher::fresh(
    double stamp, double now, double timeout) noexcept
{
  const double age = now - stamp;
  return std::isfinite(stamp) && std::isfinite(now) &&
      std::isfinite(timeout) && stamp > 0.0 && timeout > 0.0 &&
      age >= 0.0 && age <= timeout;
}

void FinalCommandPublisher::commandCallback(
    const geometry_msgs::TwistStamped::ConstPtr& message)
{
  latest_command_ = validated(message->twist);
  latest_stamp_sec_ = finite(message->twist)
      ? message->header.stamp.toSec() : 0.0;
}

void FinalCommandPublisher::timerCallback(const ros::TimerEvent&)
{
  const double now_sec = ros::Time::now().toSec();
  output_publisher_.publish(
      fresh(latest_stamp_sec_, now_sec, command_timeout_sec_)
          ? latest_command_ : geometry_msgs::Twist{});
}

}  // namespace navdog_runtime
