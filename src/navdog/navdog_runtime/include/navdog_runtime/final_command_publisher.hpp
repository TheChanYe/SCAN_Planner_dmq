#pragma once

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

#include <string>

namespace navdog_runtime
{

class FinalCommandPublisher
{
public:
  FinalCommandPublisher(ros::NodeHandle& nh,
      const std::string& input_topic, double command_timeout_sec,
      double publish_rate_hz);

  static bool finite(const geometry_msgs::Twist& command) noexcept;
  static geometry_msgs::Twist validated(
      const geometry_msgs::Twist& command) noexcept;
  static bool fresh(double stamp_sec, double now_sec,
                    double timeout_sec) noexcept;

private:
  void commandCallback(const geometry_msgs::TwistStamped::ConstPtr& message);
  void timerCallback(const ros::TimerEvent&);

  ros::Subscriber input_subscriber_;
  ros::Publisher output_publisher_;
  ros::Timer timer_;
  geometry_msgs::Twist latest_command_{};
  double latest_stamp_sec_{0.0};
  double command_timeout_sec_{0.30};
};

}  // namespace navdog_runtime
