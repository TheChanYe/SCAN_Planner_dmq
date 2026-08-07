#pragma once

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

#include <string>

namespace navdog_runtime
{

// FinalCommandPublisher
// 订阅上游的带时间戳速度指令（TwistStamped），按固定频率重新发布为普通Twist到
// /cmd_vel。若输入指令过时或非法，则输出零速度作为安全兼底。
class FinalCommandPublisher
{
public:
  // 构造函数：订阅input_topic，创建/cmd_vel发布者与定时器。
  // 输入：nh - ROS节点句柄；input_topic - 输入topic名；
  //       command_timeout_sec - 指令新鲜度超时时长；publish_rate_hz - 发布频率。
  FinalCommandPublisher(ros::NodeHandle& nh,
      const std::string& input_topic, double command_timeout_sec,
      double publish_rate_hz);

  // finite：检查Twist指令的六个分量是否均为有限数。
  static bool finite(const geometry_msgs::Twist& command) noexcept;
  // validated：若指令合法（有限数）则原样返回，否则返回零值Twist。
  static geometry_msgs::Twist validated(
      const geometry_msgs::Twist& command) noexcept;
  // fresh：判断指令是否仍在时效内（时间戳/当前时间/超时均为有限数且时间差
  // 在[0, timeout_sec]范围内）。
  static bool fresh(double stamp_sec, double now_sec,
                    double timeout_sec) noexcept;

private:
  // commandCallback：收到新指令时，校验并缓存最新指令与其时间戳（非法指令时时间戳
  // 记为0，确保后续fresh判断失败从而输出零速度）。
  void commandCallback(const geometry_msgs::TwistStamped::ConstPtr& message);
  // timerCallback：按固定频率触发，若最新缓存指令仍新鲜则转发，否则发布零速度。
  void timerCallback(const ros::TimerEvent&);

  ros::Subscriber input_subscriber_;   // 输入指令订阅者
  ros::Publisher output_publisher_;    // /cmd_vel发布者
  ros::Timer timer_;                   // 固定频率发布定时器
  geometry_msgs::Twist latest_command_{};  // 缓存的最新有效指令
  double latest_stamp_sec_{0.0};           // 最新指令的时间戳
  double command_timeout_sec_{0.30};       // 指令新鲜度超时时长
};

}  // namespace navdog_runtime
