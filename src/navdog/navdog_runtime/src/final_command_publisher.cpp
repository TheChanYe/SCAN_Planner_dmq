#include "navdog_runtime/final_command_publisher.hpp"

#include <cmath>

namespace navdog_runtime
{

// 构造函数：保存超时时长，订阅输入topic（TCP_NODELAY降低延迟），创建/cmd_vel发布者
// 与按rate固定频率触发的定时器。
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

// finite：检查Twist六个分量（线速度xyz、角速度xyz）是否均为有限数。
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

// validated：合法则原样返回，非法（含NaN/Inf）则返回默认构造的零值Twist。
geometry_msgs::Twist FinalCommandPublisher::validated(
    const geometry_msgs::Twist& command) noexcept
{ return finite(command) ? command : geometry_msgs::Twist{}; }

// fresh：判断指令是否仍在时效内。
// 条件：时间戳/当前时间/超时均为有限数且为正数，且age=now-stamp在[0, timeout]区间内
// （age<0表示时间回退，同样视为不新鲜）。
bool FinalCommandPublisher::fresh(
    double stamp, double now, double timeout) noexcept
{
  const double age = now - stamp;
  return std::isfinite(stamp) && std::isfinite(now) &&
      std::isfinite(timeout) && stamp > 0.0 && timeout > 0.0 &&
      age >= 0.0 && age <= timeout;
}

// commandCallback：收到新指令时校验并缓存。若指令非法则时间戳记为0，保证后续
// timerCallback中fresh判断必定失败从而输出零速度。
void FinalCommandPublisher::commandCallback(
    const geometry_msgs::TwistStamped::ConstPtr& message)
{
  latest_command_ = validated(message->twist);
  latest_stamp_sec_ = finite(message->twist)
      ? message->header.stamp.toSec() : 0.0;
}

// timerCallback：按固定频率触发，若最新缓存指令仍处于新鲜期则转发该指令，否则发布默认
// 构造的零速度Twist（安全兼底）。
void FinalCommandPublisher::timerCallback(const ros::TimerEvent&)
{
  const double now_sec = ros::Time::now().toSec();
  output_publisher_.publish(
      fresh(latest_stamp_sec_, now_sec, command_timeout_sec_)
          ? latest_command_ : geometry_msgs::Twist{});
}

}  // namespace navdog_runtime
