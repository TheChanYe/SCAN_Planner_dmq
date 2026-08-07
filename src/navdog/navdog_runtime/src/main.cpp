#include "navdog_runtime/navdog_runtime_node.hpp"

#include <ros/ros.h>

// navdog_runtime进程入口。
// 初始化ROS节点后构造NavdogRuntimeNode（全局/私有两个句柄），调用initialize()完成各子
// 模块初始化。initialize失败则直接退出进程，成功则进入ros::spin()主循环。
int main(int argc, char** argv)
{
  ros::init(argc, argv, "navdog_runtime");
  navdog_runtime::NavdogRuntimeNode node(ros::NodeHandle{}, ros::NodeHandle{"~"});
  if (!node.initialize()) return 1;
  ros::spin();
  return 0;
}
