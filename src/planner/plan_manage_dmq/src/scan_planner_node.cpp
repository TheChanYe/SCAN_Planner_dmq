#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage_dmq/scan_replan_fsm.h>

using namespace scan_planner;

// main：scan_planner_dmq_node节点入口。创建SCANReplanFSM实例并初始化（加载
// 参数、创建规划器与注册ROS I/O），短暂休眠1秒等待ROS图连接稳定后进入
// ros::spin()主循环。
int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_planner_dmq_node");
  ros::NodeHandle nh("~");

  SCANReplanFSM scan_replan;

  scan_replan.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
