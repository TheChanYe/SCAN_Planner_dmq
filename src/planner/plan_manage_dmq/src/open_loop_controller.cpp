#include <ros/ros.h>
#include "scan_planner/Bspline.h"

namespace
{
// trajectoryCallback：仅用于诊断的B样条轨迹订阅回调，打印轨迹ID、控制点数与
// 节点数日志，不产生任何控制输出。
void trajectoryCallback(const scan_planner::BsplineConstPtr& message)
{
  ROS_INFO("[open_loop_controller_dmq] diagnostic trajectory id=%d "
           "control_points=%lu knots=%lu",
           static_cast<long>(message->traj_id),
           static_cast<unsigned long>(message->pos_pts.size()),
           static_cast<unsigned long>(message->knots.size()));
}
}

// main：open_loop_controller_dmq节点入口。这是一个仅用于兼容性诊断的节点，
// 只订阅规划轨迹并打印日志，不发布任何里程计或速度指令。
int main(int argc, char** argv)
{
  ros::init(argc, argv, "open_loop_controller_dmq");
  ros::NodeHandle node;
  const ros::Subscriber trajectory_subscriber =
      node.subscribe("planning/bspline", 10, trajectoryCallback);
  (void)trajectory_subscriber;
  ROS_WARN("[open_loop_controller_dmq] compatibility diagnostic node; "
           "does not publish odometry or velocity");
  ros::spin();
  return 0;
}
