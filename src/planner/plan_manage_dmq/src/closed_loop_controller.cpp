#include <ros/ros.h>
#include "scan_planner/Bspline.h"

namespace
{
void trajectoryCallback(const scan_planner::BsplineConstPtr& message)
{
  ROS_INFO("[closed_loop_controller_dmq] diagnostic trajectory id=%d "
           "control_points=%lu knots=%lu",
           message->traj_id,
           static_cast<unsigned long>(message->pos_pts.size()),
           static_cast<unsigned long>(message->knots.size()));
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "closed_loop_controller_dmq");
  ros::NodeHandle node;
  const ros::Subscriber trajectory_subscriber =
      node.subscribe("planning/bspline", 10, trajectoryCallback);
  (void)trajectory_subscriber;
  ROS_WARN("[closed_loop_controller_dmq] compatibility diagnostic node; "
           "navigation core owns all velocity decisions");
  ros::spin();
  return 0;
}
