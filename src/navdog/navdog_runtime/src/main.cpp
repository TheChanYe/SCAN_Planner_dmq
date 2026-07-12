#include "navdog_runtime/navdog_runtime_node.hpp"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navdog_runtime");
  navdog_runtime::NavdogRuntimeNode node(ros::NodeHandle{}, ros::NodeHandle{"~"});
  if (!node.initialize()) return 1;
  ros::spin();
  return 0;
}
