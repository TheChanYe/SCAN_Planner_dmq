#pragma once

#include "navdog_runtime/application_config.hpp"
#include <ros/node_handle.h>

namespace navdog_runtime
{

// Ros1ConfigLoader
// 从ROS参数服务器（private NodeHandle）加载navdog_runtime全部配置项到ApplicationConfig。
class Ros1ConfigLoader
{
public:
  // load：从private_nh读取各个参数（未设置时保留默认值），返回完整的ApplicationConfig。
  static ApplicationConfig load(ros::NodeHandle& private_nh);
};

}  // namespace navdog_runtime
