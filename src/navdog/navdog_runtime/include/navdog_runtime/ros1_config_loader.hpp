#pragma once

#include "navdog_runtime/application_config.hpp"
#include <ros/node_handle.h>

namespace navdog_runtime
{

class Ros1ConfigLoader
{
public:
  static ApplicationConfig load(ros::NodeHandle& private_nh);
};

}  // namespace navdog_runtime
