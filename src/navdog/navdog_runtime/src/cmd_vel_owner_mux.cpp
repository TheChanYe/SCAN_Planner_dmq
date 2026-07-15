#include "navdog_runtime/final_command_publisher.hpp"
#include "navdog_runtime/ros1_config_loader.hpp"

#include <cmath>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_owner_mux");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  const auto config = navdog_runtime::Ros1ConfigLoader::load(private_nh);
  if (!std::isfinite(config.final_output.command_timeout_sec) ||
      config.final_output.command_timeout_sec <= 0.0 ||
      !std::isfinite(config.final_output.publish_rate_hz) ||
      config.final_output.publish_rate_hz <= 0.0)
  {
    ROS_FATAL("invalid final output configuration");
    return 1;
  }
  navdog_runtime::FinalCommandPublisher publisher(nh,
      config.runtime_io.final_cmd_topic,
      config.final_output.command_timeout_sec,
      config.final_output.publish_rate_hz);
  ros::spin();
  return 0;
}
