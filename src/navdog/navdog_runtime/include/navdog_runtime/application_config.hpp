#pragma once

#include <navdog_core/config.hpp>
#include <navdog_protocol/mqtt_bridge.hpp>
#include <navdog_scan_adapter/scan_local_planner_adapter.hpp>
#include <navdog_task/task_types.hpp>

#include <string>

namespace navdog_runtime
{

struct ApplicationConfig
{
  navdog::NavdogConfig core{};
  navdog_task::TaskConfig task{};
  navdog_protocol::MqttBridgeConfig mqtt{};
  navdog_scan_adapter::ScanAdapterConfig scan_adapter{};

  struct RuntimeIoConfig
  {
    std::string odom_topic{"/quad_0/body_pose"};
    std::string final_cmd_topic{"/navdog/final_cmd"};
    bool odom_twist_in_world_frame{true};
    double control_rate_hz{50.0};
    double status_rate_hz{10.0};
  } runtime_io;

  struct FinalOutputConfig
  {
    double command_timeout_sec{0.30};
    double publish_rate_hz{50.0};
  } final_output;
};

}  // namespace navdog_runtime
