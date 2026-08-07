#pragma once

#include <navdog_core/config.hpp>
#include <navdog_protocol/mqtt_bridge.hpp>
#include <navdog_task/task_types.hpp>

#include <string>

namespace navdog_runtime
{

// ApplicationConfig
// navdog_runtime进程的总配置集合，汇总了navdog_core/navdog_task/navdog_protocol各层的配置，
// 以及runtime自身的IO与最终输出相关参数。由Ros1ConfigLoader从ROS参数服务器加载。
struct ApplicationConfig
{
  navdog::NavdogConfig core{};              // 导航核心层（navdog_core）配置
  navdog_task::TaskConfig task{};           // 任务层（navdog_task）配置
  navdog_protocol::MqttBridgeConfig mqtt{}; // MQTT协议桥接层配置

  // RuntimeIoConfig：Runtime节点自身的输入/输出topic与控制/状态频率配置。
  struct RuntimeIoConfig
  {
    std::string odom_topic{"/quad_0/body_pose"};          // 采用的里程计topic
    std::string final_cmd_topic{"/navdog/route_cmd"};     // 最终输出速度指令topic
    bool odom_twist_in_world_frame{true};                  // 里程计速度是否为世界系
    double control_rate_hz{50.0};                          // 控制循环频率(Hz)
    double status_rate_hz{10.0};                           // 状态上报频率(Hz)
    bool publish_mqtt_status{true};                        // 是否向MQTT发布状态
  } runtime_io;

  // FinalOutputConfig：最终速度指令发布器（FinalCommandPublisher）的超时与发布频率配置。
  struct FinalOutputConfig
  {
    double command_timeout_sec{0.30};   // 输入指令新鲜度超时时长(秒)，超时则输出零速度
    double publish_rate_hz{50.0};       // 最终速度指令发布频率(Hz)
  } final_output;
};

}  // namespace navdog_runtime
