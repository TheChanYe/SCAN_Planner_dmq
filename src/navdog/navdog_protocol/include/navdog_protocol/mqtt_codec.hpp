#pragma once

#include <navdog_task/task_types.hpp>

#include <cstdint>
#include <string>

namespace navdog_protocol
{

class MqttCodec
{
public:
  /**
   * @brief 解析任务下发报文（全局规划 topic）为纯C++的NavigationEvent。
   * 输入：payload - JSON文本；default_route_z - 路点未带z时的默认高度；
   *       default_max_vx - 未带速度字段时的默认最大速度；sequence - 由上层分配的内部序号
   *       （不使用MQTT报文自带的序号）。
   * 输出：event - 解析成功时填充的START_TASK事件；charging_reserve - 是否为充电保留模式。
   * 返回：解析是否成功（JSON格式错误或必需字段缺失均返回false）。
   */
  static bool parseTaskMessage(const std::string& payload,
      double default_route_z, double default_max_vx, std::uint64_t sequence,
      navdog_task::NavigationEvent& event, bool& charging_reserve);

  /**
   * @brief 解析暂停/恢复报文（ctrl字段）为PAUSE/RESUME/CANCEL事件。
   * 输入：payload - JSON文本。输出：event - 解析得到的事件。
   * 返回：解析是否成功。
   */
  static bool parsePauseMessage(const std::string& payload,
      navdog_task::NavigationEvent& event);

  /** @brief 将状态码/错误码编码为上报状态的JSON payload。 */
  static std::string encodeStatus(int status, int error);
};

}  // namespace navdog_protocol
