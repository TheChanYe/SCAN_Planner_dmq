#pragma once

#include <navdog_task/task_types.hpp>

#include <cstdint>
#include <limits>
#include <string>

namespace navdog_protocol
{

struct NavigationMessageMeta
{
  std::uint16_t id{0};
  bool has_id{false};
  std::string map_name{};
  bool has_map_name{false};
};

struct ExternalObstacleInfo
{
  std::uint8_t status{0};
  double distance{std::numeric_limits<double>::infinity()};
  std::uint8_t error{0};
  bool valid{false};
};

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
      navdog_task::NavigationEvent& event, bool& charging_reserve,
      NavigationMessageMeta* meta = nullptr);

  /**
   * @brief 解析暂停/恢复报文（ctrl字段）为PAUSE/RESUME/CANCEL事件。
   * 输入：payload - JSON文本。输出：event - 解析得到的事件。
   * 返回：解析是否成功。
   */
  static bool parsePauseMessage(const std::string& payload,
      navdog_task::NavigationEvent& event);

  /** @brief 解析外部障碍输入 topic 的 JSON payload。 */
  static bool parseObstacleMessage(const std::string& payload,
      ExternalObstacleInfo& obstacle);

  /** @brief 将状态码/错误码编码为上报状态的JSON payload。 */
  static std::string encodeStatus(int status, int error,
      double vx = 0.0, double vy = 0.0, double yaw_rate = 0.0);

  /** @brief 编码转弯语音播放 payload。 */
  static std::string encodeVoiceMessage(const std::string& message);
};

}  // namespace navdog_protocol
