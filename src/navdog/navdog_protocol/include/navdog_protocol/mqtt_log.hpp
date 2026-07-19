#pragma once

#include <string>

namespace navdog_protocol
{

/**
 * @brief MQTT 层的无 ROS 日志工具。
 *
 * Mosquitto 回调运行在网络线程，而 Runtime 在 ROS 控制线程消费事件。
 * 此工具只使用 system_clock 生成可读时间，并用内部互斥锁保证一条日志
 * 不会与另一线程的日志交叉；它不参与协议解析或导航决策。
 */
class MqttLog
{
public:
  /** @brief 生成带本地毫秒时间的单行日志，供测试和 stderr 输出共用。 */
  static std::string format(const char* level, const std::string& message);

  /** @brief 原子地将一条 MQTT 事件日志写入 stderr。 */
  static void write(const char* level, const std::string& message);
};

}  // namespace navdog_protocol
