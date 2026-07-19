#pragma once

#include <navdog_task/task_types.hpp>
#include <mosquitto.h>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

namespace navdog_protocol
{

struct MqttBridgeConfig
{
  bool enabled{true};
  std::string host{"127.0.0.1"};
  int port{1883};
  int keepalive_sec{30};
  std::string client_id{"navdog_runtime"};
  int qos{1};
  std::string task_topic{"robot/global_planning/info"};
  std::string pause_topic{"robot/local_planning/pause_resume"};
  std::string status_topic{"robot/local_planning/ctrl"};
  double default_route_z{0.3};
  double default_max_vx{0.4};
  std::size_t max_queue_size{32};
};

class MqttBridge
{
public:
  /**
   * @brief Mosquitto/JSON 与 NavigationEvent 间的线程隔离适配器。
   *
   * Mosquitto 回调线程只解析并入队；ROS Runtime 线程通过 popEvent 消费。
   * mutex_ 同时保护事件队列、sequence、协议错误计数和 charging 标记，协议层
   * 绝不修改 NavState 或产生速度命令。
   */
  explicit MqttBridge(const MqttBridgeConfig& config);
  ~MqttBridge();
  /** @brief 创建客户端并启动网络循环；失败不会影响纯 C++ 任务层。 */
  bool start();
  /** @brief 停止网络循环并释放 Mosquitto 客户端，允许析构时重复调用。 */
  void stop();
  /** @brief 由 Runtime 线程弹出最早事件；没有事件返回 false。 */
  bool popEvent(navdog_task::NavigationEvent& event);
  /** @brief 原样发布既有状态协议 payload，不解释其 JSON 业务含义。 */
  void publishStatus(const std::string& payload);
  int consumeProtocolError();
  bool chargingReserved() const;

private:
  /** @brief 网络线程连接回调：只订阅既有 topic 并记录连接结果。 */
  static void onConnect(struct mosquitto*, void*, int);
  static void onDisconnect(struct mosquitto*, void*, int);
  static void onMessage(struct mosquitto*, void*, const struct mosquitto_message*);
  /**
   * @brief 在互斥保护下入队；满队列丢弃最早事件以保留最新控制意图。
   * cancel_first 表示已解析的取消事件应先清空尚未消费的旧意图，而非直接
   * 改变导航状态；状态改变仍由 Runtime 调用 Coordinator 完成。
   */
  void enqueue(const navdog_task::NavigationEvent& event, bool cancel_first);

  MqttBridgeConfig config_{};
  struct mosquitto* client_{nullptr};
  mutable std::mutex mutex_{};
  std::deque<navdog_task::NavigationEvent> events_{};
  std::uint64_t next_sequence_{1};
  int protocol_errors_{0};
  bool started_{false};
  bool charging_reserved_{false};
  std::string resolved_client_id_;
};

}  // namespace navdog_protocol
