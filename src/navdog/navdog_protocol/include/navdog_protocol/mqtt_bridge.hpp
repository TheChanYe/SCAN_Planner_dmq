#pragma once

#include <navdog_protocol/mqtt_codec.hpp>
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
  bool enabled{true};                    // 是否启用MQTT桥接
  std::string host{"127.0.0.1"};         // MQTT Broker地址
  int port{1883};                        // MQTT Broker端口
  int keepalive_sec{30};                 // 心跳保活间隔（秒）
  std::string client_id{"navdog_runtime"};  // MQTT客户端ID
  int qos{1};                            // 订阅/发布使用的QoS等级
  std::string task_topic{"robot/global_planning/info"};   // 任务下发topic
  std::string pause_topic{"robot/local_planning/pause_resume"};  // 暂停/恢复topic
  std::string obstacle_topic{"robot/obstacle/info"};      // 外部障碍输入topic
  std::string status_topic{"robot/local_planning/ctrl"};  // 状态上报topic
  std::string voice_topic{"robot/voice/play"};            // 转弯语音播放topic
  double default_route_z{0.3};           // 路点未带z坐标时的默认高度
  double default_max_vx{0.4};            // 任务未带速度字段时的默认最大速度
  std::size_t max_queue_size{32};        // 事件队列最大长度，超出后丢弃最旧事件
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
  /** @brief 导航正常到达后解除路线锁；ctrl=0 的取消语义保持不变。 */
  void completeActiveTask();
  /** @brief 原样发布既有状态协议 payload，不解释其 JSON 业务含义。 */
  void publishStatus(const std::string& payload);
  /** @brief 原样发布语音提示 payload。 */
  void publishVoice(const std::string& payload);
  /** @brief 获取最新外部障碍信息；仅当收到新报文后返回 true。 */
  bool latestObstacle(ExternalObstacleInfo& obstacle);
  /** @brief 获取并清零协议错误计数（供上层健康监控上报）。 */
  int consumeProtocolError();
  /** @brief 当前是否处于充电保留模式。 */
  bool chargingReserved() const;

private:
  /** @brief 网络线程连接回调：只订阅既有 topic 并记录连接结果。 */
  static void onConnect(struct mosquitto*, void*, int);
  /** @brief 网络线程断开连接回调：记录断开原因。 */
  static void onDisconnect(struct mosquitto*, void*, int);
  /** @brief 网络线程收到消息回调：根据topic分发到对应解码逻辑并入队。 */
  static void onMessage(struct mosquitto*, void*, const struct mosquitto_message*);
  /** @brief 暂停/恢复事件入队；任务启停使用 enqueueTask 保证协议锁。 */
  void enqueue(const navdog_task::NavigationEvent& event);
  /**
   * @brief 实体狗协议的活动任务锁：ctrl=0 或导航正常到达时解锁。
   * 输入输出：event - 待入队的START_TASK事件；charging - 是否为充电保留；
   *       active_sequence - 当前活动任务序号（入队成功时更新）。
   * 返回 false 表示当前任务执行期间的重复路线已被协议层忽略。
   */
  bool enqueueTask(navdog_task::NavigationEvent& event, bool charging,
                   std::uint64_t& active_sequence);
  /** @brief 已持锁前提下将事件推入队列，超过max_queue_size时丢弃最旧事件。 */
  void pushEventLocked(const navdog_task::NavigationEvent& event);

  MqttBridgeConfig config_{};                 // 桥接配置
  struct mosquitto* client_{nullptr};         // Mosquitto客户端实例
  mutable std::mutex mutex_{};                // 保护事件队列/序号/错误计数/充电标记的互斥锁
  std::deque<navdog_task::NavigationEvent> events_{};  // 待消费的事件队列
  std::uint64_t next_sequence_{1};            // 下一个内部任务序号
  std::uint64_t active_sequence_{0};          // 当前活动任务的序号（0表示无）
  int protocol_errors_{0};                    // 累计协议解析错误次数
  ExternalObstacleInfo latest_obstacle_{};     // 最新外部障碍输入
  bool obstacle_updated_{false};               // latestObstacle是否有新值可取
  bool started_{false};                       // 是否已成功启动
  bool charging_reserved_{false};             // 是否处于充电保留模式
  bool route_locked_{false};                  // 路线是否处于锁定状态（防止重复接受）
  std::string resolved_client_id_;            // 实际解析后使用的客户端ID
};

}  // namespace navdog_protocol
