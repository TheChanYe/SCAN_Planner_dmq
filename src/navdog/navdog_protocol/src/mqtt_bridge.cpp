#include "navdog_protocol/mqtt_bridge.hpp"
#include "navdog_protocol/mqtt_codec.hpp"
#include "navdog_protocol/mqtt_log.hpp"

#include <unistd.h>
#include <utility>
/**
 * 主要职责：
 * 连接Mosquitto
 * 订阅任务Topic
 * 订阅暂停Topic
 * 解析消息
 * 把NavigationEvent放进线程安全队列
 * 发布状态
 */
namespace navdog_protocol
{

/**
 * @brief 构造函数
 * 保存桥接配置并初始化Mosquitto库（进程级全局初始化，与析构函数中的cleanup配套）。
 */
MqttBridge::MqttBridge(const MqttBridgeConfig& config) : config_(config)
{ mosquitto_lib_init(); }
/**
 * @brief 析构函数
 * 先stop()停止网络循环并释放客户端，再清理Mosquitto库全局资源。
 */
MqttBridge::~MqttBridge() { stop(); mosquitto_lib_cleanup(); }
/**
 * @brief start
 * 启动MQTT桥接。
 * @return 是否成功
 */
bool MqttBridge::start()
{
  if (!config_.enabled) // 如果MQTT桥接未启用，则直接返回成功
  {
    MqttLog::write("INFO", "event=MQTT_DISABLED");
    return true;
  }
  resolved_client_id_ = config_.client_id + "-" +
      std::to_string(static_cast<long long>(::getpid()));
  client_ = mosquitto_new(resolved_client_id_.c_str(), true, this);
  if (!client_) // 如果MQTT客户端创建失败，则打印错误信息并返回失败
  {
    MqttLog::write("ERROR", "event=MQTT_CLIENT_CREATE_FAILED");
    return false;
  }
  mosquitto_connect_callback_set(client_, &MqttBridge::onConnect);
  mosquitto_disconnect_callback_set(client_, &MqttBridge::onDisconnect);
  mosquitto_message_callback_set(client_, &MqttBridge::onMessage);
  mosquitto_reconnect_delay_set(client_, 1, 30, true);
  const int rc = mosquitto_connect_async(client_, config_.host.c_str(),
      config_.port, config_.keepalive_sec);
  if (rc != MOSQ_ERR_SUCCESS)  // 如果MQTT客户端创建失败，则打印错误信息并返回失败
  {
    MqttLog::write("ERROR", "event=MQTT_CONNECT_START_FAILED host=" +
        config_.host + " port=" + std::to_string(config_.port) +
        " error=" + mosquitto_strerror(rc));
    mosquitto_destroy(client_);
    client_ = nullptr;
    return false;
  }
  const int loop_rc = mosquitto_loop_start(client_);
  if (loop_rc != MOSQ_ERR_SUCCESS) // 如果MQTT客户端创建失败，则打印错误信息并返回失败
  {
    MqttLog::write("ERROR", "event=MQTT_LOOP_START_FAILED error=" +
        std::string(mosquitto_strerror(loop_rc)));
    mosquitto_disconnect(client_);
    mosquitto_destroy(client_);
    client_ = nullptr;
    return false;
  }
  started_ = true;
  MqttLog::write("INFO", "event=MQTT_STARTED client_id=" + resolved_client_id_ +
      " host=" + config_.host + " port=" + std::to_string(config_.port));
  return true;
}
/**
 * @brief stop
 * 停止MQTT桥接。
 */
void MqttBridge::stop()
{
  if (!client_) return;
  if (started_) mosquitto_loop_stop(client_, true);
  mosquitto_disconnect(client_);
  mosquitto_destroy(client_);
  client_ = nullptr;
  started_ = false;
}
/**
 * @brief onConnect
 * MQTT连接回调函数。
 * @param client MQTT客户端实例
 * @param data 用户数据指针
 * @param rc 连接结果代码
 */
void MqttBridge::onConnect(struct mosquitto* client, void* data, int rc)
{
  auto* self = static_cast<MqttBridge*>(data);
  if (rc != 0)
  {
    MqttLog::write("WARN", "event=MQTT_DISCONNECTED code=" + std::to_string(rc));
    return;
  }
  const int task_rc = mosquitto_subscribe(
      client, nullptr, self->config_.task_topic.c_str(), self->config_.qos);
  const int pause_rc = mosquitto_subscribe(
      client, nullptr, self->config_.pause_topic.c_str(), self->config_.qos);
  MqttLog::write("INFO", "event=MQTT_CONNECTED task_topic=" +
      self->config_.task_topic + " task_sub=" + mosquitto_strerror(task_rc) +
      " pause_topic=" + self->config_.pause_topic + " pause_sub=" +
      mosquitto_strerror(pause_rc));
}
/**
 * @brief onDisconnect
 * MQTT断开连接回调函数。
 * @param client MQTT客户端实例
 * @param data 用户数据指针
 * @param rc 断开连接结果代码
 */
void MqttBridge::onDisconnect(struct mosquitto*, void*, int rc)
{
  MqttLog::write("WARN", "event=MQTT_DISCONNECTED code=" + std::to_string(rc));
}
/**
 * @brief onMessage
 * MQTT消息回调函数。
 * @param client MQTT客户端实例
 * @param data 用户数据指针
 * @param message MQTT消息结构体
 * 作用是解析MQTT消息，并将导航事件加入线程安全队列。
 */
void MqttBridge::onMessage(struct mosquitto*, void* data,
    const struct mosquitto_message* message)
{
  // 获取MqttBridge实例
  auto* self = static_cast<MqttBridge*>(data);
  if (!message || !message->payload || message->payloadlen <= 0) return;
  const std::string payload(static_cast<const char*>(message->payload),
      static_cast<std::size_t>(message->payloadlen));
  navdog_task::NavigationEvent event{};
  bool valid = false;
  bool task_message = false;
  bool charging = false;
  const std::string topic(message->topic ? message->topic : "");
  if (topic == self->config_.task_topic) // 如果是任务消息
  {
    task_message = true;
    std::uint64_t sequence;
    {
      std::lock_guard<std::mutex> lock(self->mutex_);
      sequence = self->next_sequence_;
    }
    valid = MqttCodec::parseTaskMessage(payload, self->config_.default_route_z,
        self->config_.default_max_vx, sequence, event, charging);
  }
  else if (topic == self->config_.pause_topic) // 如果是暂停消息
    valid = MqttCodec::parsePauseMessage(payload, event);
  if (valid)
  {
    std::uint64_t active_sequence = 0;
    const bool admitted = !task_message ||
        self->enqueueTask(event, charging, active_sequence);
    if (!admitted)
    {
      MqttLog::write("INFO", "event=MQTT_ROUTE_IGNORED ctrl=1 "
          "reason=WAIT_CTRL_0 active_sequence=" +
          std::to_string(active_sequence));
      return;
    }
    if (!task_message) self->enqueue(event);
    if (task_message &&
        event.type == navdog_task::NavigationEventType::CANCEL_TASK &&
        !charging)
    {
      MqttLog::write("INFO",
          "event=MQTT_ROUTE_CLEARED ctrl=0 action=CANCEL_AND_UNLOCK");
    }
    MqttLog::write("INFO", "event=MQTT_EVENT_ACCEPTED topic=" + topic +
        " type=" + std::to_string(static_cast<unsigned>(event.type)) +
        " sequence=" + std::to_string(event.task.sequence) +
        " points=" + std::to_string(event.task.points.size()));
  }
  else
  {
    std::lock_guard<std::mutex> lock(self->mutex_);
    ++self->protocol_errors_;
    MqttLog::write("WARN", "event=MQTT_MESSAGE_REJECTED topic=" + topic +
        " payload_bytes=" + std::to_string(message->payloadlen) +
        " error_category=PARSE_OR_TOPIC");
  }
}
/**
 * @brief enqueue
 * 将导航事件加入线程安全队列。
 * @param event 导航事件
 */
/**
 * @brief enqueue
 * 将非任务类事件（暂停/恢复）直接加锁后入队，不涉及协议锁判断。
 * @param event 待入队的导航事件
 */
void MqttBridge::enqueue(const navdog_task::NavigationEvent& event)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pushEventLocked(event);
}

/**
 * @brief enqueueTask
 * 任务类事件（START_TASK/CANCEL_TASK）的入队入口，负责实体狗协议的“活动任务锁”语义。
 * 步骤：
 *   1.START_TASK：若已有路线被锁定（route_locked_），拒绝入队并返回当前活动序号；
 *      否则分配新序号、锁定路线、清除充电保留标记；
 *   2.CANCEL_TASK：清空事件队列、释放活动序号与路线锁，并根据charging参数设置充电保留标记；
 *   3.无论哪种情况最后都会入队并返回当前活动序号。
 * 返回 false 表示当前任务执行期间的重复路线已被协议层忽略。
 */
bool MqttBridge::enqueueTask(navdog_task::NavigationEvent& event,
                             bool charging,
                             std::uint64_t& active_sequence)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (event.type == navdog_task::NavigationEventType::START_TASK)
  {
    if (route_locked_)
    {
      active_sequence = active_sequence_;
      return false;
    }
    event.task.sequence = next_sequence_++;
    active_sequence_ = event.task.sequence;
    route_locked_ = true;
    charging_reserved_ = false;
  }
  else if (event.type == navdog_task::NavigationEventType::CANCEL_TASK)
  {
    events_.clear();
    active_sequence_ = 0;
    route_locked_ = false;
    charging_reserved_ = charging;
  }
  pushEventLocked(event);
  active_sequence = active_sequence_;
  return true;
}

/**
 * @brief pushEventLocked
 * 已持锁前提下将事件推入队列尾部；若队列已满（达到max_queue_size）则先丢弃最旧事件，
 * 并记录一条队列溢出警告日志。
 */
void MqttBridge::pushEventLocked(
    const navdog_task::NavigationEvent& event)
{
  bool dropped_oldest = false;
  while (events_.size() >= config_.max_queue_size) { events_.pop_front(); dropped_oldest = true; }
  events_.push_back(event);
  if (dropped_oldest)
    MqttLog::write("WARN", "event=MQTT_QUEUE_OVERFLOW policy=DROP_OLDEST");
}
/**
 * @brief popEvent
 * 获取导航事件。
 * @param event 存储导航事件的对象
 * @return 是否成功获取导航事件
 */
bool MqttBridge::popEvent(navdog_task::NavigationEvent& event)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (events_.empty()) return false;
  event = std::move(events_.front());
  events_.pop_front();
  return true;
}

/**
 * @brief completeActiveTask
 * 导航正常到达后由上层调用，解除实体狗协议的活动任务锁，使后续新路线可被接受。
 * 若当前并无锁定路线则直接返回（无副作用）。
 */
void MqttBridge::completeActiveTask()
{
  std::uint64_t sequence = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!route_locked_) return;
    sequence = active_sequence_;
    active_sequence_ = 0;
    route_locked_ = false;
    charging_reserved_ = false;
  }
  MqttLog::write("INFO", "event=MQTT_TASK_UNLOCK sequence=" +
      std::to_string(sequence) + " reason=NAV_SUCCEEDED");
}
/**
 * @brief publishStatus
 * 发布导航状态。
 * @param payload 状态信息
 */
void MqttBridge::publishStatus(const std::string& payload)
{
  if (client_ && started_)
  {
    const int rc = mosquitto_publish(client_, nullptr, config_.status_topic.c_str(),
        static_cast<int>(payload.size()), payload.data(), config_.qos, false);
    if (rc != MOSQ_ERR_SUCCESS)
      MqttLog::write("WARN", "event=MQTT_PUBLISH_FAILED topic=" +
          config_.status_topic + " error=" + mosquitto_strerror(rc));
  }
}
/**
 * @brief consumeProtocolError
 * 消费导航协议错误。
 * @return 导航协议错误数量
 */
int MqttBridge::consumeProtocolError()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const int count = protocol_errors_;
  protocol_errors_ = 0;
  return count;
}
/**
 * @brief chargingReserved
 * 获取充电任务保留状态。
 * @return 充电任务保留状态
 */
bool MqttBridge::chargingReserved() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return charging_reserved_;
}

}  // namespace navdog_protocol
