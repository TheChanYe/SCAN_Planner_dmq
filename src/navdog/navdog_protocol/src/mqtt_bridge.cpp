#include "navdog_protocol/mqtt_bridge.hpp"
#include "navdog_protocol/mqtt_codec.hpp"

#include <unistd.h>
#include <cstdio>
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

MqttBridge::MqttBridge(const MqttBridgeConfig& config) : config_(config)
{ mosquitto_lib_init(); }
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
    std::fprintf(stderr, "[navdog_protocol] MQTT bridge disabled\n");
    return true;
  }
  resolved_client_id_ = config_.client_id + "-" +
      std::to_string(static_cast<long long>(::getpid()));
  client_ = mosquitto_new(resolved_client_id_.c_str(), true, this);
  if (!client_) // 如果MQTT客户端创建失败，则打印错误信息并返回失败
  {
    std::fprintf(stderr, "[navdog_protocol] MQTT client creation failed\n");
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
    std::fprintf(stderr,
        "[navdog_protocol] MQTT connect start failed host=%s port=%d error=%s\n",
        config_.host.c_str(), config_.port, mosquitto_strerror(rc));
    mosquitto_destroy(client_);
    client_ = nullptr;
    return false;
  }
  const int loop_rc = mosquitto_loop_start(client_);
  if (loop_rc != MOSQ_ERR_SUCCESS) // 如果MQTT客户端创建失败，则打印错误信息并返回失败
  {
    std::fprintf(stderr,
        "[navdog_protocol] MQTT network loop failed error=%s\n",
        mosquitto_strerror(loop_rc));
    mosquitto_disconnect(client_);
    mosquitto_destroy(client_);
    client_ = nullptr;
    return false;
  }
  started_ = true;
  std::fprintf(stderr,
      "[navdog_protocol] MQTT bridge started client_id=%s host=%s port=%d\n",
      resolved_client_id_.c_str(), config_.host.c_str(), config_.port);
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
    std::fprintf(stderr,
        "[navdog_protocol] MQTT connection rejected code=%d\n", rc);
    return;
  }
  const int task_rc = mosquitto_subscribe(
      client, nullptr, self->config_.task_topic.c_str(), self->config_.qos);
  const int pause_rc = mosquitto_subscribe(
      client, nullptr, self->config_.pause_topic.c_str(), self->config_.qos);
  std::fprintf(stderr,
      "[navdog_protocol] MQTT connected task_topic=%s task_sub=%s "
      "pause_topic=%s pause_sub=%s\n",
      self->config_.task_topic.c_str(), mosquitto_strerror(task_rc),
      self->config_.pause_topic.c_str(), mosquitto_strerror(pause_rc));
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
  std::fprintf(stderr,
      "[navdog_protocol] MQTT disconnected code=%d\n", rc);
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
  bool cancel_first = false;
  const std::string topic(message->topic ? message->topic : "");
  if (topic == self->config_.task_topic) // 如果是任务消息
  {
    bool charging = false;
    std::uint64_t sequence;
    {
      std::lock_guard<std::mutex> lock(self->mutex_);
      sequence = self->next_sequence_++;
    }
    valid = MqttCodec::parseTaskMessage(payload, self->config_.default_route_z,
        self->config_.default_max_vx, sequence, event, charging);
    cancel_first = valid &&
        event.type == navdog_task::NavigationEventType::CANCEL_TASK;
    if (valid)
    {
      std::lock_guard<std::mutex> lock(self->mutex_);
      self->charging_reserved_ = charging;
    }
  }
  else if (topic == self->config_.pause_topic) // 如果是暂停消息
    valid = MqttCodec::parsePauseMessage(payload, event);
  if (valid)
  {
    self->enqueue(event, cancel_first);
    std::fprintf(stderr,
        "[navdog_protocol] MQTT navigation event accepted topic=%s "
        "type=%u sequence=%llu points=%zu\n",
        topic.c_str(), static_cast<unsigned>(event.type),
        static_cast<unsigned long long>(event.task.sequence),
        event.task.points.size());
  }
  else
  {
    std::lock_guard<std::mutex> lock(self->mutex_);
    ++self->protocol_errors_;
    std::fprintf(stderr,
        "[navdog_protocol] MQTT navigation message rejected topic=%s "
        "payload_bytes=%d\n",
        topic.c_str(), message->payloadlen);
  }
}
/**
 * @brief enqueue
 * 将导航事件加入线程安全队列。
 * @param event 导航事件
 * @param cancel_first 是否先取消当前任务
 */
void MqttBridge::enqueue(const navdog_task::NavigationEvent& event,
                         bool cancel_first)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (cancel_first) events_.clear();
  while (events_.size() >= config_.max_queue_size) events_.pop_front();
  events_.push_back(event);
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
 * @brief publishStatus
 * 发布导航状态。
 * @param payload 状态信息
 */
void MqttBridge::publishStatus(const std::string& payload)
{
  if (client_ && started_)
    mosquitto_publish(client_, nullptr, config_.status_topic.c_str(),
        static_cast<int>(payload.size()), payload.data(), config_.qos, false);
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
