#include "navdog_protocol/mqtt_bridge.hpp"
#include "navdog_protocol/mqtt_codec.hpp"

#include <unistd.h>
#include <utility>

namespace navdog_protocol
{

MqttBridge::MqttBridge(const MqttBridgeConfig& config) : config_(config)
{ mosquitto_lib_init(); }
MqttBridge::~MqttBridge() { stop(); mosquitto_lib_cleanup(); }

bool MqttBridge::start()
{
  if (!config_.enabled) return true;
  resolved_client_id_ = config_.client_id + "-" +
      std::to_string(static_cast<long long>(::getpid()));
  client_ = mosquitto_new(resolved_client_id_.c_str(), true, this);
  if (!client_) return false;
  mosquitto_connect_callback_set(client_, &MqttBridge::onConnect);
  mosquitto_disconnect_callback_set(client_, &MqttBridge::onDisconnect);
  mosquitto_message_callback_set(client_, &MqttBridge::onMessage);
  mosquitto_reconnect_delay_set(client_, 1, 30, true);
  const int rc = mosquitto_connect_async(client_, config_.host.c_str(),
      config_.port, config_.keepalive_sec);
  if (rc != MOSQ_ERR_SUCCESS ||
      mosquitto_loop_start(client_) != MOSQ_ERR_SUCCESS) return false;
  started_ = true;
  return true;
}

void MqttBridge::stop()
{
  if (!client_) return;
  if (started_) mosquitto_loop_stop(client_, true);
  mosquitto_disconnect(client_);
  mosquitto_destroy(client_);
  client_ = nullptr;
  started_ = false;
}

void MqttBridge::onConnect(struct mosquitto* client, void* data, int rc)
{
  auto* self = static_cast<MqttBridge*>(data);
  if (rc != 0) return;
  mosquitto_subscribe(client, nullptr, self->config_.task_topic.c_str(),
                      self->config_.qos);
  mosquitto_subscribe(client, nullptr, self->config_.pause_topic.c_str(),
                      self->config_.qos);
}
void MqttBridge::onDisconnect(struct mosquitto*, void*, int) {}

void MqttBridge::onMessage(struct mosquitto*, void* data,
    const struct mosquitto_message* message)
{
  auto* self = static_cast<MqttBridge*>(data);
  if (!message || !message->payload || message->payloadlen <= 0) return;
  const std::string payload(static_cast<const char*>(message->payload),
      static_cast<std::size_t>(message->payloadlen));
  navdog_task::NavigationEvent event{};
  bool valid = false;
  bool cancel_first = false;
  const std::string topic(message->topic ? message->topic : "");
  if (topic == self->config_.task_topic)
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
  else if (topic == self->config_.pause_topic)
    valid = MqttCodec::parsePauseMessage(payload, event);
  if (valid) self->enqueue(event, cancel_first);
  else
  {
    std::lock_guard<std::mutex> lock(self->mutex_);
    ++self->protocol_errors_;
  }
}

void MqttBridge::enqueue(const navdog_task::NavigationEvent& event,
                         bool cancel_first)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (cancel_first) events_.clear();
  while (events_.size() >= config_.max_queue_size) events_.pop_front();
  events_.push_back(event);
}
bool MqttBridge::popEvent(navdog_task::NavigationEvent& event)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (events_.empty()) return false;
  event = std::move(events_.front());
  events_.pop_front();
  return true;
}
void MqttBridge::publishStatus(const std::string& payload)
{
  if (client_ && started_)
    mosquitto_publish(client_, nullptr, config_.status_topic.c_str(),
        static_cast<int>(payload.size()), payload.data(), config_.qos, false);
}
int MqttBridge::consumeProtocolError()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const int count = protocol_errors_;
  protocol_errors_ = 0;
  return count;
}
bool MqttBridge::chargingReserved() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return charging_reserved_;
}

}  // namespace navdog_protocol
