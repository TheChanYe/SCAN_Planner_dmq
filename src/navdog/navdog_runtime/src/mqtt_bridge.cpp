#include "navdog_runtime/mqtt_bridge.hpp"

#include <json/json.h>
#include <ros/ros.h>

#include <cmath>
#include <sstream>
#include <unistd.h>

namespace navdog_runtime
{
namespace
{
bool finiteJson(const Json::Value& value, double& output)
{
  if (!value.isNumeric()) return false;
  output = value.asDouble();
  return std::isfinite(output);
}
}

MqttBridge::MqttBridge(const Config& config) : config_(config)
{
  mosquitto_lib_init();
}

MqttBridge::~MqttBridge()
{
  stop();
  mosquitto_lib_cleanup();
}

bool MqttBridge::start()
{
  if (!config_.enabled) return true;
  // A fixed client id lets an old runtime instance disconnect the newly
  // launched one (and vice versa). That was producing a one-second
  // connect/disconnect loop, so a task published after the sender's cancel
  // message was routinely lost. Keep the configured prefix for observability
  // while making this process' MQTT session unique.
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
  if (rc != MOSQ_ERR_SUCCESS || mosquitto_loop_start(client_) != MOSQ_ERR_SUCCESS)
    return false;
  started_ = true;
  ROS_INFO("MQTT client id: %s", resolved_client_id_.c_str());
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
  if (rc == 0)
  {
    mosquitto_subscribe(client, nullptr, self->config_.task_topic.c_str(), self->config_.qos);
    mosquitto_subscribe(client, nullptr, self->config_.pause_topic.c_str(), self->config_.qos);
    ROS_INFO("MQTT connected");
  }
  else ROS_WARN("MQTT connect failed: %d", rc);
}

void MqttBridge::onDisconnect(struct mosquitto*, void*, int rc)
{
  ROS_WARN("MQTT disconnected: %d; local navigation continues", rc);
}

void MqttBridge::onMessage(
    struct mosquitto*, void* data, const struct mosquitto_message* message)
{
  auto* self = static_cast<MqttBridge*>(data);
  if (!message || !message->payload || message->payloadlen <= 0) return;
  const std::string payload(static_cast<const char*>(message->payload),
                            static_cast<std::size_t>(message->payloadlen));
  navdog::NavigationEvent event{};
  bool valid = false;
  bool cancel_first = false;
  const std::string topic(message->topic ? message->topic : "");
  if (topic == self->config_.task_topic)
  {
    bool charging = false;
    std::uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(self->mutex_);
      sequence = self->next_sequence_++;
    }
    valid = parseTaskMessage(payload, self->config_.default_route_z,
        self->config_.default_max_vx, sequence, event, charging);
    cancel_first = valid && event.type == navdog::NavigationEventType::CANCEL_TASK;
    if (valid)
    {
      std::lock_guard<std::mutex> lock(self->mutex_);
      self->charging_reserved_ = charging;
    }
  }
  else if (topic == self->config_.pause_topic)
  {
    valid = parsePauseMessage(payload, event);
  }
  if (valid)
  {
    ROS_INFO("MQTT navigation event received: topic=%s type=%u",
             topic.c_str(), static_cast<unsigned>(event.type));
    self->enqueue(event, cancel_first);
  }
  else
  {
    ROS_WARN("invalid MQTT navigation message on topic=%s", topic.c_str());
    std::lock_guard<std::mutex> lock(self->mutex_);
    ++self->protocol_errors_;
  }
}

void MqttBridge::enqueue(const navdog::NavigationEvent& event, bool cancel_first)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (cancel_first) events_.clear();
  while (events_.size() >= config_.max_queue_size) events_.pop_front();
  events_.push_back(event);
}

bool MqttBridge::popEvent(navdog::NavigationEvent& event)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (events_.empty()) return false;
  event = events_.front();
  events_.pop_front();
  return true;
}

void MqttBridge::publishStatus(const std::string& payload)
{
  if (!client_ || !started_) return;
  mosquitto_publish(client_, nullptr, config_.status_topic.c_str(),
      static_cast<int>(payload.size()), payload.data(), config_.qos, false);
}

int MqttBridge::consumeProtocolError()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const int result = protocol_errors_;
  protocol_errors_ = 0;
  return result;
}

bool MqttBridge::chargingReserved() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return charging_reserved_;
}

bool MqttBridge::parseTaskMessage(
    const std::string& payload, double default_route_z,
    double default_max_vx, std::uint64_t sequence,
    navdog::NavigationEvent& event, bool& charging_reserve)
{
  event = navdog::NavigationEvent{};
  charging_reserve = false;
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string errors;
  std::istringstream stream(payload);
  if (!Json::parseFromStream(builder, stream, &root, &errors) ||
      !root.isObject() || !root["ctrl"].isInt()) return false;
  const int ctrl = root["ctrl"].asInt();
  if (ctrl == 0 || ctrl == 3)
  {
    event.type = navdog::NavigationEventType::CANCEL_TASK;
    charging_reserve = ctrl == 3;
    return true;
  }
  if (ctrl != 1 || sequence == 0) return false;
  const Json::Value& data = root["navigation_data"];
  const Json::Value& points = data["points"];
  if (!data.isObject() || !points.isArray() || points.size() < 2) return false;
  event.type = navdog::NavigationEventType::START_TASK;
  event.task.sequence = sequence;
  event.task.mode = navdog::TaskMode::NORMAL_AVOID;
  event.task.max_vx = default_max_vx;
  if (data.isMember("max_vx") && !finiteJson(data["max_vx"], event.task.max_vx))
    return false;
  if (!(event.task.max_vx > 0.0)) return false;
  for (const auto& item : points)
  {
    navdog::RoutePoint point{};
    if (!item.isObject() || !finiteJson(item["x"], point.x) ||
        !finiteJson(item["y"], point.y)) return false;
    point.z = default_route_z;
    if (item.isMember("z") && !finiteJson(item["z"], point.z)) return false;
    if (item.isMember("yaw"))
    {
      if (!finiteJson(item["yaw"], point.yaw)) return false;
      point.has_yaw = true;
    }
    event.task.points.push_back(point);
  }
  return true;
}

bool MqttBridge::parsePauseMessage(
    const std::string& payload, navdog::NavigationEvent& event)
{
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string errors;
  std::istringstream stream(payload);
  if (!Json::parseFromStream(builder, stream, &root, &errors) ||
      !root["action"].isInt()) return false;
  const int action = root["action"].asInt();
  if (action != 1 && action != 2) return false;
  event = navdog::NavigationEvent{};
  event.type = action == 1 ? navdog::NavigationEventType::PAUSE
                           : navdog::NavigationEventType::RESUME;
  return true;
}

}  // namespace navdog_runtime
