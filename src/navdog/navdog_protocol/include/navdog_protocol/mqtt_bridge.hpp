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
  explicit MqttBridge(const MqttBridgeConfig& config);
  ~MqttBridge();
  bool start();
  void stop();
  bool popEvent(navdog_task::NavigationEvent& event);
  void publishStatus(const std::string& payload);
  int consumeProtocolError();
  bool chargingReserved() const;

private:
  static void onConnect(struct mosquitto*, void*, int);
  static void onDisconnect(struct mosquitto*, void*, int);
  static void onMessage(struct mosquitto*, void*, const struct mosquitto_message*);
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
