#pragma once

#include <mosquitto.h>

#include <functional>
#include <mutex>
#include <string>

namespace dmq_dog
{

struct MqttConfig
{
  bool enabled{true};
  std::string host{"127.0.0.1"};
  int port{1883};
  int keepalive_sec{30};
  std::string client_id{"dmq_dog"};
  int qos{1};
  std::string control_topic{"robot/local_planning/ctrl"};
  std::string odom_topic{"robot/dog/odom"};
  std::string dog_status_topic{"robot/dog/status"};
};

struct DogOdom
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double yaw_rate{0.0};
  bool valid{false};
};

struct DogStatus
{
  int error{0};
  int status{0};
  bool valid{false};
};

class MqttClient
{
public:
  using OdomCallback = std::function<void(const DogOdom&)>;
  using StatusCallback = std::function<void(const DogStatus&)>;

  explicit MqttClient(const MqttConfig& config);
  ~MqttClient();

  bool start();
  void stop();
  bool connected() const;

  void publishControl(int error, int status,
                      double vx, double vy, double yaw_rate);
  void setOdomCallback(OdomCallback callback);
  void setStatusCallback(StatusCallback callback);

private:
  static void onConnect(struct mosquitto* client, void* data, int rc);
  static void onDisconnect(struct mosquitto* client, void* data, int rc);
  static void onMessage(struct mosquitto* client, void* data,
                        const struct mosquitto_message* message);

  void handleOdomMessage(const std::string& payload);
  void handleStatusMessage(const std::string& payload);

  MqttConfig config_{};
  struct mosquitto* client_{nullptr};
  mutable std::mutex mutex_{};
  bool started_{false};
  bool connected_{false};
  std::string resolved_client_id_;
  OdomCallback odom_callback_;
  StatusCallback status_callback_;
};

}  // namespace dmq_dog
