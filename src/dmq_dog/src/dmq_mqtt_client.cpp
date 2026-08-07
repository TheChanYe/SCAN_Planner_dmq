#include "dmq_dog/dmq_mqtt_client.hpp"

#include <jsoncpp/json/json.h>
#include <ros/console.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <unistd.h>

namespace dmq_dog
{

namespace
{
// clampUint8：将整数限制到 [0,255] 范围，用于 error/status 字段均为 uint8 语义的场景。
int clampUint8(const int value)
{
  return std::max(0, std::min(255, value));
}
}  // namespace

// 构造函数：保存配置并初始化 mosquitto 库（进程级初始化，必须在创建客户端前调用）。
MqttClient::MqttClient(const MqttConfig& config) : config_(config)
{
  mosquitto_lib_init();
}

// 析构函数：先停止客户端释放连接资源，再清理 mosquitto 库全局资源。
MqttClient::~MqttClient()
{
  stop();
  mosquitto_lib_cleanup();
}

// start：启动 MQTT 客户端。步骤：
//   1. 若配置禁用，直接返回 true（不真正连接）；
//   2. 拼接 client_id+pid 作为实际客户端ID（避免多实例重名冲突），创建 mosquitto 客户端；
//   3. 注册连接/断开/消息三个回调，设置自动重连退避策略(1~30秒指数退避)；
//   4. 开始异步连接，失败则销毁客户端并返回 false；
//   5. 启动 mosquitto 后台网络循环线程，失败则断开并销毁客户端后返回 false。
// 输出：是否启动成功。
bool MqttClient::start()
{
  if (!config_.enabled)
  {
    ROS_WARN("dmq_dog: MQTT disabled by config.");
    return true;
  }

  resolved_client_id_ =
      config_.client_id + "-" + std::to_string(static_cast<long long>(::getpid()));
  client_ = mosquitto_new(resolved_client_id_.c_str(), true, this);
  if (!client_)
  {
    ROS_ERROR("dmq_dog: failed to create MQTT client.");
    return false;
  }

  mosquitto_connect_callback_set(client_, &MqttClient::onConnect);
  mosquitto_disconnect_callback_set(client_, &MqttClient::onDisconnect);
  mosquitto_message_callback_set(client_, &MqttClient::onMessage);
  mosquitto_reconnect_delay_set(client_, 1, 30, true);

  const int rc = mosquitto_connect_async(
      client_, config_.host.c_str(), config_.port, config_.keepalive_sec);
  if (rc != MOSQ_ERR_SUCCESS)
  {
    ROS_ERROR_STREAM("dmq_dog: MQTT connect start failed: "
                     << mosquitto_strerror(rc));
    mosquitto_destroy(client_);
    client_ = nullptr;
    return false;
  }

  const int loop_rc = mosquitto_loop_start(client_);
  if (loop_rc != MOSQ_ERR_SUCCESS)
  {
    ROS_ERROR_STREAM("dmq_dog: MQTT loop start failed: "
                     << mosquitto_strerror(loop_rc));
    mosquitto_disconnect(client_);
    mosquitto_destroy(client_);
    client_ = nullptr;
    return false;
  }

  started_ = true;
  ROS_INFO_STREAM("dmq_dog: MQTT started, host=" << config_.host
                  << " port=" << config_.port
                  << " control_topic=" << config_.control_topic);
  return true;
}

// stop：停止后台网络循环、主动断开连接并销毁客户端实例，并重置启动/连接状态标志。
void MqttClient::stop()
{
  if (!client_) return;
  if (started_) mosquitto_loop_stop(client_, true);
  mosquitto_disconnect(client_);
  mosquitto_destroy(client_);
  client_ = nullptr;
  started_ = false;
  connected_ = false;
}

// connected：线程安全地返回当前是否已连接到 Broker。
bool MqttClient::connected() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return connected_;
}

// publishControl：将控制指令打包成 JSON 并发布到 control_topic。
// 步骤：1.未启用/未启动时直接返回；2.按照真机控制器已验证的紧凑JSON格式拼接字符串
// （error/status限幅到[0,255]，速度保留3位小数）；3.发布失败时限频打印警告日志。
void MqttClient::publishControl(const int error, const int status,
                                const double vx, const double vy,
                                const double yaw_rate)
{
  if (!config_.enabled || !client_ || !started_) return;

  // Match the deployed Agibot publisher exactly. The physical controller is
  // closed-source, so keep its proven compact JSON and numeric formatting.
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "{\"error\":" << clampUint8(error)
         << ",\"status\":" << clampUint8(status)
         << ",\"velocity\":{\"vx\":" << vx
         << ",\"vy\":" << vy
         << ",\"yaw_rate\":" << yaw_rate << "}}";
  const std::string payload = stream.str();

  const int rc = mosquitto_publish(
      client_, nullptr, config_.control_topic.c_str(),
      static_cast<int>(payload.size()), payload.data(), config_.qos, false);
  if (rc != MOSQ_ERR_SUCCESS)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "dmq_dog: MQTT publish failed: "
                                      << mosquitto_strerror(rc));
  }
}

// setOdomCallback：线程安全地设置里程计数据回调。
void MqttClient::setOdomCallback(OdomCallback callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  odom_callback_ = std::move(callback);
}

// setStatusCallback：线程安全地设置狗体状态回调。
void MqttClient::setStatusCallback(StatusCallback callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  status_callback_ = std::move(callback);
}

// onConnect：mosquitto 连接完成静态回调。
// 步骤：1.通过 data 拿到 this 指针，更新 connected_ 状态；2.若连接失败(rc!=0)直接返回；
// 3.连接成功后自动订阅 odom_topic 与 dog_status_topic（topic为空则跳过）并打印日志。
void MqttClient::onConnect(struct mosquitto* client, void* data, const int rc)
{
  auto* self = static_cast<MqttClient*>(data);
  if (!self) return;

  {
    std::lock_guard<std::mutex> lock(self->mutex_);
    self->connected_ = rc == 0;
  }

  if (rc != 0)
  {
    ROS_WARN_STREAM("dmq_dog: MQTT disconnected during connect, code=" << rc);
    return;
  }

  int odom_rc = MOSQ_ERR_SUCCESS;
  int status_rc = MOSQ_ERR_SUCCESS;
  if (!self->config_.odom_topic.empty())
  {
    odom_rc = mosquitto_subscribe(
        client, nullptr, self->config_.odom_topic.c_str(), self->config_.qos);
  }
  if (!self->config_.dog_status_topic.empty())
  {
    status_rc = mosquitto_subscribe(client, nullptr,
        self->config_.dog_status_topic.c_str(), self->config_.qos);
  }
  ROS_INFO_STREAM("dmq_dog: MQTT connected, odom_topic="
                  << self->config_.odom_topic << " odom_sub="
                  << mosquitto_strerror(odom_rc) << " dog_status_topic="
                  << self->config_.dog_status_topic << " status_sub="
                  << mosquitto_strerror(status_rc));
}

// onDisconnect：mosquitto 断开连接回调，更新 connected_ 为 false 并打印警告日志。
void MqttClient::onDisconnect(struct mosquitto*, void* data, const int rc)
{
  auto* self = static_cast<MqttClient*>(data);
  if (self)
  {
    std::lock_guard<std::mutex> lock(self->mutex_);
    self->connected_ = false;
  }
  ROS_WARN_STREAM("dmq_dog: MQTT disconnected, code=" << rc);
}

// onMessage：mosquitto 收到订阅消息时的回调。
// 校验有效性后根据 topic 匹配结果分发到 handleOdomMessage 或 handleStatusMessage。
void MqttClient::onMessage(struct mosquitto*, void* data,
                           const struct mosquitto_message* message)
{
  auto* self = static_cast<MqttClient*>(data);
  if (!self || !message || !message->payload || message->payloadlen <= 0) return;

  const std::string topic(message->topic ? message->topic : "");
  const std::string payload(static_cast<const char*>(message->payload),
                            static_cast<std::size_t>(message->payloadlen));

  if (topic == self->config_.odom_topic)
    self->handleOdomMessage(payload);
  else if (topic == self->config_.dog_status_topic)
    self->handleStatusMessage(payload);
}

// handleOdomMessage：解析里程计 JSON 负载。
// 步骤：1.JSON解析失败/非对象则限频打印警告并返回；2.逐字段提取（缺失字段默认为0）；
// 3.校验所有字段均为有限数才置 valid=true，无效则直接丢弃；4.在锁内拷贝回调后释放锁再调用，
// 避免在持锁状态下调用上层代码导致潜在死锁。
void MqttClient::handleOdomMessage(const std::string& payload)
{
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string error;
  std::istringstream stream(payload);
  if (!Json::parseFromStream(builder, stream, &root, &error) || !root.isObject())
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "dmq_dog: invalid odom JSON: " << error);
    return;
  }

  DogOdom odom;
  odom.x = root.get("x", 0.0).asDouble();
  odom.y = root.get("y", 0.0).asDouble();
  odom.z = root.get("z", 0.0).asDouble();
  odom.yaw = root.get("yaw", 0.0).asDouble();
  odom.vx = root.get("vx", 0.0).asDouble();
  odom.vy = root.get("vy", 0.0).asDouble();
  odom.yaw_rate = root.get("yaw_rate", 0.0).asDouble();
  odom.valid = std::isfinite(odom.x) && std::isfinite(odom.y) &&
      std::isfinite(odom.z) && std::isfinite(odom.yaw) &&
      std::isfinite(odom.vx) && std::isfinite(odom.vy) &&
      std::isfinite(odom.yaw_rate);
  if (!odom.valid) return;

  OdomCallback callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    callback = odom_callback_;
  }
  if (callback) callback(odom);
}

// handleStatusMessage：解析狗体状态 JSON 负载。
// 步骤与 handleOdomMessage 类似：JSON解析失败则丢弃，提取error/status字段，
// 校验error/status均为非负数才视为有效，有效时在锁外回调上层。
void MqttClient::handleStatusMessage(const std::string& payload)
{
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string error;
  std::istringstream stream(payload);
  if (!Json::parseFromStream(builder, stream, &root, &error) || !root.isObject())
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "dmq_dog: invalid status JSON: " << error);
    return;
  }

  DogStatus status;
  status.error = root.get("error", 0).asInt();
  status.status = root.get("status", 0).asInt();
  status.valid = status.error >= 0 && status.status >= 0;
  if (!status.valid) return;

  StatusCallback callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    callback = status_callback_;
  }
  if (callback) callback(status);
}

}  // namespace dmq_dog
