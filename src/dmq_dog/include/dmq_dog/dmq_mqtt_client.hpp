#pragma once

#include <mosquitto.h>

#include <functional>
#include <mutex>
#include <string>

namespace dmq_dog
{

// MqttConfig：MQTT 连接参数与主题（topic）配置。
// enabled为false时，MqttClient 不会真正发起连接（start()直接返回true）。
struct MqttConfig
{
  bool enabled{true};                 // 是否启用MQTT
  std::string host{"127.0.0.1"};      // Broker地址
  int port{1883};                     // Broker端口
  int keepalive_sec{30};              // 心跳保活间隔（秒）
  std::string client_id{"dmq_dog"};   // 客户端ID前缀（实际使用时会拼接pid避免重名）
  int qos{1};                         // MQTT QoS等级
  std::string control_topic{"robot/local_planning/ctrl"}; // 下发控制指令的topic
  std::string odom_topic{"robot/dog/odom"};                // 订阅里程计的topic
  std::string dog_status_topic{"robot/dog/status"};        // 订阅狗体状态的topic
};

// DogOdom：从 MQTT 上报解析出的狗体里程计数据（位姿与速度）。
// valid 表示数据是否均为有限数（可直接使用）。
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

// DogStatus：从 MQTT 上报解析出的狗体硬件状态（错误码/状态码）。
struct DogStatus
{
  int error{0};
  int status{0};
  bool valid{false};
};

// MqttClient：封装 mosquitto 库的 MQTT 客户端，负责：
//   1. 向下发布控制指令（publishControl）给真机控制器；
//   2. 订阅并解析真机上报的里程计与状态数据，通过回调通知上层。
// 内部使用互斥锁保护连接状态与回调函数，因为 mosquitto 的网络回调跑在内部线程上。
class MqttClient
{
public:
  using OdomCallback = std::function<void(const DogOdom&)>;
  using StatusCallback = std::function<void(const DogStatus&)>;

  // 构造函数：保存配置并初始化 mosquitto 库。
  explicit MqttClient(const MqttConfig& config);
  // 析构函数：停止并释放 mosquitto 客户端资源。
  ~MqttClient();

  // start：创建 mosquitto 客户端实例、注册连接/断开/消息回调，开始异步连接并启动
  // 后台网络循环线程。返回値：是否启动成功（config.enabled=false时直接返回true）。
  bool start();
  // stop：停止网络循环、断开连接并销毁客户端实例。
  void stop();
  // connected：线程安全地查询当前是否已连接到 Broker。
  bool connected() const;

  // publishControl：将控制指令打包成 JSON 并发布到 control_topic。
  // 输入：error/status - 错误码/状态码（会被限幅到[0,255]）；vx/vy/yaw_rate - 目标速度。
  void publishControl(int error, int status,
                      double vx, double vy, double yaw_rate);
  // setOdomCallback：设置收到里程计数据时的回调（线程安全）。
  void setOdomCallback(OdomCallback callback);
  // setStatusCallback：设置收到狗体状态数据时的回调（线程安全）。
  void setStatusCallback(StatusCallback callback);

private:
  // onConnect：mosquitto 连接完成回调（静态，通过 data 拿到 this 指针），
  // 连接成功后自动订阅 odom_topic 与 dog_status_topic。
  static void onConnect(struct mosquitto* client, void* data, int rc);
  // onDisconnect：mosquitto 断开连接回调，更新 connected_ 状态。
  static void onDisconnect(struct mosquitto* client, void* data, int rc);
  // onMessage：mosquitto 收到订阅消息时的回调，根据 topic 分发到
  // handleOdomMessage 或 handleStatusMessage。
  static void onMessage(struct mosquitto* client, void* data,
                        const struct mosquitto_message* message);

  // handleOdomMessage：解析里程计 JSON 负载，校验数值有效性后通过 odom_callback_ 通知上层。
  void handleOdomMessage(const std::string& payload);
  // handleStatusMessage：解析狗体状态 JSON 负载，校验后通过 status_callback_ 通知上层。
  void handleStatusMessage(const std::string& payload);

  MqttConfig config_{};                  // MQTT配置
  struct mosquitto* client_{nullptr};    // mosquitto库客户端句柄
  mutable std::mutex mutex_{};           // 保护 connected_/回调函数的互斥锁
  bool started_{false};                  // 是否已调用过start()
  bool connected_{false};                // 当前是否已连接到Broker
  std::string resolved_client_id_;       // 实际使用的客户端ID（client_id+pid）
  OdomCallback odom_callback_;           // 里程计数据回调
  StatusCallback status_callback_;       // 狗体状态回调
};

}  // namespace dmq_dog
