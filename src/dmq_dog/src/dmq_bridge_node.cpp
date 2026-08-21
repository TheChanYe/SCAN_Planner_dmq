#include "dmq_dog/dmq_mqtt_client.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt8.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <utility>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr unsigned char kRouteFollowMode = 1;
constexpr unsigned char kLocalAvoidMode = 2;

// MotionClass：当前周期下发速度的运动分类，仅用于日志诊断。
enum class MotionClass
{
  STOP,
  TURN,
  DRIVE
};

// clampValue：将数值限幅到 [-limit, limit] 区间内，limit<=0 时不限幅（直接返回原值）。
template <typename T>
T clampValue(const T value, const T limit)
{
  if (limit <= T{}) return value;
  return std::max(-limit, std::min(limit, value));
}

// applyMinimumEffectiveVelocity：死区修正。输入 value 为目标速度，zero_threshold 为
// 视作零的阈值，minimum_effective 为真机能真正响应的最小有效速度。
// 逻辑：小于zero_threshold直接归零；大于等于minimum_effective保持不变；
// 介于两者之间时抬升到minimum_effective（保留符号），避免真机因指令过小而不动。
double applyMinimumEffectiveVelocity(const double value,
                                     const double zero_threshold,
                                     const double minimum_effective)
{
  const double magnitude = std::fabs(value);
  if (magnitude < zero_threshold) return 0.0;
  if (magnitude >= minimum_effective) return value;
  return std::copysign(minimum_effective, value);
}

// classifyMotion：根据线速度模长与角速度将当前指令分类为 DRIVE/TURN/STOP，仅用于日志。
MotionClass classifyMotion(const double vx, const double vy,
                           const double yaw_rate)
{
  if (std::hypot(vx, vy) > 0.02) return MotionClass::DRIVE;
  if (std::fabs(yaw_rate) > 0.015) return MotionClass::TURN;
  return MotionClass::STOP;
}

// motionClassName：将 MotionClass 转为字符串，供日志打印使用。
const char* motionClassName(const MotionClass value)
{
  switch (value)
  {
    case MotionClass::DRIVE: return "DRIVE";
    case MotionClass::TURN: return "TURN";
    case MotionClass::STOP:
    default: return "STOP";
  }
}

// normalizeFrameId：去除 frame_id 开头的 '/'（tf2 要求 frame_id 不能带前导斜杠）。
std::string normalizeFrameId(std::string frame_id)
{
  if (!frame_id.empty() && frame_id.front() == '/') frame_id.erase(0, 1);
  return frame_id;
}

// DmqBridgeNode：dmq_dog 桥接节点。连接 ROS 与真机 MQTT 接口，职责包括：
//   1. 订阅 /cmd_vel 等规划指令，经限幅/死区/前进优先适配等处理后通过 MQTT 下发给真机；
//   2. 订阅真机上报的里程计/状态并转发为 ROS 消息；
//   3. 转发雷达点云/里程计给导航链路，并发布雷达安装位置的静态TF。
class DmqBridgeNode
{
public:
  // 构造函数：加载参数、注册所有订阅/发布者、绑定 MQTT 回调、发布雷达静态TF、
  // 启动 MQTT 客户端、创建周期性发布定时器。
  DmqBridgeNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)), pnh_(std::move(pnh)), mqtt_(loadMqttConfig())
  {
    loadRuntimeConfig();

    cmd_sub_ = nh_.subscribe(cmd_vel_topic_, 10, &DmqBridgeNode::cmdCallback,
                             this, ros::TransportHints().tcpNoDelay());
    ros_odom_sub_ = nh_.subscribe(ros_odom_topic_, 10,
        &DmqBridgeNode::rosOdomCallback, this, ros::TransportHints().tcpNoDelay());
    nav_state_sub_ = nh_.subscribe(nav_state_topic_, 10,
        &DmqBridgeNode::navStateCallback, this);
    nav_mode_sub_ = nh_.subscribe(nav_mode_topic_, 10,
        &DmqBridgeNode::navModeCallback, this);
    protocol_status_sub_ = nh_.subscribe(protocol_status_topic_, 10,
        &DmqBridgeNode::protocolStatusCallback, this);
    protocol_error_sub_ = nh_.subscribe(protocol_error_topic_, 10,
        &DmqBridgeNode::protocolErrorCallback, this);
    lidar_scan_sub_ = nh_.subscribe(lidar_scan_topic_, 10,
        &DmqBridgeNode::lidarScanCallback, this, ros::TransportHints().tcpNoDelay());
    lidar_cloud_sub_ = nh_.subscribe(lidar_cloud_topic_, 10,
        &DmqBridgeNode::lidarCloudCallback, this, ros::TransportHints().tcpNoDelay());

    mqtt_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(mqtt_odom_ros_topic_, 10);
    navigation_odom_pub_ =
        nh_.advertise<nav_msgs::Odometry>(navigation_odom_topic_, 10);
    navigation_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(navigation_cloud_topic_, 2);
    lidar_pose_pub_ = nh_.advertise<nav_msgs::Odometry>(lidar_pose_topic_, 10);
    dog_error_pub_ = nh_.advertise<std_msgs::UInt8>("dmq_dog/error", 10, true);
    dog_status_pub_ = nh_.advertise<std_msgs::UInt8>("dmq_dog/status", 10, true);

    mqtt_.setOdomCallback([this](const dmq_dog::DogOdom& odom) {
      publishMqttOdom(odom);
    });
    mqtt_.setStatusCallback([this](const dmq_dog::DogStatus& status) {
      updateDogStatus(status);
    });

    publishStaticLidarTf();

    if (!mqtt_.start())
      ROS_ERROR("dmq_dog: MQTT startup failed; node stays alive for diagnostics.");

    publish_timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, publish_rate_hz_)),
        &DmqBridgeNode::publishTimer, this);

    ROS_INFO_STREAM("dmq_dog: ready. cmd_vel=" << cmd_vel_topic_
                    << " ros_odom=" << ros_odom_topic_
                    << " lidar_scan=" << lidar_scan_topic_
                    << " lidar_cloud=" << lidar_cloud_topic_);
  }

private:
  // loadMqttConfig：从参数服务器读取 MQTT 相关配置（host/port/topic等），未设置时使用默认值。
  dmq_dog::MqttConfig loadMqttConfig()
  {
    dmq_dog::MqttConfig config;
    pnh_.param("mqtt/enabled", config.enabled, config.enabled);
    pnh_.param("mqtt/host", config.host, config.host);
    pnh_.param("mqtt/port", config.port, config.port);
    pnh_.param("mqtt/keepalive_sec", config.keepalive_sec, config.keepalive_sec);
    pnh_.param("mqtt/client_id", config.client_id, config.client_id);
    pnh_.param("mqtt/qos", config.qos, config.qos);
    pnh_.param("mqtt/control_topic", config.control_topic, config.control_topic);
    pnh_.param("mqtt/odom_topic", config.odom_topic, config.odom_topic);
    pnh_.param("mqtt/dog_status_topic", config.dog_status_topic,
               config.dog_status_topic);
    return config;
  }

  // loadRuntimeConfig：从参数服务器读取运行时配置。包括：topic名、发布频率与超时阈值、
  // 速度限幅、死区参数、前进优先适配参数（包括仅转向模式的进入/退出角度阈值转换）、
  // 坐标系名称、狗体尺寸与雷达安装位姿。其中 turn_only_enter/exit 角度带滞后区间，
  // exit角度不得超过enter角度，避免在临界值附近频繁抖动。
  void loadRuntimeConfig()
  {
    pnh_.param("topics/cmd_vel", cmd_vel_topic_, cmd_vel_topic_);
    pnh_.param("topics/ros_odom", ros_odom_topic_, ros_odom_topic_);
    pnh_.param("topics/lidar_scan", lidar_scan_topic_, lidar_scan_topic_);
    pnh_.param("topics/lidar_cloud", lidar_cloud_topic_, lidar_cloud_topic_);
    pnh_.param("topics/mqtt_odom_ros", mqtt_odom_ros_topic_, mqtt_odom_ros_topic_);
    pnh_.param("topics/navigation_odom", navigation_odom_topic_,
               navigation_odom_topic_);
    pnh_.param("topics/navigation_cloud", navigation_cloud_topic_,
               navigation_cloud_topic_);
    pnh_.param("topics/lidar_pose", lidar_pose_topic_, lidar_pose_topic_);
    pnh_.param("topics/nav_state", nav_state_topic_, nav_state_topic_);
    pnh_.param("topics/nav_mode", nav_mode_topic_, nav_mode_topic_);
    pnh_.param("topics/protocol_status", protocol_status_topic_,
               protocol_status_topic_);
    pnh_.param("topics/protocol_error", protocol_error_topic_,
               protocol_error_topic_);

    pnh_.param("publish_rate_hz", publish_rate_hz_, publish_rate_hz_);
    pnh_.param("cmd_vel_timeout_sec", cmd_vel_timeout_sec_, cmd_vel_timeout_sec_);
    pnh_.param("odom_timeout_sec", odom_timeout_sec_, odom_timeout_sec_);
    pnh_.param("lidar_timeout_sec", lidar_timeout_sec_, lidar_timeout_sec_);
    pnh_.param("use_watchdog_error", use_watchdog_error_, use_watchdog_error_);

    pnh_.param("limits/max_vx", max_vx_, max_vx_);
    pnh_.param("limits/max_vy", max_vy_, max_vy_);
    pnh_.param("limits/max_yaw_rate", max_yaw_rate_, max_yaw_rate_);
    pnh_.param("driver_limits/max_vx", max_vx_, max_vx_);
    pnh_.param("driver_limits/max_vy", max_vy_, max_vy_);
    pnh_.param("driver_limits/max_yaw_rate", max_yaw_rate_, max_yaw_rate_);
    pnh_.param("driver_deadband/enabled", deadband_enabled_, deadband_enabled_);
    pnh_.param("driver_deadband/zero_threshold_vx", zero_threshold_vx_,
               zero_threshold_vx_);
    pnh_.param("driver_deadband/zero_threshold_vy", zero_threshold_vy_,
               zero_threshold_vy_);
    pnh_.param("driver_deadband/zero_threshold_yaw_rate",
               zero_threshold_yaw_rate_, zero_threshold_yaw_rate_);
    pnh_.param("driver_deadband/min_effective_vx", min_effective_vx_,
               min_effective_vx_);
    pnh_.param("driver_deadband/min_effective_vy", min_effective_vy_,
               min_effective_vy_);
    pnh_.param("driver_deadband/min_effective_yaw_rate",
               min_effective_yaw_rate_, min_effective_yaw_rate_);
    pnh_.param("driver_motion/prefer_forward_motion",
               prefer_forward_motion_, prefer_forward_motion_);
    pnh_.param("driver_motion/lateral_to_yaw_gain",
               lateral_to_yaw_gain_, lateral_to_yaw_gain_);
    pnh_.param("driver_motion/max_yaw_rate",
               forward_motion_max_yaw_rate_, forward_motion_max_yaw_rate_);
    pnh_.param("driver_motion/lateral_filter_alpha",
               lateral_filter_alpha_, lateral_filter_alpha_);
    pnh_.param("driver_motion/max_yaw_accel",
               max_yaw_accel_, max_yaw_accel_);
    pnh_.param("driver_motion/max_yaw_decel",
               max_yaw_decel_, max_yaw_decel_);
    lateral_filter_alpha_ = std::max(0.0, std::min(1.0, lateral_filter_alpha_));
    double turn_only_angle_deg = turn_only_enter_angle_rad_ * 180.0 / kPi;
    pnh_.param("driver_motion/turn_only_angle_deg",
               turn_only_angle_deg, turn_only_angle_deg);
    double turn_only_enter_angle_deg = turn_only_angle_deg;
    double turn_only_exit_angle_deg =
        turn_only_exit_angle_rad_ * 180.0 / kPi;
    pnh_.param("driver_motion/turn_only_enter_angle_deg",
               turn_only_enter_angle_deg, turn_only_enter_angle_deg);
    pnh_.param("driver_motion/turn_only_exit_angle_deg",
               turn_only_exit_angle_deg, turn_only_exit_angle_deg);
    turn_only_enter_angle_rad_ =
        std::max(0.0, turn_only_enter_angle_deg) * kPi / 180.0;
    turn_only_exit_angle_rad_ =
        std::max(0.0, std::min(turn_only_exit_angle_deg,
                              turn_only_enter_angle_deg)) * kPi / 180.0;

    pnh_.param("frames/odom", odom_frame_id_, odom_frame_id_);
    pnh_.param("frames/base", base_frame_id_, base_frame_id_);
    pnh_.param("frames/lidar", lidar_frame_id_, lidar_frame_id_);

    pnh_.param("dog/length", dog_length_m_, dog_length_m_);
    pnh_.param("dog/width", dog_width_m_, dog_width_m_);
    pnh_.param("dog/height", dog_height_m_, dog_height_m_);

    pnh_.param("lidar/xyz/x", lidar_x_, dog_length_m_ * 0.5 - 0.10);
    pnh_.param("lidar/xyz/y", lidar_y_, lidar_y_);
    pnh_.param("lidar/xyz/z", lidar_z_, dog_height_m_);
    pnh_.param("lidar/rpy/roll", lidar_roll_, lidar_roll_);
    pnh_.param("lidar/rpy/pitch", lidar_pitch_, lidar_pitch_);
    pnh_.param("lidar/rpy/yaw", lidar_yaw_, lidar_yaw_);
  }

  // cmdCallback：接收规划下发的 /cmd_vel 指令，线程安全地缓存最新指令与时间戳，
  // 供 publishTimer 周期性取用（并用于超时判断）。
  void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cmd_ = *msg;
    latest_cmd_time_ = ros::Time::now();
  }

  // rosOdomCallback：接收 ROS 里程计消息。
  // 步骤：1.更新里程计时间戳用于看门狗超时检测；2.原样转发给导航链路；
  // 3.将机体位姿按雷达安装外参(lidar_x/y/z, lidar_roll/pitch/yaw)平移+旋转得到
  //   雷达在世界系下的位姿，发布为 lidar_pose 供定位/建图模块使用。
  void rosOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_ros_odom_time_ = ros::Time::now();
    }
    navigation_odom_pub_.publish(*msg);

    nav_msgs::Odometry lidar_pose = *msg;
    const tf::Quaternion body_q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Quaternion lidar_q;
    lidar_q.setRPY(lidar_roll_, lidar_pitch_, lidar_yaw_);
    const tf::Vector3 offset_world =
        tf::Matrix3x3(body_q) * tf::Vector3(lidar_x_, lidar_y_, lidar_z_);
    const tf::Quaternion world_lidar_q = body_q * lidar_q;
    lidar_pose.pose.pose.position.x += offset_world.x();
    lidar_pose.pose.pose.position.y += offset_world.y();
    lidar_pose.pose.pose.position.z += offset_world.z();
    lidar_pose.pose.pose.orientation.x = world_lidar_q.x();
    lidar_pose.pose.pose.orientation.y = world_lidar_q.y();
    lidar_pose.pose.pose.orientation.z = world_lidar_q.z();
    lidar_pose.pose.pose.orientation.w = world_lidar_q.w();
    lidar_pose.child_frame_id = lidar_frame_id_;
    lidar_pose_pub_.publish(lidar_pose);
  }

  // navStateCallback：接收导航状态机当前状态（NAV_STATE枚举值），缓存并标记为有效。
  void navStateCallback(const std_msgs::UInt8::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_nav_state_ = msg->data;
    nav_state_valid_ = true;
  }

  // navModeCallback：接收导航模式（ROUTE_FOLLOW/LOCAL_AVOID等）并缓存。
  // LOCAL_AVOID->ROUTE_FOLLOW 时只清理 SCAN lateral steering 状态，保留物理
  // yaw rate 连续性。
  void navModeCallback(const std_msgs::UInt8::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const unsigned char previous = latest_nav_mode_;
    latest_nav_mode_ = msg->data;
    if (previous == kLocalAvoidMode && latest_nav_mode_ == kRouteFollowMode)
      resetLateralSteeringAdapter();
  }

  void protocolStatusCallback(const std_msgs::UInt8::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_protocol_status_ = msg->data;
    protocol_status_valid_ = true;
  }

  void protocolErrorCallback(const std_msgs::UInt8::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_protocol_error_ = msg->data;
    protocol_error_valid_ = true;
  }

  // protocolState：将内部 NAV_STATE 枚举映射为真机 MQTT 协议约定的 status/error 编号。
  // 若尚未收到过导航状态，保持 status/error 不变。未知状态则回退为 status=0, error=1。
  void protocolState(int& status, int& error) const
  {
    if (protocol_status_valid_) status = latest_protocol_status_;
    if (protocol_error_valid_) error = latest_protocol_error_;
    if (protocol_status_valid_ || protocol_error_valid_) return;
    if (!nav_state_valid_) return;
    switch (latest_nav_state_)
    {
      case 0: status = 0; break;                  // IDLE
      case 1: status = 1; break;                  // PLANNING
      case 2: case 3: status = 3; break;          // START_ALIGN / TRACKING
      case 4: status = 5; break;                  // PAUSED
      case 5: status = 3; break;                  // RECOVERY
      case 6: status = 3; break;                  // GOAL_ALIGN
      case 7: status = 4; break;                  // SUCCEEDED
      case 8: status = 2; error = 2; break;       // EMERGENCY_STOP
      case 9: status = 0; error = 2; break;       // FAILED
      default: status = 0; error = 1; break;
    }
  }

  // lidarScanCallback：只用于更新雷达时间戳（看门狗），不转发具体数据。
  void lidarScanCallback(const sensor_msgs::LaserScan::ConstPtr&)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_lidar_time_ = ros::Time::now();
  }

  // lidarCloudCallback：更新雷达时间戳并将点云原样转发给导航链路使用。
  void lidarCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_lidar_time_ = ros::Time::now();
    }
    navigation_cloud_pub_.publish(*msg);
  }

  // resetMotionAdapter：重置前进优先适配器的内部状态（侧向误差滤波、仅转向标志、
  // 上次发布角速度与时间戳），在指令过时/为零时调用，避免遗留状态影响下一次运动。
  void resetMotionAdapter()
  {
    lateral_heading_error_ = 0.0;
    lateral_filter_initialized_ = false;
    turn_only_active_ = false;
    published_yaw_rate_ = 0.0;
    last_publish_time_ = ros::Time();
  }

  void resetLateralSteeringAdapter()
  {
    lateral_heading_error_ = 0.0;
    lateral_filter_initialized_ = false;
    turn_only_active_ = false;
  }

  // adaptToForwardMotion：将全向规划器输出的全向速度(vx,vy,yaw_rate)适配为
  // 非全向真机（只能前进+转向）可执行的指令。
  // 输入输出：vx/vy/yaw_rate 为引用传参，就地修改。
  // 步骤：
  //   1. 由 vy 计算朝向误差 heading_error=atan2(vy,|vx|)，并用一阶低通滤波
  //      （lateral_filter_alpha_）平滑到 lateral_heading_error_，避免抖动；
  //      vy接近零时误差指数衰减归零；
  //   2. 把该误差按增益 lateral_to_yaw_gain_ 叠加到 yaw_rate 上，让机体转向这个
  //      方向而非侧向平移，保留规划器的路径意图，并将 vy 清零；
  //   3. 带滞后判断是否进入/退出“仅转向”模式（turn_only_active_）：
  //      误差超过 enter 角度触发进入，低于 exit 角度退出，避免临界频繁切换；
  //   4. 仅转向模式下将 vx 也清零，适合大角度偏差时原地转向对齐而不走斜线；
  //   5. 模式发生切换时打印日志便于排查。
  void adaptToForwardMotion(double& vx, double& vy, double& yaw_rate)
  {
    if (std::fabs(vy) < 1e-6)
    {
      lateral_heading_error_ *= (1.0 - lateral_filter_alpha_);
      if (std::fabs(lateral_heading_error_) < 1e-4)
        lateral_heading_error_ = 0.0;
    }
    else
    {
      const double heading_error = std::atan2(vy, std::fabs(vx));
      if (!lateral_filter_initialized_)
      {
        lateral_heading_error_ = heading_error;
        lateral_filter_initialized_ = true;
      }
      else
      {
        lateral_heading_error_ += lateral_filter_alpha_ *
            (heading_error - lateral_heading_error_);
      }
    }

    // The planner is holonomic. Preserve its path intent while making the
    // physical dog rotate toward that direction instead of walking sideways.
    yaw_rate += lateral_to_yaw_gain_ * lateral_heading_error_;
    vy = 0.0;
    const bool previous_turn_only = turn_only_active_;
    const double abs_heading_error = std::fabs(lateral_heading_error_);
    if (!turn_only_active_ &&
        abs_heading_error >= turn_only_enter_angle_rad_)
      turn_only_active_ = true;
    else if (turn_only_active_ &&
             abs_heading_error <= turn_only_exit_angle_rad_)
      turn_only_active_ = false;

    if (turn_only_active_)
      vx = 0.0;

    if (turn_only_active_ != previous_turn_only)
    {
      ROS_INFO("DRIVER_TURN_HYSTERESIS active=%d heading_error_deg=%.1f "
               "enter_deg=%.1f exit_deg=%.1f",
               turn_only_active_ ? 1 : 0,
               lateral_heading_error_ * 180.0 / kPi,
               turn_only_enter_angle_rad_ * 180.0 / kPi,
               turn_only_exit_angle_rad_ * 180.0 / kPi);
    }
  }

  // limitPublishedYawRate：对实际下发的角速度做加加速度限制，避免真机转向突变。
  // 输入：target - 本周期期望的角速度；now - 当前时间（用于计算dt）。
  // 步骤：1.根据上次发布时刻计算实际dt（异常则回退到发布周期值）；
  // 2.若新旧方向相反，先按最大减速度递减到零再反向（真实转向必须过零才能反向）；
  // 3.否则根据是加速还是减速选择对应的最大变化量并限幅过渡。
  // 输出：限幅后的角速度，同时更新 published_yaw_rate_ 作为下次计算的起点。
  double limitPublishedYawRate(const double target, const ros::Time& now)
  {
    double dt = 1.0 / std::max(1.0, publish_rate_hz_);
    if (!last_publish_time_.isZero())
    {
      const double measured_dt = (now - last_publish_time_).toSec();
      if (measured_dt > 0.0 && measured_dt < 0.5) dt = measured_dt;
    }
    last_publish_time_ = now;

    double result = published_yaw_rate_;
    if (result * target < 0.0)
    {
      // A physical turn must pass through zero before reversing direction.
      const double step = std::min(std::fabs(result), max_yaw_decel_ * dt);
      result -= std::copysign(step, result);
    }
    else
    {
      const bool slowing = std::fabs(target) < std::fabs(result);
      const double max_delta = (slowing ? max_yaw_decel_ : max_yaw_accel_) * dt;
      result += clampValue(target - result, max_delta);
    }

    published_yaw_rate_ = result;
    return result;
  }

  // logMqttControl：诊断日志输出。在看门狗状态变化、运动分类变化时打印一次变化日志，
  // 并以 1Hz 频率打印完整 trace 日志。输入：原始指令、实际下发速度、指令是否
  // 过时/为零、当前status/error。无返回值，仅供排查使用。
  void logMqttControl(const geometry_msgs::Twist& raw_cmd,
                      const double vx, const double vy,
                      const double yaw_rate, const bool cmd_stale,
                      const bool zero_command, const int status,
                      const int error)
  {
    if (!watchdog_log_initialized_ || cmd_stale != last_logged_cmd_stale_)
    {
      if (cmd_stale)
      {
        ROS_WARN("MQTT_CMD_WATCHDOG state=STALE timeout=%.3f action=ZERO",
            cmd_vel_timeout_sec_);
      }
      else
      {
        ROS_INFO("MQTT_CMD_WATCHDOG state=FRESH");
      }
      last_logged_cmd_stale_ = cmd_stale;
      watchdog_log_initialized_ = true;
    }

    const MotionClass motion = classifyMotion(vx, vy, yaw_rate);
    const char* reason = cmd_stale ? "CMD_STALE" :
        (zero_command ? "INPUT_ZERO" :
        (motion == MotionClass::TURN ? "TURN_ONLY" :
        (motion == MotionClass::STOP ? "ADAPTER_ZERO" : "FORWARD")));

    if (!mqtt_motion_log_initialized_ || motion != last_logged_mqtt_motion_)
    {
      ROS_INFO("MQTT_CTRL motion=%s reason=%s status=%d error=%d "
               "raw=[%.3f %.3f %.3f] output=[%.3f %.3f %.3f]",
          motionClassName(motion), reason, status, error,
          raw_cmd.linear.x, raw_cmd.linear.y, raw_cmd.angular.z,
          vx, vy, yaw_rate);
      last_logged_mqtt_motion_ = motion;
      mqtt_motion_log_initialized_ = true;
    }

    ROS_INFO_THROTTLE(1.0,
        "MQTT_TRACE motion=%s reason=%s status=%d error=%d stale=%d "
        "raw=[%.3f %.3f %.3f] output=[%.3f %.3f %.3f]",
        motionClassName(motion), reason, status, error, cmd_stale ? 1 : 0,
        raw_cmd.linear.x, raw_cmd.linear.y, raw_cmd.angular.z,
        vx, vy, yaw_rate);
  }

  // publishTimer：定时器回调，按 publish_rate_hz_ 频率汇总当前指令/状态并通过 MQTT 下发给真机。
  // 步骤：
  //   1. 判断 cmd_vel 是否超时（cmd_stale），超时则使用全零指令；
  //   2. 确定 status/error：优先使用真机上报的状态，否则根据指令是否为零推断，
  //      再用 protocolState 根据导航状态机覆盖；
  //   3. 若启用看门狗且里程计/雷达数据超时，强制置 error=1；
  //   4. 仅 LOCAL_AVOID 下调用 adaptToForwardMotion 适配SCAN全向指令；
  //   5. 对 vx/vy/yaw_rate 依次做限幅（前进优先时角速度限幅取 max_yaw_rate_ 与
  //      forward_motion_max_yaw_rate_ 中较小者）；
  //   6. 若启用死区，对三个分量分别应用 applyMinimumEffectiveVelocity 修正；
  //   7. 前进优先模式下，指令过时/为零时重置适配器并强制角速度为0，否则对角速度
  //      应用 limitPublishedYawRate 做加加速度限制；
  //   8. 记录诊断日志，最后通过 MQTT 发布最终指令。
  void publishTimer(const ros::TimerEvent&)
  {
    geometry_msgs::Twist cmd;
    int error = 0;
    int status = 0;
    bool cmd_stale = false;
    unsigned char nav_mode = 0;
    const ros::Time now = ros::Time::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd_stale = latest_cmd_time_.isZero() ||
          (now - latest_cmd_time_).toSec() > cmd_vel_timeout_sec_;
      cmd = cmd_stale ? geometry_msgs::Twist{} : latest_cmd_;
      nav_mode = latest_nav_mode_;

      status = latest_dog_status_.valid ? latest_dog_status_.status :
          ((std::fabs(cmd.linear.x) > 1e-4 || std::fabs(cmd.linear.y) > 1e-4 ||
            std::fabs(cmd.angular.z) > 1e-4) ? 3 : 0);
      error = latest_dog_status_.valid ? latest_dog_status_.error : 0;
      protocolState(status, error);

      if (use_watchdog_error_ && !latest_ros_odom_time_.isZero() &&
          (now - latest_ros_odom_time_).toSec() > odom_timeout_sec_)
        error = 1;
      if (use_watchdog_error_ && !latest_lidar_time_.isZero() &&
          (now - latest_lidar_time_).toSec() > lidar_timeout_sec_)
        error = 1;
    }

    double vx = cmd.linear.x;
    double vy = cmd.linear.y;
    double yaw_rate = cmd.angular.z;
    const geometry_msgs::Twist raw_cmd = cmd;
    const bool zero_command = std::fabs(vx) < 1e-6 &&
        std::fabs(vy) < 1e-6 && std::fabs(yaw_rate) < 1e-6;
    if (prefer_forward_motion_ && nav_mode == kLocalAvoidMode)
      adaptToForwardMotion(vx, vy, yaw_rate);
    else if (nav_mode == kRouteFollowMode)
      vy = 0.0;

    vx = clampValue(vx, max_vx_);
    vy = clampValue(vy, max_vy_);
    const double yaw_limit = prefer_forward_motion_
        ? std::min(max_yaw_rate_, forward_motion_max_yaw_rate_)
        : max_yaw_rate_;
    yaw_rate = clampValue(yaw_rate, yaw_limit);
    if (deadband_enabled_)
    {
      vx = applyMinimumEffectiveVelocity(
          vx, zero_threshold_vx_, min_effective_vx_);
      vy = applyMinimumEffectiveVelocity(
          vy, zero_threshold_vy_, min_effective_vy_);
      yaw_rate = applyMinimumEffectiveVelocity(
          yaw_rate, zero_threshold_yaw_rate_, min_effective_yaw_rate_);
    }
    if (prefer_forward_motion_)
    {
      if (cmd_stale || zero_command)
      {
        resetMotionAdapter();
        yaw_rate = 0.0;
      }
      else
      {
        yaw_rate = limitPublishedYawRate(yaw_rate, now);
      }
    }
    logMqttControl(raw_cmd, vx, vy, yaw_rate, cmd_stale, zero_command,
        status, error);
    mqtt_.publishControl(error, status, vx, vy, yaw_rate);
  }

  // publishMqttOdom：将真机上报的里程计数据转换为 nav_msgs::Odometry 并发布，
  // 供需要真机自报位姿的上层模块使用（与rosOdomCallback转发的ROS自定位区分）。
  void publishMqttOdom(const dmq_dog::DogOdom& odom)
  {
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = odom_frame_id_;
    msg.child_frame_id = base_frame_id_;
    msg.pose.pose.position.x = odom.x;
    msg.pose.pose.position.y = odom.y;
    msg.pose.pose.position.z = odom.z;
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom.yaw);
    msg.twist.twist.linear.x = odom.vx;
    msg.twist.twist.linear.y = odom.vy;
    msg.twist.twist.angular.z = odom.yaw_rate;
    mqtt_odom_pub_.publish(msg);
  }

  // updateDogStatus：缓存真机上报的硬件状态，并将 error/status 分别发布为独立topic
  // （带latch，方便新订阅者立即拿到最后一次值）。
  void updateDogStatus(const dmq_dog::DogStatus& status)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_dog_status_ = status;
    }
    std_msgs::UInt8 error_msg;
    std_msgs::UInt8 status_msg;
    error_msg.data = static_cast<unsigned char>(std::max(0, std::min(255, status.error)));
    status_msg.data = static_cast<unsigned char>(std::max(0, std::min(255, status.status)));
    dog_error_pub_.publish(error_msg);
    dog_status_pub_.publish(status_msg);
  }

  // publishStaticLidarTf：根据雷达安装外参发布 base_link->lidar 的静态TF，
  // 并打印狗体尺寸与雷达安装位置日志。
  void publishStaticLidarTf()
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = normalizeFrameId(base_frame_id_);
    tf_msg.child_frame_id = normalizeFrameId(lidar_frame_id_);
    tf_msg.transform.translation.x = lidar_x_;
    tf_msg.transform.translation.y = lidar_y_;
    tf_msg.transform.translation.z = lidar_z_;
    tf::Quaternion q;
    q.setRPY(lidar_roll_, lidar_pitch_, lidar_yaw_);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    static_tf_broadcaster_.sendTransform(tf_msg);

    ROS_INFO_STREAM("dmq_dog: dog_size=[" << dog_length_m_ << ", "
                    << dog_width_m_ << ", " << dog_height_m_
                    << "] lidar_xyz=[" << lidar_x_ << ", " << lidar_y_
                    << ", " << lidar_z_ << "]");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  dmq_dog::MqttClient mqtt_;
  std::mutex mutex_;

  ros::Subscriber cmd_sub_;
  ros::Subscriber ros_odom_sub_;
  ros::Subscriber nav_state_sub_;
  ros::Subscriber nav_mode_sub_;
  ros::Subscriber protocol_status_sub_;
  ros::Subscriber protocol_error_sub_;
  ros::Subscriber lidar_scan_sub_;
  ros::Subscriber lidar_cloud_sub_;
  ros::Publisher mqtt_odom_pub_;
  ros::Publisher navigation_odom_pub_;
  ros::Publisher navigation_cloud_pub_;
  ros::Publisher lidar_pose_pub_;
  ros::Publisher dog_error_pub_;
  ros::Publisher dog_status_pub_;
  ros::Timer publish_timer_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  geometry_msgs::Twist latest_cmd_;
  dmq_dog::DogStatus latest_dog_status_;
  ros::Time latest_cmd_time_;
  ros::Time latest_ros_odom_time_;
  ros::Time latest_lidar_time_;
  unsigned char latest_nav_state_{0};
  unsigned char latest_nav_mode_{0};
  unsigned char latest_protocol_status_{0};
  unsigned char latest_protocol_error_{0};
  bool nav_state_valid_{false};
  bool protocol_status_valid_{false};
  bool protocol_error_valid_{false};

  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string ros_odom_topic_{"/Odometry"};
  std::string lidar_scan_topic_{"/scan"};
  std::string lidar_cloud_topic_{"/rslidar_points"};
  std::string mqtt_odom_ros_topic_{"/dmq_dog/odom"};
  std::string navigation_odom_topic_{"/dmq_dog/navigation_odom"};
  std::string navigation_cloud_topic_{"/dmq_dog/navigation_cloud"};
  std::string lidar_pose_topic_{"/dmq_dog/lidar_pose"};
  std::string nav_state_topic_{"/navdog/state"};
  std::string nav_mode_topic_{"/navdog/navigation_mode"};
  std::string protocol_status_topic_{"/navdog/protocol_status"};
  std::string protocol_error_topic_{"/navdog/protocol_error"};
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_link"};
  std::string lidar_frame_id_{"lidar"};

  double publish_rate_hz_{10.0};
  double cmd_vel_timeout_sec_{0.30};
  double odom_timeout_sec_{0.50};
  double lidar_timeout_sec_{0.50};
  double max_vx_{3.0};
  double max_vy_{1.0};
  double max_yaw_rate_{3.0};
  double zero_threshold_vx_{0.03};
  double zero_threshold_vy_{0.05};
  double zero_threshold_yaw_rate_{0.015};
  double min_effective_vx_{0.06};
  double min_effective_vy_{0.12};
  double min_effective_yaw_rate_{0.03};
  double lateral_to_yaw_gain_{1.2};
  double forward_motion_max_yaw_rate_{0.65};
  double lateral_filter_alpha_{0.20};
  double max_yaw_accel_{0.80};
  double max_yaw_decel_{1.20};
  double turn_only_enter_angle_rad_{35.0 * kPi / 180.0};
  double turn_only_exit_angle_rad_{20.0 * kPi / 180.0};
  double lateral_heading_error_{0.0};
  double published_yaw_rate_{0.0};
  ros::Time last_publish_time_;
  double dog_length_m_{0.60};
  double dog_width_m_{0.30};
  double dog_height_m_{0.30};
  double lidar_x_{0.20};
  double lidar_y_{0.0};
  double lidar_z_{0.30};
  double lidar_roll_{0.0};
  double lidar_pitch_{0.0};
  double lidar_yaw_{0.0};
  bool use_watchdog_error_{true};
  bool deadband_enabled_{true};
  bool prefer_forward_motion_{true};
  bool lateral_filter_initialized_{false};
  bool turn_only_active_{false};
  bool watchdog_log_initialized_{false};
  bool last_logged_cmd_stale_{true};
  bool mqtt_motion_log_initialized_{false};
  MotionClass last_logged_mqtt_motion_{MotionClass::STOP};
};

}  // namespace

// main：节点入口。初始化 ROS、构造 DmqBridgeNode（构造函数内完成所有订阅/发布/
// MQTT启动的初始化），进入 ros::spin() 循环处理回调。
int main(int argc, char** argv)
{
  ros::init(argc, argv, "dmq_bridge_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  // cppcheck-suppress unreadVariable
  DmqBridgeNode node(nh, pnh);
  ros::spin();
  return 0;
}
