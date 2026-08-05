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

template <typename T>
T clampValue(const T value, const T limit)
{
  if (limit <= T{}) return value;
  return std::max(-limit, std::min(limit, value));
}

double applyMinimumEffectiveVelocity(const double value,
                                     const double zero_threshold,
                                     const double minimum_effective)
{
  const double magnitude = std::fabs(value);
  if (magnitude < zero_threshold) return 0.0;
  if (magnitude >= minimum_effective) return value;
  return std::copysign(minimum_effective, value);
}

void preferForwardMotion(double& vx, double& vy, double& yaw_rate,
                         const double lateral_to_yaw_gain,
                         const double turn_only_angle_rad)
{
  if (std::fabs(vy) < 1e-6) return;

  // Convert the holonomic body-frame velocity direction into steering. This
  // keeps SCAN's 2-D path intent while making the physical dog face its path.
  const double motion_heading_error = std::atan2(vy, std::fabs(vx));
  yaw_rate += lateral_to_yaw_gain * motion_heading_error;
  vy = 0.0;

  if (std::fabs(motion_heading_error) >= turn_only_angle_rad)
    vx = 0.0;
}

std::string normalizeFrameId(std::string frame_id)
{
  if (!frame_id.empty() && frame_id.front() == '/') frame_id.erase(0, 1);
  return frame_id;
}

class DmqBridgeNode
{
public:
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
    double turn_only_angle_deg = turn_only_angle_rad_ * 180.0 / kPi;
    pnh_.param("driver_motion/turn_only_angle_deg",
               turn_only_angle_deg, turn_only_angle_deg);
    turn_only_angle_rad_ = std::max(0.0, turn_only_angle_deg) * kPi / 180.0;

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

  void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cmd_ = *msg;
    latest_cmd_time_ = ros::Time::now();
  }

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

  void navStateCallback(const std_msgs::UInt8::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_nav_state_ = msg->data;
    nav_state_valid_ = true;
  }

  void navModeCallback(const std_msgs::UInt8::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_nav_mode_ = msg->data;
  }

  void protocolState(int& status, int& error) const
  {
    if (!nav_state_valid_) return;
    switch (latest_nav_state_)
    {
      case 0: status = 0; break;                  // IDLE
      case 1: status = 1; break;                  // PLANNING
      case 2: case 3: status = 3; break;          // START_ALIGN / TRACKING
      case 4: status = 5; break;                  // PAUSED
      case 5: status = 3; break;                  // RECOVERY
      case 6: status = 6; break;                  // GOAL_ALIGN
      case 7: status = 4; break;                  // SUCCEEDED
      case 8: status = 2; error = 2; break;       // EMERGENCY_STOP
      case 9: status = 0; error = 2; break;       // FAILED
      default: status = 0; error = 1; break;
    }
  }

  void lidarScanCallback(const sensor_msgs::LaserScan::ConstPtr&)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_lidar_time_ = ros::Time::now();
  }

  void lidarCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_lidar_time_ = ros::Time::now();
    }
    navigation_cloud_pub_.publish(*msg);
  }

  void publishTimer(const ros::TimerEvent&)
  {
    geometry_msgs::Twist cmd;
    int error = 0;
    int status = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const ros::Time now = ros::Time::now();
      const bool cmd_stale = latest_cmd_time_.isZero() ||
          (now - latest_cmd_time_).toSec() > cmd_vel_timeout_sec_;
      cmd = cmd_stale ? geometry_msgs::Twist{} : latest_cmd_;

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
    if (prefer_forward_motion_)
      preferForwardMotion(vx, vy, yaw_rate,
          lateral_to_yaw_gain_, turn_only_angle_rad_);

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
    mqtt_.publishControl(error, status, vx, vy, yaw_rate);
  }

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
  bool nav_state_valid_{false};

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
  double turn_only_angle_rad_{25.0 * kPi / 180.0};
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
};

}  // namespace

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
