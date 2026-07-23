#pragma once

#include "navdog_runtime/application_config.hpp"

#include <navdog_core/navigation_coordinator.hpp>
#include <navdog_protocol/mqtt_bridge.hpp>
#include <navdog_scan_adapter/scan_grid_map_query.hpp>
#include <navdog_scan_adapter/scan_obstacle_summary_evaluator_3d.hpp>
#include <navdog_scan_adapter/scan_route_corridor_evaluator_3d.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>

#include <memory>
#include <mutex>
#include <cstdint>

namespace navdog_runtime
{

class NavdogRuntimeNode
{
public:
  /**
   * @brief Runtime 是 ROS、MQTT、SCAN 与纯 C++ NavigationCoordinator 的组合根。
   *
   * 它只做消息/时间转换、固定顺序调度与发布；Route/SCAN 进入退出判断始终
   * 留在 navdog_core 的 NavigationModeManager。
   */
  NavdogRuntimeNode(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~NavdogRuntimeNode();
  /** @brief 创建 ROS I/O、SCAN 适配器与 MQTT 桥；不改变核心默认参数。 */
  bool initialize();

  static void statusForOutput(const navdog::CoreOutput& output,
      bool protocol_error, int& status, int& error);
  static geometry_msgs::Twist toTwist(const navdog::VelocityCommand& command);
  static navdog::PlannerFeedback feedbackForAction(
      const navdog::PlannerAction& action, double now_sec);
  static navdog::NavdogConfig loadNavdogConfig(ros::NodeHandle& nh);

private:
  /** @brief ROS 回调：将里程计转换为世界系 m/rad 的 RobotState 并互斥保存。 */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& message);
  /**
   * @brief 固定 50 Hz 控制顺序：事件、输入快照、SCAN 观察、Core、SCAN 副作用、发布。
   * Runtime 不在此处重新判断 Route/SCAN 切换条件。
   */
  void controlCallback(const ros::TimerEvent&);
  void processEvents();
  void processPlannerAction(const navdog::PlannerAction& action, double now_sec);
  /** @brief Reset is reserved for task lifecycle boundaries; path publication is deferred for ROS ordering. */
  void resetNativeScan(const char* reason);
  void scheduleNativeScanReferencePath();
  void publishRoute();
  /** @brief 发布当前进度后的剩余路点，已通过的路点绝不重新交给 Native SCAN。 */
  void publishNativeScanReferencePath(const navdog::RouteProgress& progress);
  void publishOutput(const navdog::CoreOutput& output, double now_sec);
  void publishMqttStatus(const navdog::CoreOutput& output);
  void logNavigationChanges(const navdog::CoreOutput& output,
      const navdog::CoreInput& input);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ApplicationConfig application_config_{};
  std::unique_ptr<navdog::NavigationCoordinator> coordinator_;
  std::shared_ptr<navdog_scan_adapter::ScanGridMapQuery> grid_query_;
  std::unique_ptr<navdog_scan_adapter::ScanRouteCorridorEvaluator3D> corridor_evaluator_;
  std::unique_ptr<navdog_scan_adapter::ScanObstacleSummaryEvaluator3D> obstacle_evaluator_;
  std::unique_ptr<navdog_protocol::MqttBridge> mqtt_;

  ros::Subscriber odom_subscriber_;
  ros::Publisher route_publisher_;
  ros::Publisher native_scan_path_publisher_;
  ros::Publisher native_scan_reset_publisher_;
  ros::Publisher state_publisher_;
  ros::Publisher mode_publisher_;
  ros::Publisher final_cmd_publisher_;
  ros::Timer control_timer_;

  mutable std::mutex odom_mutex_;
  navdog::RobotState robot_{};
  navdog::RouteProgress last_route_progress_{};
  navdog::PlannerFeedback pending_planner_feedback_{};
  ros::Time last_status_publish_{};

  bool pending_native_scan_path_{false};
  std::uint32_t native_scan_reset_count_{0};
  ros::Time native_scan_reset_time_{};
  double body_height_{0.3};
  navdog::NavState last_logged_state_{navdog::NavState::IDLE};
  navdog::NavigationMode last_logged_mode_{navdog::NavigationMode::NONE};
  bool log_state_initialized_{false};
  bool log_mode_initialized_{false};
};

}  // namespace navdog_runtime
