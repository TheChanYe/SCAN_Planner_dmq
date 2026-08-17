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
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
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

  // statusForOutput：根据核心输出与协议错误标志推导上报的status/error码。
  static void statusForOutput(const navdog::CoreOutput& output,
      bool protocol_error, int& status, int& error);
  static void statusForOutput(const navdog::CoreOutput& output,
      bool dynamic_obstacle_stop, bool map_error, int& status, int& error);
  // toTwist：将核心层的VelocityCommand转换为geometry_msgs::Twist。
  static geometry_msgs::Twist toTwist(const navdog::VelocityCommand& command);
  // feedbackForAction：将规划器动作转换为反馈给NavigationCoordinator的PlannerFeedback。
  static navdog::PlannerFeedback feedbackForAction(
      const navdog::PlannerAction& action, double now_sec);
  // loadNavdogConfig：从ROS参数服务器加载核心导航配置NavdogConfig。
  static navdog::NavdogConfig loadNavdogConfig(ros::NodeHandle& nh);

private:
  /** @brief ROS 回调：将里程计转换为世界系 m/rad 的 RobotState 并互斥保存。 */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& message);
  // scanTakeoverReadyCallback：接收SCAN接管就绪信号，更新scan_takeover_ready_标志。
  void scanTakeoverReadyCallback(const std_msgs::Bool::ConstPtr& message);
  void finalCmdFeedbackCallback(const geometry_msgs::TwistStamped::ConstPtr& message);
  /**
   * @brief 固定 50 Hz 控制顺序：事件、输入快照、SCAN 观察、Core、SCAN 副作用、发布。
   * Runtime 不在此处重新判断 Route/SCAN 切换条件。
   */
  void controlCallback(const ros::TimerEvent&);
  // processEvents：从mqtt_消费待处理事件并依次提交给导航协调器处理。
  void processEvents();
  // processPlannerAction：将导航协调器产生的规划器动作（请求重规划等）分发给SCAN适配器。
  void processPlannerAction(const navdog::PlannerAction& action, double now_sec);
  /** @brief Reset is reserved for task lifecycle boundaries; path publication is deferred for ROS ordering. */
  void resetNativeScan(const char* reason);
  // scheduleNativeScanReferencePath：标记需要在本周期末尾向Native SCAN发布参考路径，
  // 延迟到controlCallback统一发布以保证ROS消息顺序。
  void scheduleNativeScanReferencePath();
  // handleScanRecovery：当SCAN接管尝试失败时按次数限制重试，超过上限则放弃并切回默认控制。
  void handleScanRecovery(const navdog::CoreOutput& output, double now_sec);
  // publishRoute：将当前导航任务的完整路线作为Path发布（供可视化/监控）。
  void publishRoute();
  /** @brief 发布当前进度后的剩余路点，已通过的路点绝不重新交给 Native SCAN。 */
  void publishNativeScanReferencePath(const navdog::RouteProgress& progress);
  // publishOutput：将核心输出的速度指令转换并发布到最终输出主topic。
  void publishOutput(const navdog::CoreOutput& output, double now_sec);
  // publishMqttStatus：根据核心输出编码并发布MQTT状态上报。
  void publishMqttStatus(const navdog::CoreOutput& output);
  void publishProtocolState(const navdog::CoreOutput& output);
  void publishMaxVxLimit(double max_vx);
  void updateTurnVoice(const navdog::CoreOutput& output);
  void updateDynamicObstacleStop(const navdog::CoreOutput& output,
      bool map_error, double now_sec);
  bool shouldPublishTurnVoice(const navdog::CoreOutput& output) const;
  // publishTakeoverSync：发布与Native SCAN接管同步相关的信息。
  void publishTakeoverSync(const navdog::CoreInput& input,
      const navdog::CoreOutput& output);
  /** @brief Reset native SCAN exactly once when the coordinator enters a terminal task state. */
  void handleTerminalTransition(const navdog::CoreOutput& output);
  // logNavigationChanges：对比本周期与上一次的导航状态/模式/进度，仅在发生变化时打印日志。
  void logNavigationChanges(const navdog::CoreOutput& output,
      const navdog::CoreInput& input);
  /**
   * @brief Read-only diagnostic: flags a sustained near-max-yaw-rate command
   * whose measured heading barely advances (see DEVELOPMENT_REQUIREMENTS on
   * odometry timeout/jump/impossible-speed safety stops). Odometry can stay
   * fresh (regular timestamps) while the reported pose itself is degraded or
   * stuck during in-place rotation; that case is not covered by the existing
   * SafetySupervisor odom_timeout_sec check. Never modifies final_cmd.
   */
  void checkHeadingStall(const navdog::CoreOutput& output,
      const navdog::CoreInput& input, double now_sec);

  ros::NodeHandle nh_;                    // 全局节点句柄
  ros::NodeHandle private_nh_;             // 私有（带命名空间）节点句柄
  ApplicationConfig application_config_{}; // 应用总配置
  std::unique_ptr<navdog::NavigationCoordinator> coordinator_;  // 纯 C++ 导航协调器
  std::shared_ptr<navdog_scan_adapter::ScanGridMapQuery> grid_query_;  // 膨胀地图查询
  std::unique_ptr<navdog_scan_adapter::ScanRouteCorridorEvaluator3D> corridor_evaluator_;  // 路径走廊评估器
  std::unique_ptr<navdog_scan_adapter::ScanObstacleSummaryEvaluator3D> obstacle_evaluator_;  // 障碍汇总评估器
  std::unique_ptr<navdog_protocol::MqttBridge> mqtt_;  // MQTT桥接

  ros::Subscriber odom_subscriber_;                       // 里程计订阅者
  ros::Subscriber scan_takeover_ready_subscriber_;        // SCAN接管就绪信号订阅者
  ros::Subscriber final_cmd_feedback_subscriber_;          // Mux最终速度反馈订阅者
  ros::Publisher route_publisher_;                        // 路线发布者
  ros::Publisher native_scan_path_publisher_;             // Native SCAN参考路径发布者
  ros::Publisher native_scan_reset_publisher_;            // Native SCAN重置信号发布者
  ros::Publisher native_scan_takeover_sync_publisher_;    // 接管同步信息发布者
  ros::Publisher state_publisher_;                        // 导航状态发布者
  ros::Publisher mode_publisher_;                         // 导航模式发布者
  ros::Publisher stair_up_active_publisher_;              // Native SCAN楼梯约束发布者
  ros::Publisher external_stop_publisher_;                 // 外部动态障碍停车请求
  ros::Publisher max_vx_limit_publisher_;                  // 当前任务速度上限
  ros::Publisher protocol_status_publisher_;               // 内部协议状态码
  ros::Publisher protocol_error_publisher_;                // 内部协议错误码
  ros::Publisher final_cmd_publisher_;                    // 最终速度指令发布者
  ros::Timer control_timer_;                              // 固定频率控制循环定时器

  mutable std::mutex odom_mutex_;                 // 保护robot_的互斥锁
  navdog::RobotState robot_{};                    // 最新里程计转换得到的机器人状态
  navdog::RouteProgress last_route_progress_{};   // 上一次的路线进度
  navdog::PlannerFeedback pending_planner_feedback_{};  // 待提交给协调器的规划器反馈
  ros::Time last_status_publish_{};               // 上一次状态发布时刻
  geometry_msgs::TwistStamped latest_final_cmd_feedback_{};
  bool latest_final_cmd_feedback_valid_{false};
  bool latest_map_error_{false};

  enum class DynamicObstacleState
  {
    CLEAR,
    STOPPING,
    WAIT_CLEAR
  };
  DynamicObstacleState dynamic_obstacle_state_{DynamicObstacleState::CLEAR};
  navdog_protocol::ExternalObstacleInfo latest_external_obstacle_{};
  ros::Time latest_external_obstacle_stamp_{};
  ros::Time dynamic_obstacle_stop_until_{};
  ros::Time last_turn_voice_publish_{};

  bool pending_native_scan_path_{false};          // 是否待发布Native SCAN参考路径
  bool pending_takeover_sync_{false};             // 是否待发布接管同步信息
  bool scan_takeover_ready_{false};               // SCAN接管是否就绪
  double scan_takeover_request_sec_{0.0};         // 发起接管请求的时刻
  double scan_takeover_timeout_sec_{1.5};         // 接管就绪等待超时
  int scan_recovery_attempts_{0};                 // 当前已尝试的SCAN恢复次数
  int scan_recovery_max_attempts_{2};             // 最大允许的SCAN恢复尝试次数
  std::uint32_t native_scan_reset_count_{0};      // Native SCAN重置累计次数
  ros::Time native_scan_reset_time_{};            // 最近一次Native SCAN重置时刻
  navdog::NavState last_logged_state_{navdog::NavState::IDLE};  // 上次记录日志的导航状态
  navdog::NavigationMode last_logged_mode_{navdog::NavigationMode::NONE};  // 上次记录日志的导航模式
  bool log_state_initialized_{false};             // 状态日志是否已初始化
  bool log_mode_initialized_{false};              // 模式日志是否已初始化
  bool last_logged_stair_up_active_{false};
  double last_logged_stair_hold_until_arc_m_{0.0};
  int last_logged_motion_class_{-1};              // 上次记录的运动分类
  std::size_t last_logged_segment_index_{0};      // 上次记录的路线段索引
  bool log_progress_initialized_{false};          // 进度日志是否已初始化
  navdog::NavState last_output_state_{navdog::NavState::IDLE};  // 上次输出的导航状态
  bool output_state_initialized_{false};          // 输出状态是否已初始化
  std::uint64_t terminal_cleanup_sequence_{0};    // 已执行终态清理的任务序号

  // 以下为checkHeadingStall()使用的状态；原因说明见该方法的注释。
  bool heading_stall_window_active_{false};       // 朝向卡滞检测窗口是否激活
  double heading_stall_window_start_sec_{0.0};    // 检测窗口开始时刻
  double heading_stall_window_start_yaw_{0.0};    // 窗口开始时的朝向
  double heading_stall_check_sec_{2.5};           // 卡滞检测窗口时长
  double heading_stall_min_delta_rad_{0.15};      // 窗口内朝向必须变化的最小弧度
  double heading_stall_yaw_rate_frac_{0.8};       // 视为“接近最大角速度”的比例阈值
};

}  // namespace navdog_runtime
