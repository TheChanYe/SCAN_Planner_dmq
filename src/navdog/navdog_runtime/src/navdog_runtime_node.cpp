#include "navdog_runtime/navdog_runtime_node.hpp"
#include "navdog_runtime/ros1_config_loader.hpp"

#include <navdog_protocol/mqtt_codec.hpp>
#include <plan_env/grid_map.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace navdog_runtime
{

namespace
{

// MotionClass：用于日志分类的运动状态枚举（停止/原地转向/前进）。
enum class MotionClass
{
  STOP = 0,
  TURN,
  DRIVE
};

// classifyMotion：根据最终速度指令粗略分类当前运动状态，仅用于日志变化检测，
// 不参与任何控制决策。判定顺序：指令无效→STOP；线速度模超过阈值→DRIVE；
// 角速度绝对值超过阈值→TURN；否则→STOP。
MotionClass classifyMotion(const navdog::VelocityCommand& cmd)
{
  if (!cmd.valid) return MotionClass::STOP;
  if (std::hypot(cmd.vx, cmd.vy) > 0.02) return MotionClass::DRIVE;
  if (std::fabs(cmd.yaw_rate) > 0.015) return MotionClass::TURN;
  return MotionClass::STOP;
}

// motionClassName：将MotionClass枚举转换为可读字符串，供日志打印使用。
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

bool navigationActive(const navdog::NavState state)
{
  return state == navdog::NavState::PLANNING ||
      state == navdog::NavState::START_ALIGN ||
      state == navdog::NavState::TRACKING ||
      state == navdog::NavState::GOAL_ALIGN ||
      state == navdog::NavState::RECOVERY;
}

}  // namespace

// loadNavdogConfig：从ROS参数服务器加载完整Runtime配置后，只取出其中的
// navdog_core配置部分返回（供外部只需要核心配置的场景使用，例如单元测试）。
navdog::NavdogConfig NavdogRuntimeNode::loadNavdogConfig(ros::NodeHandle& nh)
{ return Ros1ConfigLoader::load(nh).core; }

// 构造函数：保存全局/私有节点句柄，从ROS参数服务器加载完整应用配置
// （core/task/mqtt/runtime_io/final_output），并用core+task配置构造纯C++的
// NavigationCoordinator实例。此处不做任何ROS话题订阅/发布的注册，
// 具体的I/O创建延后到initialize()中完成。
NavdogRuntimeNode::NavdogRuntimeNode(
    ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(std::move(nh)), private_nh_(std::move(private_nh)),
      application_config_(Ros1ConfigLoader::load(private_nh_)),
      coordinator_(new navdog::NavigationCoordinator(
          application_config_.core, application_config_.task))
{}

// 析构函数：若MQTT桥接已启动，则显式停止其后台线程，避免对象销毁后线程
// 继续访问已释放的资源。
NavdogRuntimeNode::~NavdogRuntimeNode()
{ if (mqtt_) mqtt_->stop(); }

// initialize：完成Runtime节点的全部一次性初始化工作。
// 步骤：
// 1. 校验控制/状态频率参数合法（有限且为正）；
// 2. 从私有参数服务器加载SCAN接管超时/最大重试次数配置并校验；
// 3. 加载朝向卡滞诊断（heading_stall）相关参数并校验；
// 4. 创建独立的GridMap用于走廊/障碍评估（与Native SCAN节点自身的GridMap相互独立）；
// 5. 构造ScanGridMapQuery/走廊评估器/障碍汇总评估器；
// 6. 构造并启动MQTT桥接（启动失败不阻断本地导航，仅告警）；
// 7. 注册全部ROS订阅者/发布者，并创建固定频率控制定时器；
// 8. 打印导航模式与SCAN恢复相关的配置日志，便于问题定位。
// 返回值：任一关键校验失败返回false（节点应终止），否则返回true。
bool NavdogRuntimeNode::initialize()
{
  const auto& io = application_config_.runtime_io;
  if (!std::isfinite(io.control_rate_hz) || io.control_rate_hz <= 0.0 ||
      !std::isfinite(io.status_rate_hz) || io.status_rate_hz <= 0.0)
  {
    ROS_ERROR("invalid runtime rates");
    return false;
  }
  private_nh_.param("scan_recovery/takeover_timeout_sec",
      scan_takeover_timeout_sec_, 1.5);
  private_nh_.param("scan_recovery/max_attempts",
      scan_recovery_max_attempts_, 2);
  if (!std::isfinite(scan_takeover_timeout_sec_) ||
      scan_takeover_timeout_sec_ <= 0.0 || scan_recovery_max_attempts_ < 0)
  {
    ROS_ERROR("invalid SCAN recovery configuration");
    return false;
  }
  private_nh_.param("heading_stall/check_sec",
      heading_stall_check_sec_, 2.5);
  private_nh_.param("heading_stall/min_delta_rad",
      heading_stall_min_delta_rad_, 0.15);
  private_nh_.param("heading_stall/yaw_rate_frac",
      heading_stall_yaw_rate_frac_, 0.8);
  if (!std::isfinite(heading_stall_check_sec_) ||
      heading_stall_check_sec_ <= 0.0 ||
      !std::isfinite(heading_stall_min_delta_rad_) ||
      heading_stall_min_delta_rad_ < 0.0 ||
      !std::isfinite(heading_stall_yaw_rate_frac_) ||
      heading_stall_yaw_rate_frac_ <= 0.0 || heading_stall_yaw_rate_frac_ > 1.0)
  {
    ROS_ERROR("invalid heading_stall diagnostic configuration");
    return false;
  }

  // Create a standalone GridMap for corridor/obstacle evaluation.
  // The native SCAN node (scan_planner_dmq_node) owns its own GridMap.
  auto standalone_grid_map = std::make_shared<GridMap>();
  standalone_grid_map->initMap(private_nh_);
  grid_query_ = std::make_shared<navdog_scan_adapter::ScanGridMapQuery>(
      standalone_grid_map);
  corridor_evaluator_.reset(
      new navdog_scan_adapter::ScanRouteCorridorEvaluator3D(
          application_config_.core.route_corridor, grid_query_));
  obstacle_evaluator_.reset(
      new navdog_scan_adapter::ScanObstacleSummaryEvaluator3D(
          navdog_scan_adapter::ScanObstacleSummaryEvaluator3D::Config{},
          grid_query_));

  mqtt_.reset(new navdog_protocol::MqttBridge(application_config_.mqtt));
  if (!mqtt_->start())
    ROS_WARN("MQTT unavailable at startup; local navigation remains active");

  odom_subscriber_ = nh_.subscribe(io.odom_topic, 10,
      &NavdogRuntimeNode::odomCallback, this,
      ros::TransportHints().tcpNoDelay());
  scan_takeover_ready_subscriber_ = nh_.subscribe(
      "/native_scan/takeover_ready", 10,
      &NavdogRuntimeNode::scanTakeoverReadyCallback, this);
  final_cmd_feedback_subscriber_ = nh_.subscribe(
      io.final_cmd_feedback_topic, 10,
      &NavdogRuntimeNode::finalCmdFeedbackCallback, this,
      ros::TransportHints().tcpNoDelay());
  route_publisher_ =
      nh_.advertise<nav_msgs::Path>("/navdog/global_route", 1, true);
  native_scan_path_publisher_ =
      nh_.advertise<nav_msgs::Path>("/native_scan/initial_path", 1, true);
  native_scan_reset_publisher_ =
      nh_.advertise<std_msgs::Empty>(
          "/native_scan/reset", 1, false);
  native_scan_takeover_sync_publisher_ = nh_.advertise<std_msgs::UInt32>(
      "/native_scan/takeover_sync", 1, false);
  state_publisher_ = nh_.advertise<std_msgs::UInt8>("/navdog/state", 1);
  mode_publisher_ =
      nh_.advertise<std_msgs::UInt8>("/navdog/navigation_mode", 1);
  stair_up_active_publisher_ =
      nh_.advertise<std_msgs::Bool>("/navdog/stair_up_active", 1, true);
  external_stop_publisher_ =
      nh_.advertise<std_msgs::Bool>(io.external_stop_topic, 1, true);
  max_vx_limit_publisher_ =
      nh_.advertise<std_msgs::Float64>(io.max_vx_limit_topic, 1, true);
  protocol_status_publisher_ =
      nh_.advertise<std_msgs::UInt8>(io.protocol_status_topic, 1, true);
  protocol_error_publisher_ =
      nh_.advertise<std_msgs::UInt8>(io.protocol_error_topic, 1, true);
  final_cmd_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>(
      io.final_cmd_topic, 1);
  control_timer_ = nh_.createTimer(ros::Duration(1.0 / io.control_rate_hz),
      &NavdogRuntimeNode::controlCallback, this);

  const auto& nm = application_config_.core.navigation_mode;
  ROS_INFO("NAV_MODE_CONFIG enter_dist=%.3f enter_confirm=%.3f "
           "immediate_dist=%.3f min_avoid_hold=%.3f "
           "exit_confirm=%.3f exit_front=%.3f exit_left=%.3f exit_right=%.3f",
      nm.enter_blocked_distance_m, nm.enter_confirm_sec,
      nm.immediate_enter_distance_m, nm.min_local_avoid_hold_sec,
      nm.exit_clear_confirm_sec, nm.exit_front_clearance_m,
      nm.exit_left_clearance_m, nm.exit_right_clearance_m);
  ROS_INFO("SCAN_RECOVERY_CONFIG takeover_timeout=%.2f max_attempts=%d",
      scan_takeover_timeout_sec_, scan_recovery_max_attempts_);
  const auto& stair = application_config_.core.stair_up;
  ROS_INFO("STAIR_UP_CONFIG enabled=%d linear_speed=%.3f lookahead=%.2f "
           "trigger_rise=%.2f min_rising_points=%d min_average_slope=%.2f "
           "flat_tolerance=%.2f exit_margin=%.2f "
           "exit_confirm=%.2f",
      stair.enabled ? 1 : 0, stair.linear_speed_mps,
      stair.lookahead_distance_m,
      stair.trigger_rise_m, stair.min_consecutive_rising_points,
      stair.min_average_slope, stair.flat_tolerance_m,
      stair.exit_progress_margin_m, stair.exit_confirm_sec);

  return true;
}

// scanTakeoverReadyCallback：接收Native SCAN接管就绪generation。
void NavdogRuntimeNode::scanTakeoverReadyCallback(
    const std_msgs::UInt32::ConstPtr& msg)
{
  if (!msg) return;
  scan_takeover_ready_generation_ = msg->data;
  scan_takeover_ready_ = msg->data != 0 &&
      msg->data == scan_takeover_generation_;
  if (msg->data != 0 && msg->data != scan_takeover_generation_)
  {
    ROS_WARN_THROTTLE(1.0,
        "SCAN_TAKEOVER_READY_STALE expected=%u received=%u",
        scan_takeover_generation_, msg->data);
  }
}

void NavdogRuntimeNode::finalCmdFeedbackCallback(
    const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  if (!msg || !std::isfinite(msg->twist.linear.x) ||
      !std::isfinite(msg->twist.linear.y) ||
      !std::isfinite(msg->twist.angular.z))
  {
    ROS_WARN_THROTTLE(1.0, "FINAL_CMD_FEEDBACK_INVALID");
    return;
  }
  latest_final_cmd_feedback_ = *msg;
  latest_final_cmd_feedback_valid_ = true;
}

// odomCallback：ROS里程计回调，将Odometry消息转换为世界系下的RobotState。
// 步骤：1.抽取位置与从yaw；2.若配置表示旋速为机体系，则旋转到世界系；
// 3.抽取角速度与时间戳；4.对所有字段做有限性检查得到valid标志；
// 5.加锁写入共享状态robot_，供控制周期读取。
void NavdogRuntimeNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  navdog::RobotState robot{};
  robot.x = msg->pose.pose.position.x;
  robot.y = msg->pose.pose.position.y;
  robot.z = msg->pose.pose.position.z;
  robot.yaw = tf::getYaw(msg->pose.pose.orientation);
  robot.vx = msg->twist.twist.linear.x;
  robot.vy = msg->twist.twist.linear.y;
  if (!application_config_.runtime_io.odom_twist_in_world_frame)
  {
    const double x = robot.vx;
    const double y = robot.vy;
    robot.vx = std::cos(robot.yaw) * x - std::sin(robot.yaw) * y;
    robot.vy = std::sin(robot.yaw) * x + std::cos(robot.yaw) * y;
  }
  robot.yaw_rate = msg->twist.twist.angular.z;
  robot.stamp_sec = msg->header.stamp.toSec();
  robot.valid = std::isfinite(robot.x) && std::isfinite(robot.y) &&
      std::isfinite(robot.z) && std::isfinite(robot.yaw) &&
      std::isfinite(robot.vx) && std::isfinite(robot.vy) &&
      std::isfinite(robot.yaw_rate) && std::isfinite(robot.stamp_sec);
  std::lock_guard<std::mutex> lock(odom_mutex_);
  robot_ = robot;
}

// processEvents：从MQTT事件队列中依次取出待处理事件并提交给导航协调器处理，
// 根据处理结果做相应的副作用：
// - STARTED：重置终止清理序号/进度日志初始化标志，重置Native SCAN与接管计数，
//   发布完整路线并标记待发布参考路径（预热，让RouteFollower先控制机器人）；
// - CANCELLED：标记进度日志未初始化，若SCAN尚未因终止状态被清理则主动重置Native SCAN，
//   清空进度/反馈并发布空路线；
// - REJECTED_BUSY：仅节流日志提示当前任务占用中。
void NavdogRuntimeNode::processEvents()
{
  navdog_task::NavigationEvent event{};
  while (mqtt_ && mqtt_->popEvent(event))
  {
    const auto result = coordinator_->handleEvent(std::move(event));
    if (result == navdog_task::TaskHandleResult::STARTED)
    {
      terminal_cleanup_sequence_ = 0;
      log_progress_initialized_ = false;
      resetNativeScan("TASK_STARTED");
      scan_recovery_attempts_ = 0;
      scan_takeover_ready_ = false;
      scan_takeover_request_sec_ = 0.0;
      pending_takeover_sync_ = false;
      last_route_progress_ = navdog::RouteProgress{};
      publishRoute();
      // SCAN builds its global/local reference while RouteFollower owns the
      // robot.  This makes LOCAL_AVOID a command handoff, not a cold start.
      pending_native_scan_path_ = true;
      ROS_INFO("SCAN_PREWARM task_sequence=%lu route_points=%lu reset_count=%u",
          static_cast<unsigned long>(coordinator_->taskSession().sequence),
          static_cast<unsigned long>(coordinator_->routeManager().route().size()),
          native_scan_reset_count_);
      ROS_INFO("navigation task started: sequence=%lu",
          static_cast<unsigned long>(coordinator_->taskSession().sequence));
    }
    else if (result == navdog_task::TaskHandleResult::CANCELLED)
    {
      log_progress_initialized_ = false;
      const bool scan_already_cleaned =
          terminal_cleanup_sequence_ != 0 && output_state_initialized_ &&
          (last_output_state_ == navdog::NavState::SUCCEEDED ||
           last_output_state_ == navdog::NavState::FAILED);
      if (!scan_already_cleaned)
        resetNativeScan("TASK_CANCELLED");
      last_route_progress_ = navdog::RouteProgress{};
      pending_planner_feedback_ = navdog::PlannerFeedback{};
      route_publisher_.publish(nav_msgs::Path{});
      ROS_INFO("navigation task cancelled: scan_reset=%d",
          scan_already_cleaned ? 0 : 1);
    }
    else if (result == navdog_task::TaskHandleResult::REJECTED_BUSY)
    {
      ROS_INFO_THROTTLE(1.0,
          "NAV_EVENT_IGNORED type=START_TASK reason=TASK_BUSY active_sequence=%lu",
          static_cast<unsigned long>(coordinator_->taskSession().sequence));
    }
  }
}

// processPlannerAction：将导航协调器产生的规划器动作转换为下一周期提交给协调器的
// 规划器反馈：SET_ROUTE时构造就绪反馈，CANCEL时清空待提交反馈，其余类型不处理。
void NavdogRuntimeNode::processPlannerAction(
    const navdog::PlannerAction& action, double now_sec)
{
  if (action.type == navdog::PlannerActionType::SET_ROUTE)
  {
    pending_planner_feedback_ = feedbackForAction(action, now_sec);
    publishMaxVxLimit(action.max_vx);
  }
  else if (action.type == navdog::PlannerActionType::UPDATE_SPEED_LIMIT)
  {
    publishMaxVxLimit(action.max_vx);
    ROS_INFO("NAV_MAX_VX_UPDATE sequence=%lu max_vx=%.3f",
        static_cast<unsigned long>(action.task.sequence), action.max_vx);
  }
  else if (action.type == navdog::PlannerActionType::CANCEL)
    pending_planner_feedback_ = navdog::PlannerFeedback{};
}

// feedbackForAction：将SET_ROUTE类型的规划器动作转换为就绪状态的PlannerFeedback；
// 若非SET_ROUTE、序号为0或now_sec无效，则返回默认（无效）反馈。
navdog::PlannerFeedback NavdogRuntimeNode::feedbackForAction(
    const navdog::PlannerAction& action, double now_sec)
{
  navdog::PlannerFeedback feedback{};
  if (action.type != navdog::PlannerActionType::SET_ROUTE ||
      action.task.sequence == 0 || !std::isfinite(now_sec)) return feedback;
  feedback.state = navdog::PlannerState::READY;
  feedback.trajectory_id = action.task.sequence;
  feedback.stamp_sec = now_sec;
  feedback.valid = true;
  return feedback;
}

// controlCallback：固定频率（默认50Hz）的主控制循环，严格按下列顺序执行：
// 1. 处理待处理MQTT事件（processEvents）；
// 2. 加锁读取最新机器人状态作为CoreInput输入；
// 3. 若障碍汇总评估器存在则评估障碍信息；
// 4. 若当前有有效路线且上次进度有效且地图已就绪，则评估路径走廊观测；
// 5. 写入待提交的规划器反馈并清空；
// 6. 调用导航协调器的update()得到本周期输出；
// 7. 记录导航状态变化日志、检查朝向卡滞诊断；
// 8. 若导航模式发生转换，处理进入/退出LOCAL_AVOID的交接日志与同步发布；
// 9. 处理SCAN接管超时重试；
// 10. 延迟发布待发布的Native SCAN参考路径（确保重置先于路径到达）；
// 11. 分发规划器动作并更新路线进度快照；
// 12. 发布最终速度指令/状态/模式；
// 13. 处理终止状态迁移（先发布终止状态让mux硬停车，再重置Native SCAN，
//     避免50Hz重置循环）；
// 14. 按频率限制发布MQTT状态。
// Runtime不在此处重新判断Route/SCAN切换条件，该判断始终留在navdog_core。
void NavdogRuntimeNode::controlCallback(const ros::TimerEvent&)
{
  const double now_sec = ros::Time::now().toSec();
  processEvents();
  navdog::CoreInput input{};
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    input.robot = robot_;
  }
  if (obstacle_evaluator_)
    input.obstacles = obstacle_evaluator_->evaluate(input.robot, now_sec);
  if (coordinator_->routeManager().hasRoute() && input.robot.valid &&
      last_route_progress_.valid && grid_query_ && grid_query_->ready())
  {
    input.route_corridor_observation = corridor_evaluator_->evaluate(
        coordinator_->routeManager().taskView(), last_route_progress_,
        input.robot, now_sec);
  }
  input.planner = pending_planner_feedback_;
  pending_planner_feedback_ = navdog::PlannerFeedback{};

  const navdog::CoreOutput output = coordinator_->update(input, now_sec);
  latest_map_error_ = (output.state == navdog::NavState::START_ALIGN ||
      output.state == navdog::NavState::TRACKING ||
      output.state == navdog::NavState::GOAL_ALIGN) &&
      !input.obstacles.valid &&
      (!output.route_corridor.valid ||
       !std::isfinite(output.route_corridor.map_stamp_sec));
  logNavigationChanges(output, input);
  checkHeadingStall(output, input, now_sec);
  if (output.navigation_mode.transitioned)
  {
    if (output.navigation_mode.mode == navdog::NavigationMode::LOCAL_AVOID)
    {
      beginScanTakeover("MODE_ENTER", input, output, now_sec);
      scan_recovery_attempts_ = 0;
      ROS_INFO("SCAN_HANDOFF_ENTER prewarmed=1 scan_ready=%d",
          pending_native_scan_path_ ? 0 : 1);
    }
    else if (output.navigation_mode.previous_mode ==
                 navdog::NavigationMode::LOCAL_AVOID)
    {
      ROS_INFO("SCAN_HANDOFF_EXIT route_segment_index=%lu route_arc_length=%.3f remaining_distance=%.3f",
          static_cast<unsigned long>(output.route_progress.segment_index),
          output.route_progress.arc_length_m,
          output.route_progress.remaining_distance_m);
    }
  }
  handleScanRecovery(output, now_sec);
  // Deferred native scan reference path: ensure reset arrives before path
  scheduleNativeScanReferencePath();
  processPlannerAction(output.planner_action, now_sec);
  if (output.route_progress.valid) last_route_progress_ = output.route_progress;
  publishOutput(output, now_sec);
  updateDynamicObstacleStop(output, latest_map_error_, now_sec);
  publishProtocolState(output);
  updateTurnVoice(output);
  // Publish the terminal state first so the mux hard-stops before native
  // SCAN is reset.  The edge detector prevents a 50 Hz reset loop.
  handleTerminalTransition(output);
  if (application_config_.runtime_io.publish_mqtt_status &&
      (last_status_publish_.isZero() ||
      (ros::Time::now() - last_status_publish_).toSec() >=
          1.0 / application_config_.runtime_io.status_rate_hz))
  {
    publishMqttStatus(output);
    last_status_publish_ = ros::Time::now();
  }
}

std::uint32_t NavdogRuntimeNode::beginScanTakeover(const char* reason,
    const navdog::CoreInput& input, const navdog::CoreOutput& output,
    double now_sec)
{
  ++scan_takeover_generation_;
  if (scan_takeover_generation_ == 0)
    ++scan_takeover_generation_;
  scan_takeover_ready_generation_ = 0;
  scan_takeover_ready_ = false;
  scan_takeover_request_sec_ = now_sec;
  pending_takeover_sync_ = false;
  std_msgs::UInt32 generation;
  generation.data = scan_takeover_generation_;
  native_scan_takeover_sync_publisher_.publish(generation);
  ROS_INFO("SCAN_TAKEOVER_SYNC_REQUEST generation=%u reason=%s sequence=%lu "
           "robot_x=%.3f robot_y=%.3f robot_vx=%.3f robot_vy=%.3f",
      scan_takeover_generation_, reason ? reason : "UNKNOWN",
      static_cast<unsigned long>(output.task_sequence), input.robot.x,
      input.robot.y, input.robot.vx, input.robot.vy);
  return scan_takeover_generation_;
}

// handleScanRecovery：当处于LOCAL_AVOID模式且SCAN接管迟迟未就绪时，按次数限制重试：
// 1. 若不处于LOCAL_AVOID，清空计时/计数并在需要时取消待发布标志，直接返回；
// 2. 若已就绪，清空计时并返回；
// 3. 若尚未开始计时，记录请求时刻并返回；
// 4. 若超过接管超时时长，判断重试次数是否已用尽：已用尽则仅告警不再重试（安全停车）；
// 5. 否则增加重试计数、主动重置Native SCAN并标记待重新发布参考路径/接管同步，
//    并重置请求计时以开始下一轮超时等待。
void NavdogRuntimeNode::handleScanRecovery(
    const navdog::CoreOutput& output, double now_sec)
{
  if (output.navigation_mode.mode != navdog::NavigationMode::LOCAL_AVOID)
  {
    scan_takeover_request_sec_ = 0.0;
    scan_recovery_attempts_ = 0;
    if (pending_takeover_sync_)
      pending_native_scan_path_ = false;
    pending_takeover_sync_ = false;
    return;
  }

  if (scan_takeover_ready_)
  {
    scan_takeover_request_sec_ = 0.0;
    return;
  }

  if (scan_takeover_request_sec_ <= 0.0)
  {
    scan_takeover_request_sec_ = now_sec;
    return;
  }

  const double elapsed = now_sec - scan_takeover_request_sec_;
  if (!std::isfinite(elapsed) || elapsed < scan_takeover_timeout_sec_)
    return;

  if (scan_recovery_attempts_ >= scan_recovery_max_attempts_)
  {
    ROS_ERROR_THROTTLE(2.0,
        "SCAN_RECOVERY_EXHAUSTED sequence=%lu attempts=%d action=SAFE_STOP",
        static_cast<unsigned long>(output.task_sequence),
        scan_recovery_attempts_);
    return;
  }

  ++scan_recovery_attempts_;
  ROS_WARN("SCAN_TAKEOVER_TIMEOUT sequence=%lu elapsed=%.3f attempt=%d/%d",
      static_cast<unsigned long>(output.task_sequence), elapsed,
      scan_recovery_attempts_, scan_recovery_max_attempts_);
  resetNativeScan("TAKEOVER_TIMEOUT");
  pending_native_scan_path_ = true;
  pending_takeover_sync_ = true;
  scan_takeover_request_sec_ = now_sec;
}

// logNavigationChanges：对比本周期与上一次记录的导航状态/运动分类/路线进度段/
// 导航模式，仅在发生变化时打印对应日志（避免日志泛滥）：
// - NAV_STATE：状态变化时打印前后状态、指令、进度、机器人位姿等完整信息；
// - NAV_COMMAND：运动分类变化时打印当前运动/状态/模式/指令；
// - ROUTE_PROGRESS：路线段索引变化时打印前后段与进度详情；
// - NAV_TRACE：固定1Hz节流打印完整状态追踪信息（不依赖变化检测）；
// - NAV_MODE：导航模式变化时打印详细的走廊/障碍观测与确认计时信息。
void NavdogRuntimeNode::logNavigationChanges(
    const navdog::CoreOutput& output, const navdog::CoreInput& input)
{
  const auto& elevation = output.route_elevation;
  const auto& stair_config = application_config_.core.stair_up;
  if (stair_config.enabled && elevation.valid &&
      !elevation.baseline_consistent)
  {
    ROS_WARN_THROTTLE(1.0,
        "STAIR_UP_REJECT reason=ROUTE_Z_BASELINE_DROP "
        "current_z=%.3f baseline_z=%.3f baseline_drop=%.3f "
        "allowed=%.3f arc=%.3f",
        elevation.current_z,
        elevation.baseline_z,
        elevation.baseline_drop_m,
        stair_config.baseline_drop_tolerance_m,
        output.route_progress.arc_length_m);
  }
  const bool robot_z_valid = input.robot.valid && std::isfinite(input.robot.z);
  const double route_robot_z_error = robot_z_valid
      ? std::fabs(elevation.current_z - input.robot.z)
      : 0.0;
  if (stair_config.enabled && elevation.valid && elevation.ascending &&
      elevation.baseline_consistent &&
      (!robot_z_valid ||
       route_robot_z_error > stair_config.max_robot_route_z_error_m))
  {
    ROS_WARN_THROTTLE(1.0,
        "STAIR_UP_REJECT reason=ROUTE_ROBOT_Z_MISMATCH "
        "route_z=%.3f robot_z=%.3f error=%.3f allowed=%.3f arc=%.3f",
        elevation.current_z,
        input.robot.z,
        route_robot_z_error,
        stair_config.max_robot_route_z_error_m,
        output.route_progress.arc_length_m);
  }

  const bool stair_activated = !last_logged_stair_up_active_ &&
      output.navigation_mode.stair_up_active;
  if (stair_activated)
  {
    ROS_INFO("STAIR_UP_TRIGGER rise=%.3f average_slope=%.3f "
             "current_z=%.3f baseline_z=%.3f ascent_end_z=%.3f rising_points=%d "
             "robot_z=%.3f arc=%.3f hold_until=%.3f mode_transition=%d",
        elevation.rise_m, elevation.average_slope,
        elevation.current_z, elevation.baseline_z, elevation.ascent_end_z,
        elevation.consecutive_rising_points,
        input.robot.z, output.route_progress.arc_length_m,
        output.navigation_mode.stair_hold_until_arc_m,
        output.navigation_mode.transitioned ? 1 : 0);
  }
  if (last_logged_stair_up_active_ &&
      !output.navigation_mode.stair_up_active)
  {
    ROS_INFO("STAIR_UP_EXIT arc=%.3f hold_until=%.3f",
        output.route_progress.arc_length_m,
        last_logged_stair_hold_until_arc_m_);
  }
  last_logged_stair_up_active_ = output.navigation_mode.stair_up_active;
  if (output.navigation_mode.stair_up_active)
  {
    last_logged_stair_hold_until_arc_m_ =
        output.navigation_mode.stair_hold_until_arc_m;
  }

  if (!log_state_initialized_ || output.state != last_logged_state_)
  {
    const auto& progress = output.route_progress;
    ROS_INFO("NAV_STATE prev=%s next=%s seq=%lu source=%s "
             "cmd_valid=%d cmd=[%.3f %.3f %.3f] "
             "progress_valid=%d segment=%lu ratio=%.3f remaining=%.3f "
             "lateral_error=%.3f robot=[%.3f %.3f %.3f]",
        log_state_initialized_ ? navdog::navStateName(last_logged_state_) : "NONE",
        navdog::navStateName(output.state),
        static_cast<unsigned long>(output.task_sequence),
        navdog::commandSourceName(output.final_cmd.source),
        output.final_cmd.valid ? 1 : 0,
        output.final_cmd.vx, output.final_cmd.vy, output.final_cmd.yaw_rate,
        progress.valid ? 1 : 0,
        static_cast<unsigned long>(progress.segment_index),
        progress.segment_ratio, progress.remaining_distance_m,
        progress.lateral_error_m,
        input.robot.x, input.robot.y, input.robot.yaw);
    last_logged_state_ = output.state;
    log_state_initialized_ = true;
  }

  const MotionClass motion = classifyMotion(output.final_cmd);
  if (last_logged_motion_class_ != static_cast<int>(motion))
  {
    const auto& progress = output.route_progress;
    ROS_INFO("NAV_COMMAND motion=%s state=%s mode=%s source=%s valid=%d "
             "cmd=[%.3f %.3f %.3f] segment=%lu ratio=%.3f remaining=%.3f",
        motionClassName(motion), navdog::navStateName(output.state),
        navdog::navigationModeName(output.navigation_mode.mode),
        navdog::commandSourceName(output.final_cmd.source),
        output.final_cmd.valid ? 1 : 0,
        output.final_cmd.vx, output.final_cmd.vy, output.final_cmd.yaw_rate,
        static_cast<unsigned long>(progress.segment_index),
        progress.segment_ratio, progress.remaining_distance_m);
    last_logged_motion_class_ = static_cast<int>(motion);
  }

  if (output.route_progress.valid &&
      (!log_progress_initialized_ ||
       output.route_progress.segment_index != last_logged_segment_index_))
  {
    ROS_INFO("ROUTE_PROGRESS segment_prev=%lu segment_next=%lu ratio=%.3f "
             "arc=%.3f remaining=%.3f lateral=%.3f motion=%s cmd=[%.3f %.3f %.3f]",
        static_cast<unsigned long>(log_progress_initialized_
            ? last_logged_segment_index_ : output.route_progress.segment_index),
        static_cast<unsigned long>(output.route_progress.segment_index),
        output.route_progress.segment_ratio,
        output.route_progress.arc_length_m,
        output.route_progress.remaining_distance_m,
        output.route_progress.lateral_error_m,
        motionClassName(motion), output.final_cmd.vx,
        output.final_cmd.vy, output.final_cmd.yaw_rate);
    last_logged_segment_index_ = output.route_progress.segment_index;
    log_progress_initialized_ = true;
  }

  ROS_INFO_THROTTLE(1.0,
      "NAV_TRACE state=%s mode=%s source=%s cmd=[%.3f %.3f %.3f] "
      "segment=%lu ratio=%.3f arc=%.3f remaining=%.3f lateral=%.3f "
      "robot=[%.3f %.3f %.3f] obstacles=[%.3f %.3f %.3f]",
      navdog::navStateName(output.state),
      navdog::navigationModeName(output.navigation_mode.mode),
      navdog::commandSourceName(output.final_cmd.source),
      output.final_cmd.vx, output.final_cmd.vy, output.final_cmd.yaw_rate,
      static_cast<unsigned long>(output.route_progress.segment_index),
      output.route_progress.segment_ratio, output.route_progress.arc_length_m,
      output.route_progress.remaining_distance_m,
      output.route_progress.lateral_error_m,
      input.robot.x, input.robot.y, input.robot.yaw,
      input.obstacles.front_min, input.obstacles.left_min,
      input.obstacles.right_min);

  if (!log_mode_initialized_ ||
      output.navigation_mode.mode !=
          last_logged_mode_)
  {
    const auto& mode =
        output.navigation_mode;

    const auto& corridor =
        input.route_corridor_observation;

    const auto& obstacles =
        input.obstacles;

    ROS_INFO(
        "NAV_MODE "
        "prev=%s next=%s "
        "seq=%lu "
        "reason=%s "
        "corridor_valid=%d "
        "corridor_blocked=%d "
        "checked_distance=%.3f "
        "blocked_forward=%.3f "
        "obstacle_valid=%d "
        "front_min=%.3f "
        "left_min=%.3f "
        "right_min=%.3f "
        "blocked_confirm=%.3f "
        "clear_confirm=%.3f "
        "avoid_cycle=%u",
        log_mode_initialized_
            ? navdog::navigationModeName(
                  last_logged_mode_)
            : "NONE",
        navdog::navigationModeName(
            mode.mode),
        static_cast<unsigned long>(
            output.task_sequence),
        navdog::navigationModeReasonName(
            mode.reason),
        corridor.valid ? 1 : 0,
        corridor.blocked ? 1 : 0,
        corridor.checked_distance_m,
        corridor.first_blocked_distance_ahead_m,
        obstacles.valid ? 1 : 0,
        obstacles.front_min,
        obstacles.left_min,
        obstacles.right_min,
        mode.blocked_confirm_elapsed_sec,
        mode.clear_confirm_elapsed_sec,
        static_cast<unsigned>(
            mode.avoidance_cycle_count));

    last_logged_mode_ =
        mode.mode;

    log_mode_initialized_ = true;
  }
}

// checkHeadingStall：只读诊断，不修改final_cmd。用于检测“持续接近最大角速度
// 指令但实际朝向几乎不推进”的异常情况（可能意味着定位退化或机体卡滞，
// 而现有SafetySupervisor的odom_timeout_sec只能检测里程计时间戳过时，
// 无法覆盖里程计持续新鲜但位姿本身停滞的情况）。
// 步骤：
// 1. 输入无效则重置窗口并返回；
// 2. 判断是否“几乎不平移+接近最大角速度”，否则重置窗口并返回；
// 3. 若窗口未激活，开启新窗口（记录起始时刻与起始yaw）并返回；
// 4. 若窗口时长未达到检查间隔，返回继续累积；
// 5. 计算并归一化yaw变化量，若小于阈值则告警输出存在持续旋转但无实际转向的异常；
// 6. 不论是否告警，重新开启窗口以便持续旋转过程中反复检查。
void NavdogRuntimeNode::checkHeadingStall(
    const navdog::CoreOutput& output, const navdog::CoreInput& input,
    double now_sec)
{
  // Read-only diagnostic; never modifies output.final_cmd. See header
  // comment for rationale: the existing SafetySupervisor odom_timeout_sec
  // check only detects stale odometry timestamps, not odometry that keeps
  // arriving on time while the reported pose itself stops advancing during
  // an in-place rotation (a signature of degraded/lost localization).
  if (!input.robot.valid || !std::isfinite(input.robot.yaw) ||
      !output.final_cmd.valid || !std::isfinite(now_sec))
  {
    heading_stall_window_active_ = false;
    return;
  }

  const double max_w = application_config_.core.limits.max_yaw_rate;
  const bool near_max_turn =
      std::isfinite(max_w) && max_w > 1e-6 &&
      std::hypot(output.final_cmd.vx, output.final_cmd.vy) <= 0.02 &&
      std::fabs(output.final_cmd.yaw_rate) >=
          heading_stall_yaw_rate_frac_ * max_w;

  if (!near_max_turn)
  {
    heading_stall_window_active_ = false;
    return;
  }

  if (!heading_stall_window_active_)
  {
    heading_stall_window_active_ = true;
    heading_stall_window_start_sec_ = now_sec;
    heading_stall_window_start_yaw_ = input.robot.yaw;
    return;
  }

  const double elapsed = now_sec - heading_stall_window_start_sec_;
  if (!std::isfinite(elapsed) || elapsed < heading_stall_check_sec_)
    return;

  double delta_yaw = input.robot.yaw - heading_stall_window_start_yaw_;
  while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
  while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;

  if (std::fabs(delta_yaw) < heading_stall_min_delta_rad_)
  {
    ROS_WARN("NAV_HEADING_STALL commanded_w=%.3f actual_delta_yaw=%.3f "
             "elapsed=%.2f state=%s mode=%s",
        output.final_cmd.yaw_rate, delta_yaw, elapsed,
        navdog::navStateName(output.state),
        navdog::navigationModeName(output.navigation_mode.mode));
  }

  // Restart the window regardless of outcome so a sustained turn keeps
  // being re-checked instead of triggering once and going silent.
  heading_stall_window_active_ = true;
  heading_stall_window_start_sec_ = now_sec;
  heading_stall_window_start_yaw_ = input.robot.yaw;
}

// toTwist：将navdog核心层的VelocityCommand转换为geometry_msgs::Twist。
// 若指令无效或存在非有限值，则返回默认零速度的Twist。
geometry_msgs::Twist NavdogRuntimeNode::toTwist(
    const navdog::VelocityCommand& command)
{
  geometry_msgs::Twist result;
  if (command.valid && std::isfinite(command.vx) &&
      std::isfinite(command.vy) && std::isfinite(command.yaw_rate))
  {
    result.linear.x = command.vx;
    result.linear.y = command.vy;
    result.angular.z = command.yaw_rate;
  }
  return result;
}

// publishOutput：将核心输出的速度指令转换并发布到最终输出主topic，
// 同时发布当前导航状态与导航模式（供其他节点/监控订阅）。
void NavdogRuntimeNode::publishOutput(
    const navdog::CoreOutput& output, double now_sec)
{
  geometry_msgs::TwistStamped command;
  command.header.stamp.fromSec(now_sec);
  command.twist = toTwist(output.final_cmd);
  final_cmd_publisher_.publish(command);
  std_msgs::UInt8 state;
  state.data = static_cast<std::uint8_t>(output.state);
  state_publisher_.publish(state);
  std_msgs::UInt8 mode;
  mode.data = static_cast<std::uint8_t>(output.navigation_mode.mode);
  mode_publisher_.publish(mode);
  std_msgs::Bool stair_up_active;
  stair_up_active.data = output.navigation_mode.stair_up_active;
  stair_up_active_publisher_.publish(stair_up_active);
}

// publishRoute：将当前导航任务的完整路线作为Path发布（供可视化/监控使用）。
void NavdogRuntimeNode::publishRoute()
{
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";
  for (const auto& point : coordinator_->routeManager().route())
  {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(point.has_yaw ? point.yaw : 0.0);
    path.poses.push_back(pose);
  }
  route_publisher_.publish(path);
}

// handleTerminalTransition：当导航协调器进入终止状态（SUCCEEDED/FAILED）时精确重置
// Native SCAN一次（供同一任务序号只触发一次，避免重复重置）：
// 1. 判断是否刚刚进入SUCCEEDED或FAILED；
// 2. 若刚进入且本任务序号尚未清理过，则标记已清理、重置Native SCAN，
//    清空待发布标志与待提交反馈，若成功则通知MQTT完成当前活动任务，
//    并发布空路线、打印终止日志；
// 3. 无论是否发生迁移，都更新上次输出状态与初始化标志。
void NavdogRuntimeNode::handleTerminalTransition(
    const navdog::CoreOutput& output)
{
  const bool entered_succeeded =
      output.state == navdog::NavState::SUCCEEDED &&
      (!output_state_initialized_ || last_output_state_ != navdog::NavState::SUCCEEDED);
  const bool entered_failed =
      output.state == navdog::NavState::FAILED &&
      (!output_state_initialized_ || last_output_state_ != navdog::NavState::FAILED);

  if ((entered_succeeded || entered_failed) && output.task_sequence != 0 &&
      terminal_cleanup_sequence_ != output.task_sequence)
  {
    terminal_cleanup_sequence_ = output.task_sequence;
    resetNativeScan(entered_succeeded ? "TASK_SUCCEEDED" : "TASK_FAILED");
    pending_native_scan_path_ = false;
    pending_planner_feedback_ = navdog::PlannerFeedback{};
    if (entered_succeeded && mqtt_)
      mqtt_->completeActiveTask();

    nav_msgs::Path empty_path;
    empty_path.header.stamp = ros::Time::now();
    empty_path.header.frame_id = "world";
    route_publisher_.publish(empty_path);
    ROS_INFO("NAV_TASK_TERMINAL sequence=%lu state=%s scan_reset=1",
        static_cast<unsigned long>(output.task_sequence),
        navdog::navStateName(output.state));
  }

  last_output_state_ = output.state;
  output_state_initialized_ = true;
}

// resetNativeScan：向Native SCAN发布重置信号，并更新重置时刻与累计重置次数、
// 取消待发布标志，最后打印重置原因告警日志便于问题定位。
void NavdogRuntimeNode::resetNativeScan(const char* reason)
{
  native_scan_reset_publisher_.publish(std_msgs::Empty{});

  pending_native_scan_path_ = false;
  native_scan_reset_time_ = ros::Time::now();
  ++native_scan_reset_count_;

  ROS_WARN("NATIVE_SCAN_RESET reason=%s",
      reason ? reason : "UNKNOWN");
}

// scheduleNativeScanReferencePath：若标记了待发布Native SCAN参考路径，
// 且距上次重置已经过了最小延迟（确保Native SCAN先收到重置信号），
// 则实际发布参考路径并清除标志；若同时有待发布的接管同步，
// 则发布同步信号并重置接管请求计时（开启下一轮超时等待）。
void NavdogRuntimeNode::scheduleNativeScanReferencePath()
{
  if (pending_native_scan_path_ &&
      (ros::Time::now() - native_scan_reset_time_).toSec() >= 0.02)
  {
    publishNativeScanReferencePath(last_route_progress_);
    pending_native_scan_path_ = false;
    if (pending_takeover_sync_)
    {
      navdog::CoreInput input{};
      input.robot = robot_;
      navdog::CoreOutput output{};
      output.task_sequence = coordinator_ && coordinator_->hasActiveTask()
          ? coordinator_->taskSession().sequence : 0;
      beginScanTakeover("RECOVERY_RETRY", input, output,
          ros::Time::now().toSec());
      pending_takeover_sync_ = false;
      ROS_INFO("SCAN_RECOVERY_SYNC_REQUEST attempt=%d",
          scan_recovery_attempts_);
    }
  }
}

// publishNativeScanReferencePath：将当前进度之后的剩余路点作为参考路径发布给
// Native SCAN，已通过的路点绝不重新交给Native SCAN。
// 步骤：1.无效路线直接返回；2.根据进度确定首个剩余索引（若进度序号与当前
// 任务序号匹配）；3.首点使用当前机器人位置；4.后续点仅取首个剩余索引之后的
// 路点；5.发布并打印详细日志。
void NavdogRuntimeNode::publishNativeScanReferencePath(
    const navdog::RouteProgress& progress)
{
  if (!coordinator_ || !coordinator_->routeManager().hasRoute())
    return;

  const auto& route = coordinator_->routeManager().route();
  if (route.size() < 2)
    return;

  // Determine the first remaining route index from progress.
  // Never re-send waypoints the robot has already passed.
  std::size_t first_remaining_index = 0;
  if (progress.valid &&
      progress.task_sequence ==
          coordinator_->taskSession().sequence)
  {
    first_remaining_index =
        std::min(
            progress.segment_index + 1,
            route.size() - 1);
  }

  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";

  // First point: current robot position.
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = robot_.x;
    pose.pose.position.y = robot_.y;
    pose.pose.position.z = robot_.z;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_.yaw);
    path.poses.push_back(pose);
  }

  // Remaining points: only waypoints after the current progress segment.
  for (std::size_t i = first_remaining_index; i < route.size(); ++i)
  {
    const auto& point = route[i];
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(
        point.has_yaw ? point.yaw : 0.0);
    path.poses.push_back(pose);
  }

  native_scan_path_publisher_.publish(path);
  ROS_INFO("NATIVE_SCAN_PATH_PUBLISHED points=%lu "
           "first_remaining_index=%lu",
      static_cast<unsigned long>(path.poses.size()),
      static_cast<unsigned long>(first_remaining_index));
}

void NavdogRuntimeNode::updateDynamicObstacleStop(
    const navdog::CoreOutput& output, bool, double now_sec)
{
  bool updated = false;
  navdog_protocol::ExternalObstacleInfo obstacle{};
  while (mqtt_ && mqtt_->latestObstacle(obstacle))
  {
    latest_external_obstacle_ = obstacle;
    latest_external_obstacle_stamp_.fromSec(now_sec);
    updated = true;
  }

  const auto& config = application_config_.dynamic_obstacle;
  const bool enabled = config.enabled &&
      std::isfinite(config.stop_distance_m) &&
      std::isfinite(config.hold_sec) &&
      std::isfinite(config.timeout_sec) &&
      config.stop_distance_m > 0.0 &&
      config.hold_sec >= 0.0 &&
      config.timeout_sec > 0.0;

  const bool obstacle_fresh = !latest_external_obstacle_stamp_.isZero() &&
      (ros::Time::now() - latest_external_obstacle_stamp_).toSec() <=
          config.timeout_sec;
  const bool dynamic_close = enabled && navigationActive(output.state) &&
      obstacle_fresh && latest_external_obstacle_.valid &&
      latest_external_obstacle_.error == 0 &&
      latest_external_obstacle_.status == 2 &&
      std::isfinite(latest_external_obstacle_.distance) &&
      latest_external_obstacle_.distance <= config.stop_distance_m;
  const bool dynamic_clear = !obstacle_fresh ||
      !latest_external_obstacle_.valid ||
      latest_external_obstacle_.error != 0 ||
      latest_external_obstacle_.status != 2 ||
      (std::isfinite(latest_external_obstacle_.distance) &&
       latest_external_obstacle_.distance > config.stop_distance_m);

  switch (dynamic_obstacle_state_)
  {
    case DynamicObstacleState::CLEAR:
      if (dynamic_close)
      {
        dynamic_obstacle_state_ = DynamicObstacleState::STOPPING;
        dynamic_obstacle_stop_until_.fromSec(now_sec + config.hold_sec);
        ROS_WARN("DYNAMIC_OBSTACLE_STOP status=%u distance=%.3f hold=%.3f",
            static_cast<unsigned>(latest_external_obstacle_.status),
            latest_external_obstacle_.distance, config.hold_sec);
      }
      break;

    case DynamicObstacleState::STOPPING:
      if (!navigationActive(output.state))
      {
        dynamic_obstacle_state_ = DynamicObstacleState::CLEAR;
        dynamic_obstacle_stop_until_ = ros::Time{};
      }
      else if (!dynamic_obstacle_stop_until_.isZero() &&
          now_sec >= dynamic_obstacle_stop_until_.toSec())
      {
        dynamic_obstacle_state_ = DynamicObstacleState::WAIT_CLEAR;
        ROS_INFO("DYNAMIC_OBSTACLE_HOLD_DONE action=WAIT_CLEAR");
      }
      break;

    case DynamicObstacleState::WAIT_CLEAR:
      if (!navigationActive(output.state) || dynamic_clear)
      {
        dynamic_obstacle_state_ = DynamicObstacleState::CLEAR;
        dynamic_obstacle_stop_until_ = ros::Time{};
        ROS_INFO("DYNAMIC_OBSTACLE_CLEAR updated=%d", updated ? 1 : 0);
      }
      break;
  }

  std_msgs::Bool stop;
  stop.data = dynamic_obstacle_state_ == DynamicObstacleState::STOPPING;
  external_stop_publisher_.publish(stop);
}

bool NavdogRuntimeNode::shouldPublishTurnVoice(
    const navdog::CoreOutput& output) const
{
  const auto& config = application_config_.turn_voice;
  if (!config.enabled || !mqtt_ || !latest_final_cmd_feedback_valid_ ||
      !navigationActive(output.state) ||
      output.state == navdog::NavState::PAUSED ||
      output.state == navdog::NavState::SUCCEEDED)
    return false;

  if (!std::isfinite(config.min_yaw_rate) ||
      !std::isfinite(config.max_linear_speed) ||
      !std::isfinite(config.cooldown_sec) ||
      config.min_yaw_rate < 0.0 || config.max_linear_speed < 0.0 ||
      config.cooldown_sec < 0.0)
    return false;

  const auto& twist = latest_final_cmd_feedback_.twist;
  const bool turning =
      std::fabs(twist.angular.z) >= config.min_yaw_rate &&
      std::hypot(twist.linear.x, twist.linear.y) <= config.max_linear_speed;
  if (!turning) return false;

  return last_turn_voice_publish_.isZero() ||
      (ros::Time::now() - last_turn_voice_publish_).toSec() >=
          config.cooldown_sec;
}

// statusForOutput：根据核心输出状态与协议错误标志推导上报的status/error码：
// 失败/紧急停止时status=0且error=2；暂停时status=5；
// 规划/对齐/跟踪/恢复中等中间过程状态status=1；其余默认status=0，
// error取决于输入的protocol_error。
void NavdogRuntimeNode::statusForOutput(const navdog::CoreOutput& output,
    bool protocol_error, int& status, int& error)
{
  (void)protocol_error;
  statusForOutput(output, false, false, status, error);
}

void NavdogRuntimeNode::statusForOutput(const navdog::CoreOutput& output,
    bool dynamic_obstacle_stop, bool map_error, int& status, int& error)
{
  status = 0;
  error = map_error ? 1 : 0;
  if (dynamic_obstacle_stop)
  { status = 2; error = 0; return; }
  if (output.state == navdog::NavState::FAILED ||
      output.state == navdog::NavState::EMERGENCY_STOP)
  { status = 0; error = 2; return; }
  if (output.state == navdog::NavState::PAUSED) status = 5;
  else if (output.state == navdog::NavState::PLANNING) status = 1;
  else if (output.state == navdog::NavState::SUCCEEDED)
    status = output.obstacle_finished ? 6 : 4;
  else if (output.state == navdog::NavState::START_ALIGN ||
           output.state == navdog::NavState::TRACKING ||
           output.state == navdog::NavState::GOAL_ALIGN ||
           output.state == navdog::NavState::RECOVERY) status = 3;
}

// publishMqttStatus：根据核心输出与MQTT协议错误计数编码并发布MQTT状态上报，
// 若MQTT桥接不存在则直接返回。
void NavdogRuntimeNode::publishMqttStatus(const navdog::CoreOutput& output)
{
  if (!mqtt_) return;
  int status = 0;
  int error = 0;
  const int protocol_errors = mqtt_->consumeProtocolError();
  if (protocol_errors > 0)
    ROS_WARN_THROTTLE(2.0,
        "MQTT_PROTOCOL_ERRORS count=%d action=LOG_ONLY", protocol_errors);
  statusForOutput(output,
      dynamic_obstacle_state_ == DynamicObstacleState::STOPPING,
      latest_map_error_, status, error);
  const auto& cmd = latest_final_cmd_feedback_;
  const double vx = latest_final_cmd_feedback_valid_ ? cmd.twist.linear.x : 0.0;
  const double vy = latest_final_cmd_feedback_valid_ ? cmd.twist.linear.y : 0.0;
  const double yaw_rate =
      latest_final_cmd_feedback_valid_ ? cmd.twist.angular.z : 0.0;
  mqtt_->publishStatus(
      navdog_protocol::MqttCodec::encodeStatus(status, error, vx, vy, yaw_rate));
}

void NavdogRuntimeNode::publishMaxVxLimit(double max_vx)
{
  if (!std::isfinite(max_vx) || max_vx <= 0.0) return;
  std_msgs::Float64 message;
  message.data = max_vx;
  max_vx_limit_publisher_.publish(message);
}

void NavdogRuntimeNode::updateTurnVoice(const navdog::CoreOutput& output)
{
  if (!shouldPublishTurnVoice(output)) return;
  mqtt_->publishVoice(navdog_protocol::MqttCodec::encodeVoiceMessage(
      application_config_.turn_voice.message));
  last_turn_voice_publish_ = ros::Time::now();
  const auto& twist = latest_final_cmd_feedback_.twist;
  ROS_INFO("TURN_VOICE_PUBLISHED state=%s mode=%s vx=%.3f yaw_rate=%.3f",
      navdog::navStateName(output.state),
      navdog::navigationModeName(output.navigation_mode.mode),
      twist.linear.x, twist.angular.z);
}

void NavdogRuntimeNode::publishProtocolState(const navdog::CoreOutput& output)
{
  int status = 0;
  int error = 0;
  statusForOutput(output,
      dynamic_obstacle_state_ == DynamicObstacleState::STOPPING,
      latest_map_error_, status, error);
  std_msgs::UInt8 status_msg;
  std_msgs::UInt8 error_msg;
  status_msg.data = static_cast<std::uint8_t>(std::max(0, std::min(255, status)));
  error_msg.data = static_cast<std::uint8_t>(std::max(0, std::min(255, error)));
  protocol_status_publisher_.publish(status_msg);
  protocol_error_publisher_.publish(error_msg);
}

}  // namespace navdog_runtime
