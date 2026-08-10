
#include <plan_manage_dmq/scan_replan_fsm.h>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <unistd.h>

namespace
{
  std::string shellQuote(const std::string &value)
  {
    std::string quoted = "'";
    for (const char c : value)
    {
      if (c == '\'')
        quoted += "'\\''";
      else
        quoted += c;
    }
    quoted += "'";
    return quoted;
  }
} // namespace

namespace scan_planner
{

  // init：SCANReplanFSM的初始化入口，完成状态机的全部一次性启动工作。
  // 步骤：1.重置全部内部状态标志与计数器；2.从参数服务器读取FSM相关参数
  //   （导航模式、重规划阈值、规划地平线、安全重规划各类超时/冷却时间、
  //   自身膨胀体几何参数等）；3.若为PRESET_TARGET模式则加载keypoint.yaml
  //   预设航点参数；4.校验安全重规划参数的合法性，非法则致命退出；
  //   5.创建可视化模块与SCANPlannerManager并完成规划模块初始化；
  //   6.注册主控制定时器execFSMCallback（100Hz）与碰撞检测定时器
  //   checkCollisionCallback（20Hz）；7.订阅里程计/执行冻结/重置/接管同步等话题，
  //   发布B样条轨迹/数据展示/自身膨胀可视化话题；8.按navi_mode_分支：
  //   手动目标模式订阅RViz目标点，预设航点模式等待里程计就绪后直接规划第一个
  //   航点，参考路径模式订阅外部初始路径话题。
  void SCANReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = ros::Time::now();

    /*  fsm param  */
    nh.param("fsm/navi_mode", navi_mode_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizon_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/max_replan_fail_count", max_replan_fail_count_, 12);
    nh.param("fsm/safety_immediate_replan_sec", safety_immediate_replan_sec_, 1.0);
    nh.param("fsm/safety_direct_replan_sec", safety_direct_replan_sec_, 3.0);
    nh.param("fsm/safety_replan_cooldown_sec", safety_replan_cooldown_sec_, 0.20);
    nh.param("fsm/nominal_replan_period_sec", nominal_replan_period_sec_, 0.20);
    nh.param("fsm/min_replan_progress_m", min_replan_progress_m_, 0.05);
    nh.param("fsm/replan_retry_interval_sec", replan_retry_interval_sec_, 0.10);
    nh.param("fsm/replan_lead_time_sec", replan_lead_time_sec_, 0.40);
    nh.param("fsm/emergency_retry_interval_sec", emergency_retry_interval_sec_, 0.50);
    nh.param("grid_map/obstacles_inflation_z_up", self_inflation_z_up_, 0.0);
    nh.param("grid_map/obstacles_inflation_z_down", self_inflation_z_down_, 0.0);
    nh.param("grid_map/double_cylinder_radius", self_double_cylinder_radius_, 0.0);
    nh.param("grid_map/double_cylinder_offset", self_double_cylinder_offset_, 0.0);
    nh.param("grid_map/body_height", body_height_, 0.30);
    nh.param("grid_map/frame_id", self_inflation_frame_id_, std::string("world"));

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const std::string keypoints_yaml = "\"$(rospack find scan_planner)/../../../tools/keypoint.yaml\"";
      const std::string load_keypoints_cmd =
          "rosparam load " + keypoints_yaml + " " + shellQuote(nh.getNamespace());
      if (std::system(load_keypoints_cmd.c_str()) != 0)
      {
        ROS_ERROR("[SCANReplanFSM] Failed to load keypoints_yaml: tools/keypoint.yaml");
        ros::shutdown();
        return;
      }

      nh.param("fsm/waypoint_num", waypoint_num_, -1);

      if (waypoint_num_ <= 0)
      {
        ROS_ERROR("[SCANReplanFSM] navi_mode=2 requires keypoints_yaml with fsm/waypoint_num and fsm/waypoint{i}_{x,y,z}.");
        ros::shutdown();
        return;
      }
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        nh.param("fsm/waypoint" + to_string(i) + "_x", preset_waypoints_[i](0), -1.0);
        nh.param("fsm/waypoint" + to_string(i) + "_y", preset_waypoints_[i](1), -1.0);
        nh.param("fsm/waypoint" + to_string(i) + "_z", preset_waypoints_[i](2), -1.0);
      }
    }

    // Validate safety replan parameters.
    if (safety_immediate_replan_sec_ <= 0.0 ||
        safety_direct_replan_sec_ < safety_immediate_replan_sec_ ||
        safety_replan_cooldown_sec_ < 0.0 ||
        nominal_replan_period_sec_ <= 0.0 || min_replan_progress_m_ < 0.0 ||
        replan_retry_interval_sec_ <= 0.0 || replan_lead_time_sec_ <= 0.0)
    {
      ROS_FATAL("[SCANReplanFSM] Invalid safety replan params: "
                "immediate=%.3f direct=%.3f cooldown=%.3f",
                safety_immediate_replan_sec_,
                safety_direct_replan_sec_,
                safety_replan_cooldown_sec_);
      ros::shutdown();
      return;
    }

    ROS_INFO("SCAN_BODY_MODEL radius=%.3f offset=%.3f "
             "body_height=%.3f z_up=%.3f z_down=%.3f",
        self_double_cylinder_radius_, self_double_cylinder_offset_,
        body_height_, self_inflation_z_up_, self_inflation_z_down_);

    ROS_INFO("SCAN_REPLAN_CONFIG max_fail_count=%d "
             "immediate_sec=%.3f direct_sec=%.3f cooldown_sec=%.3f "
             "planning_horizon=%.3f emergency_retry_sec=%.3f",
        max_replan_fail_count_, safety_immediate_replan_sec_,
        safety_direct_replan_sec_, safety_replan_cooldown_sec_,
        planning_horizon_, emergency_retry_interval_sec_);

    // SCAN_FSM_STARTED marks every (re)start of this node with its PID.
    // If roslaunch respawn ever restarts a dead scan_planner_dmq_node, a new
    // PID appears here; if the PID stays the same while SCAN_FSM_HEARTBEAT
    // stops advancing, the process is hung rather than dead.
    ROS_WARN("SCAN_FSM_STARTED pid=%d navi_mode=%d", static_cast<int>(getpid()), navi_mode_);

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &SCANReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &SCANReplanFSM::checkCollisionCallback, this);

    std::string body_pose_topic;
    ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
    odom_sub_ = nh.subscribe(body_pose_topic, 1, &SCANReplanFSM::odometryCallback, this);
    go2_execution_frozen_sub_ = nh.subscribe("/planning/go2_execution_frozen", 10, &SCANReplanFSM::go2ExecutionFrozenCallback, this);
    reset_sub_ = nh.subscribe("/native_scan/reset", 1, &SCANReplanFSM::resetCallback, this);
    takeover_sync_sub_ = nh.subscribe("/native_scan/takeover_sync", 1,
        &SCANReplanFSM::takeoverSyncCallback, this);

    bspline_pub_ = nh.advertise<scan_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<scan_planner::DataDisp>("/planning/data_display", 100);
    self_inflation_pub_ = nh.advertise<visualization_msgs::Marker>("self_inflation", 10, true);

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &SCANReplanFSM::rvizGoalCallback, this);
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      planGlobalTrajbyGivenWps();
    }
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      path_sub_ = nh.subscribe("/initial_path", 1, &SCANReplanFSM::pathCallback, this);
    else
      ROS_ERROR("Wrong navi_mode_ value! navi_mode_=%d", navi_mode_);
  }

  // planGlobalTrajbyGivenWps：PRESET_TARGET模式下按预设航点列表规划第一段
  // 全局轨迹。步骤：1.在可视化中显示全部预设航点；2.以里程计当前位置作为
  // 起点，重置航点索引为0并置位trigger_；3.调用planNextWaypoint()规划到
  // 第一个航点的全局轨迹，成功则切换到GEN_NEW_TRAJ状态，失败则报错。
  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory to first preset waypoint!");
    }
  }

  // rvizGoalCallback：接收RViz "2D Nav Goal"发布的目标点。输入msg为目标位姿
  // （仅使用XY，高度沿用初次里程计高度）。若尚未记录过初始高度则忽略该目标，
  // 否则将其包装为单点Path并转发给waypointCallback统一处理。
  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      ROS_WARN("[SCANReplanFSM] Ignore RViz goal before receiving initial body pose.");
      return;
    }

    nav_msgs::PathPtr path(new nav_msgs::Path);
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  // waypointCallback：MANUAL_TARGET模式下接收单点目标路径msg并规划全局轨迹。
  // 步骤：1.校验消息非空且目标高度合法；2.以里程计当前位置为起点，取消息
  // 中第一个点（高度用rviz_goal_height_覆盖）作为终点end_pt_，调用
  // planGlobalTraj生成min-snap多项式全局轨迹；3.若终点被占据则调用
  // adjustGlobalTargetIfOccupied回退到轨迹上最近的空闲点；4.成功后置位
  // 目标标志并根据当前FSM状态触发GEN_NEW_TRAJ重规划，同时可视化全局路径与
  // 目标点。
  void SCANReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      ROS_WARN_THROTTLE(1.0, "[waypointCallback] Empty waypoint message, ignore.");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    ROS_DEBUG("SCAN_RVIZ_TARGET_TRIGGERED");
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(GEN_NEW_TRAJ, "NEW_TARGET");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  // planGlobalTrajByWaypoints：按任意多点航点列表waypoints规划一条造访全部航点的
  // 全局轨迹（主要用于REFERENCE_PATH模式的外部路径）。步骤：1.校验非空；
  // 2.取最后一个点为终点end_pt_并可视化全部航点；3.调用
  // planner_manager_->planGlobalTrajWaypoints生成沿全部航点序列的分段min-snap
  // 轨迹；4.若终点被占据则回退到空闲点；5.置位目标标志并可视化全局轨迹与
  // 终点。返回是否规划成功。
  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())
    {
      ROS_WARN("[planGlobalTrajByWaypoints] No waypoint to plan.");
      return false;
    }

    end_pt_ = waypoints.back();

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_,
        odom_vel_,
        Eigen::Vector3d::Zero(),
        waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      ROS_ERROR("Unable to generate global trajectory from waypoints!");
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  // planNextWaypoint：PRESET_TARGET模式下规划到当前current_wp_指向的下一个预设
  // 航点。步骤：1.校验航点索引合法；2.取该航点为终点，调用
  // setStartStateFromOdomOrCurrentTraj确定连续的起始状态；3.规划全局轨迹并在
  // 必要时回退到空闲终点；4.成功后置位目标标志并可视化。返回是否规划成功。
  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      ROS_WARN("[navi_mode=%d] No active waypoint to plan.", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      ROS_ERROR("[navi_mode=%d] Unable to generate trajectory to waypoint %d.", navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    ROS_INFO("[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f].",
             navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  // isWaypointSequenceMode：判断当前是否为PRESET_TARGET预设多航点序列模式。
  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  // adjustGlobalTargetIfOccupied：检查全局轨迹的终点是否被地图占据，若被占据
  // 则沿轨迹向后回退搜索一个空闲点并修改end_pt_与全局轨迹时长。步骤：
  // 1.若无地图或时长过短直接认为合法；2.先检查终点本身是否空闲，空闲则直接
  // 返回true；3.否则从轨迹末端向前逐样时间采样查找第一个空闲点，找到则更新
  // end_pt_为该点、缩短全局轨迹时长global_duration_并同步修正
  // last_progress_time_；4.若整条轨迹都无空闲点则返回false。
  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        ROS_WARN("[global target] Target [%.2f, %.2f, %.2f] is occupied; use backward collision-free point [%.2f, %.2f, %.2f].",
                 raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    ROS_ERROR("[global target] Target is occupied, and no collision-free point was found along the global trajectory.");
    return false;
  }

  // escapeInflatedStart：GEN_NEW_TRAJ每次重试都固定用里程计当前位置作为起点，
  // 若机体真实位置贴近障碍以致该点本身被判定为膨胀障碍，则地图与机体位置在
  // 重试之间都不会变化，导致每次都在同一个卡死点上重复失败（对应日志中的
  // "the robot is in an inflated obstacle."/"Ran out of pool"）。这里沿机体
  // 当前朝向的反方向（来向，通常是刚驶来的安全区）以地图分辨率为步长逐步
  // 回退搜索，找到最近的空闲点后仅作为本次规划的虚拟起点使用——不修改里程计
  // 记录的真实机体位置，机体后续会沿新轨迹自然重新贴合真实位置。
  bool SCANReplanFSM::escapeInflatedStart(Eigen::Vector3d &start_pt) const
  {
    auto map = planner_manager_->grid_map_;
    if (!map)
      return false;

    const double yaw = getOdomYaw();
    if (map->getInflateOccupancy(start_pt, yaw) == 0)
      return true;

    const Eigen::Vector2d retreat_dir(-std::cos(yaw), -std::sin(yaw));
    const double step = std::max(1e-3, map->getResolution());
    const double max_retreat =
        self_double_cylinder_radius_ + self_double_cylinder_offset_ + 0.5;

    for (double d = step; d <= max_retreat; d += step)
    {
      Eigen::Vector3d candidate = start_pt;
      candidate(0) += retreat_dir(0) * d;
      candidate(1) += retreat_dir(1) * d;

      if (map->getInflateOccupancy(candidate, yaw) == 0)
      {
        ROS_WARN("SCAN_START_ESCAPE origin=(%.2f,%.2f) escaped=(%.2f,%.2f) retreat=%.2f",
            start_pt(0), start_pt(1), candidate(0), candidate(1), d);
        start_pt = candidate;
        return true;
      }
    }

    return false;
  }

  // pathCallback：REFERENCE_PATH模式下接收外部（如MQTT桥接）下发的参考路径msg并
  // 重新启动一次完整规划会话。步骤：1.校验非空；2.逐点清洗：剔除非有限点、
  // 与里程计当前位置或上一保留点过于接近的重复点（避免零长度段），Z高度统一
  // 用里程计当前高度（平面导航）；3.若清洗后无可用点则报错返回；4.调用
  // planGlobalTrajByWaypoints重新规划全局轨迹；5.成功则强制清零全部重规划失败
  // 计数、应急停止标志、重试时间戳等一切旧会话状态（避免旧状态污染新任务），
  // 并重置局部轨迹数据后强制进入GEN_NEW_TRAJ重新规划；失败则报错。
  void SCANReplanFSM::pathCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      ROS_WARN_THROTTLE(1.0, "[pathCallback] Received empty /initial_path, ignore.");
      return;
    }

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());
    constexpr double kDuplicatePointDistanceM = 0.01;
    std::size_t removed_point_count = 0;

    for (const auto& pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      // Reference-path navigation is planar. Keep every waypoint on the
      // robot's current odometry height; do not add body height a second time.
      wp(2) = odom_pos_(2);
      if (!wp.allFinite())
      {
        ++removed_point_count;
        continue;
      }
      const bool duplicates_start =
          waypoints.empty() &&
          (wp - odom_pos_).head<2>().norm() < kDuplicatePointDistanceM;
      const bool duplicates_previous =
          !waypoints.empty() &&
          (wp - waypoints.back()).head<2>().norm() <
              kDuplicatePointDistanceM;
      if (duplicates_start || duplicates_previous)
      {
        ++removed_point_count;
        continue;
      }
      waypoints.push_back(wp);
    }

    if (waypoints.empty())
    {
      ROS_ERROR("[pathCallback] No usable waypoint after removing %zu "
                "duplicate/invalid points.", removed_point_count);
      return;
    }

    trigger_ = true;
    ROS_INFO("SCAN_PATH_SANITIZED received=%zu kept=%zu removed=%zu",
             msg->poses.size(), waypoints.size(), removed_point_count);
    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      // Force a fresh planning session regardless of current state.
      // Old FSM state, failure counters, and emergency flags must not
      // carry over from a previous task or a previous LOCAL_AVOID exit.
      replan_fail_count_ = 0;
      continuation_failure_count_ = 0;
      first_replan_failure_time_ = ros::Time(0);
      last_replan_attempt_time_ = ros::Time(0);
      last_successful_replan_time_ = ros::Time(0);
      last_nominal_replan_attempt_time_ = ros::Time(0);
      last_replan_robot_position_ = odom_pos_;
      next_target_retry_time_ = ros::Time(0);
      initial_plan_attempt_count_ = 0;
      emergency_stop_active_ = false;

      flag_escape_emergency_ = true;
      need_hover_stop_ = false;

      planner_manager_->local_data_.reset();

      changeFSMExecState(
          GEN_NEW_TRAJ,
          "NEW_REFERENCE_PATH");

      ROS_INFO("==========================================\n");
    }
    else
    {
      ROS_ERROR("❌ Unable to generate global trajectory!");
    }
  }

  // resetCallback：接收外部重置信号，将FSM全部状态恢复到初始空闲状态并返回
  // WAIT_TARGET。清空目标/航点/失败计数器/接管相关时间戳，并将起始/局部/终点
  // 状态均重置为里程计当前值，同时重置规划器内部的全局/局部轨迹数据。
  void SCANReplanFSM::resetCallback(const std_msgs::EmptyConstPtr &)
  {
    trigger_ = false;
    have_target_ = false;
    have_new_target_ = false;

    active_waypoints_.clear();
    current_wp_ = 0;

    replan_fail_count_ = 0;
    continuation_failure_count_ = 0;
    first_replan_failure_time_ = ros::Time(0);
    continuously_called_times_ = 0;
    initial_plan_attempt_count_ = 0;
    emergency_stop_active_ = false;

    need_hover_stop_ = false;
    flag_escape_emergency_ = true;

    go2_execution_frozen_ = false;

    last_replan_attempt_time_ = ros::Time(0);
    last_successful_replan_time_ = ros::Time(0);
    last_nominal_replan_attempt_time_ = ros::Time(0);
    last_replan_robot_position_ = odom_pos_;
    planning_in_progress_ = false;
    takeover_sync_pending_ = false;
    force_takeover_poly_init_ = false;
    last_freeze_update_time_ = ros::Time::now();
    next_emergency_retry_time_ = ros::Time(0);
    next_target_retry_time_ = ros::Time(0);
    last_safety_replan_time_ = ros::Time(0);

    start_pt_ = odom_pos_;
    start_vel_.setZero();
    start_acc_.setZero();

    local_target_pt_ = odom_pos_;
    local_target_vel_.setZero();

    end_pt_ = odom_pos_;
    end_vel_.setZero();

    exec_state_ = WAIT_TARGET;

    planner_manager_->local_data_.reset();
    planner_manager_->global_data_.reset();

    ROS_WARN("SCAN_FSM_RESET state=WAIT_TARGET");
  }

  // odometryCallback：接收机体位姿（body_pose）并更新里程计位置/速度/姿态。
  // 若为MANUAL_TARGET模式且尚未记录初始高度，则以首次收到的Z作为RViz目标高度
  // 基准。最后置位have_odom_并更新里程计时间戳，发布自身膨胀体可视化。
  void SCANReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      ROS_INFO("[SCANReplanFSM] Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    last_odom_time_ = ros::Time::now();
    publishSelfInflationMarker();
  }

  // go2ExecutionFrozenCallback：接收Go2执行层的“执行冻结”信号（接管或避让
  // 期间暂停推进时间轴），仅缓存标志供updateLocalTrajTimeFreeze使用。
  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::BoolConstPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  // takeoverSyncCallback：接收Native SCAN接管同步信号（从手动/其他控制模式
  // 切换回到本规划器控制）。仅在REFERENCE_PATH模式下生效。若里程计或目标尚未
  // 就绪则置位takeover_sync_pending_延迟处理。正常流程：保留global_data_/
  // end_pt_与MQTT参考路径，仅重置尚未执行过的局部B样条轨迹；以里程计当前
  // 位置/速度作为新起点，清零全部重规划失败计数与时间戳，置位
  // force_takeover_poly_init_要求下一次重规划强制多项式初始化，并立即触发
  // GEN_NEW_TRAJ重规划。
  void SCANReplanFSM::takeoverSyncCallback(const std_msgs::EmptyConstPtr &)
  {
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH)
      return;
    if (!have_odom_ || !have_target_ || !planner_manager_)
    {
      takeover_sync_pending_ = true;
      ROS_WARN("SCAN_TAKEOVER_SYNC_DEFERRED have_odom=%d have_target=%d",
          have_odom_ ? 1 : 0, have_target_ ? 1 : 0);
      return;
    }

    // Preserve global_data_, end_pt_ and the MQTT reference route.  Only the
    // locally prewarmed, never-executed B-spline is invalid at handoff.
    planner_manager_->local_data_.reset();
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();
    continuation_failure_count_ = 0;
    initial_plan_attempt_count_ = 0;
    replan_fail_count_ = 0;
    first_replan_failure_time_ = ros::Time(0);
    last_replan_attempt_time_ = ros::Time(0);
    last_successful_replan_time_ = ros::Time(0);
    last_nominal_replan_attempt_time_ = ros::Time(0);
    last_replan_robot_position_ = odom_pos_;
    next_target_retry_time_ = ros::Time(0);
    planning_in_progress_ = false;
    emergency_stop_active_ = false;
    takeover_sync_pending_ = false;
    force_takeover_poly_init_ = true;
    changeFSMExecState(GEN_NEW_TRAJ, "TAKEOVER_SYNC");
    ROS_INFO("SCAN_TAKEOVER_LOCAL_RESET start=(%.3f,%.3f) velocity=(%.3f,%.3f) global_target=(%.3f,%.3f)",
        start_pt_(0), start_pt_(1), start_vel_(0), start_vel_(1),
        end_pt_(0), end_pt_(1));
  }

  // updateLocalTrajTimeFreeze：当Go2执行层处于冻结状态时，推迟局部轨迹的起始
  // 时间start_time_，使得时间轴与实际已执行时长保持一致（避免冻结期间被
  // 误判为轨迹已过期）。每次调用根据上次更新时间计算dt，若dt异常（<=0或
  // >0.2s，可能因时钟跳变）则跳过本次更新。
  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const ros::Time now = ros::Time::now();
    double dt = (now - last_freeze_update_time_).toSec();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.toSec() > 1e-5)
      info->start_time_ += ros::Duration(dt);
  }

  // getOdomYaw：从里程计姿态四元数提取机体前方方向并返回对应的yaw角（若前方在
  // XY平面的投影接近零则返回0）。
  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  // estimateYawFromSegment：根据两点from→to的连线方向估计朝向角，若两点过于
  // 接近则回退使用当前里程计朝向。
  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  // estimateTrajectoryYaw：对给定轨迹trajectory在time_sec时刻求导得到速度向量，
  // 以速度方向作为该时刻的朝向估计；若速度接近零则回退使用当前里程计朝向。
  double SCANReplanFSM::estimateTrajectoryYaw(
      UniformBspline &trajectory, double time_sec) const
  {
    UniformBspline velocity_trajectory = trajectory.getDerivative();
    const Eigen::Vector3d velocity =
        velocity_trajectory.evaluateDeBoorT(time_sec);
    return velocity.head<2>().squaredNorm() < 1e-8
        ? getOdomYaw()
        : std::atan2(velocity(1), velocity(0));
  }

  // normalizeAngle：将角度归一化到[-pi, pi]区间。
  double SCANReplanFSM::normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  // publishSelfInflationMarker：发布机体自身膨胀体（双圆柱模型，前后各一个
  // 圆柱）的可视化标记，用于调试观察碰撞检测使用的安全体积。步骤：1.根据
  // 自身膨胀各项参数设置圆柱半径与高度；2.以里程计位置为中心并沿垂直方向
  // 按z_up/z_down偏移；3.根据当前朝向计算前/后两个圆柱中心并分别发布。
  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = ros::Duration(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_.publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_.publish(marker);
  }

  // changeFSMExecState：切换状态机执行状态并记录日志。若新状态与当前相同则累加
  // 连续调用计数（用于检测卡死在同一状态），否则重置计数为1；仅在状态
  // 确实发生转换时打印日志（避免同状态重复调用刷屏）。
  void SCANReplanFSM::changeFSMExecState(
      FSM_EXEC_STATE new_state,
      string pos_call)
  {
    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static const char* state_str[] = {
        "INIT",
        "WAIT_TARGET",
        "GEN_NEW_TRAJ",
        "EXEC_TRAJ",
        "EMERGENCY_STOP"};

    const int previous_state =
        static_cast<int>(exec_state_);

    const int next_state =
        static_cast<int>(new_state);

    // 同一状态的重复调用仍保留计数，但不刷屏。
    if (new_state != exec_state_)
    {
      ROS_INFO(
          "SCAN_FSM prev=%s next=%s caller=%s",
          state_str[previous_state],
          state_str[next_state],
          pos_call.c_str());
    }

    exec_state_ = new_state;
  }

  // timesOfConsecutiveStateCalls：返回当前状态连续被调用的次数与当前状态本身。
  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  // printFSMExecState：输出当前FSM状态名称的调试日志。
  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[6] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    ROS_DEBUG("[FSM]: state: %s", state_str[int(exec_state_)].c_str());
  }

  // execFSMCallback：主状态机循环，以100Hz固定周期驱动整个重规划流程。
  // 整体流程：1.先推进冻结补偿（updateLocalTrajTimeFreeze）；2.若接管同步处于
  // 延迟状态且里程计/目标已就绪，补发接管同步回调；3.按固定频率输出心跳日志
  // （用于判断进程是否卸死）；4.根据当前exec_state_分支处理：
  //   - INIT：等待里程计与触发信号就绪后进入WAIT_TARGET；
  //   - WAIT_TARGET：等待收到目标后进入GEN_NEW_TRAJ；
  //   - GEN_NEW_TRAJ：以里程计当前位置/速度为新起点，交替使用确定性/
  //     随机多项式初始化，并根据连续失败次数逐步回退目标距离上限，调用
  //     callReboundReplan尝试重规划；成功则进入EXEC_TRAJ并清零各项计数；
  //     目标不可用则仅设置重试定时器不累加失败计数；优化失败则回滚局部轨迹并
  //     累加失败计数；
  //   - EXEC_TRAJ：先处理预设多航点到达切换；然后根据周期性重规划条件（距
  //     上次重规划时间/移动距离/剩余时长接近前瞬时间）判断是否需要滚动重规划
  //     （planFromCurrentTraj），失败但旧轨迹仍安全且未到期则继续使用旧轨迹，
  //     否则强制回到GEN_NEW_TRAJ重新规划；若已接近终点则进入WAIT_TARGET；若
  //     轨迹已过期则切换到下一个预设航点或强制重新规划；
  //   - EMERGENCY_STOP：刚进入时发布一次静止轨迹（仅一次，不重复生成）；
  //     后续周期在机体静止后尝试重新规划或退回WAIT_TARGET。
  // 5.最后调用finishProcess检查是否需要进入应急停止，并发布数据展示。
  void SCANReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    updateLocalTrajTimeFreeze();

    if (takeover_sync_pending_ && have_odom_ && have_target_ &&
        planner_manager_ && navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      // Deferred synchronization is completed as soon as both odometry and
      // the retained reference-path target are available.
      takeoverSyncCallback(std_msgs::EmptyConstPtr());
    }

    static const char* state_names[] = {
        "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ",
        "EXEC_TRAJ", "EMERGENCY_STOP"};
    // SCAN_FSM_HEARTBEAT proves this process's exec timer is still firing.
    // Previously this line was ROS_DEBUG-only and therefore never appeared
    // in the production log (default logger level is INFO), so a silent
    // process death/hang between tasks was indistinguishable from a live
    // process simply waiting for a new target. Kept at a low, throttled
    // rate so it stays cheap on the 100Hz timer.
    const double odom_age = have_odom_ ? (ros::Time::now() - last_odom_time_).toSec() : -1.0;
    ROS_INFO_THROTTLE(2.0,
        "SCAN_FSM_HEARTBEAT pid=%d state=%s trigger=%d have_odom=%d odom_age=%.2f target=%d",
        static_cast<int>(getpid()),
        state_names[int(exec_state_)],
        static_cast<int>(trigger_),
        static_cast<int>(have_odom_),
        odom_age,
        static_cast<int>(have_target_));

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      // Rate-limit retries when no target is available.
      if (!next_target_retry_time_.isZero() &&
          ros::Time::now() < next_target_retry_time_)
      {
        break;
      }

      // Rate-limit retries: wait at least 100ms between actual planning
      // attempts so the map has time to update.
      if (!replanRetryReady(0.10))
        break;

      // GEN_NEW_TRAJ explicitly abandons the old local trajectory and starts
      // from real odometry. Continuation is exclusively an EXEC_TRAJ concern.
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      // If the robot's true position is itself classified as an inflated
      // obstacle (body stopped too close to something), odom_pos_ and the
      // map stay identical across retries, so every attempt below would
      // fail on the exact same dead point. Retreat the *planning* start
      // point only (see escapeInflatedStart) so the optimizer has a
      // reachable point to plan from; the real robot pose is untouched.
      escapeInflatedStart(start_pt_);

      // Use independent attempt counter — NOT timesOfConsecutiveStateCalls().
      const int attempt = initial_plan_attempt_count_;

      // Alternate between deterministic and random/A* initialization.
      const bool takeover_first_plan = force_takeover_poly_init_;
      const bool random_init = takeover_first_plan ? false : (attempt % 2) == 1;

      // Progressive target distance backoff.
      const int backoff_level = std::min(attempt / 2, 4);
      const double target_cap =
          std::max(1.2, planning_horizon_ - 0.5 * backoff_level);

      // Preserve previous trajectory on failure.
      const LocalTrajData previous_local =
          planner_manager_->local_data_;

      const ReplanResult result =
          callReboundReplan(true, random_init, target_cap);

      if (result == ReplanResult::SUCCESS)
      {
        force_takeover_poly_init_ = false;
        initial_plan_attempt_count_ = 0;
        continuation_failure_count_ = 0;
        replan_fail_count_ = 0;
        first_replan_failure_time_ = ros::Time(0);
        emergency_stop_active_ = false;
        next_target_retry_time_ = ros::Time(0);
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else if (result == ReplanResult::TARGET_UNAVAILABLE)
      {
        // No target found — optimizer never ran. Do NOT increment
        // attempt counter or failure count. Retry after a delay.
        planner_manager_->local_data_ = previous_local;
        next_target_retry_time_ =
            ros::Time::now() + ros::Duration(replan_retry_interval_sec_);
      }
      else  // OPTIMIZATION_FAILED
      {
        planner_manager_->local_data_ = previous_local;
        ++initial_plan_attempt_count_;

        if (first_replan_failure_time_.isZero())
          first_replan_failure_time_ = ros::Time::now();

        ++replan_fail_count_;
        ROS_WARN_THROTTLE(
            1.0,
            "SCAN_REPLAN_FAILED count=%d attempt=%d",
            replan_fail_count_, attempt);
      }
      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      const double remaining_time = std::max(0.0, info->duration_ - t_cur);
      const double elapsed_since_replan = last_successful_replan_time_.isZero()
          ? std::numeric_limits<double>::infinity()
          : (time_now - last_successful_replan_time_).toSec();
      const double moved_since_replan =
          (odom_pos_ - last_replan_robot_position_).head<2>().norm();
      const bool periodic_due = elapsed_since_replan >= nominal_replan_period_sec_ &&
          moved_since_replan >= min_replan_progress_m_;
      const bool trajectory_ending = remaining_time <= replan_lead_time_sec_;
      const bool retry_ready = last_nominal_replan_attempt_time_.isZero() ||
          (time_now - last_nominal_replan_attempt_time_).toSec() >= replan_retry_interval_sec_;

      // Waypoint mode (navi_mode=2) uses pure path tracking: the initial
      // B-spline is executed to completion without rolling replanning.
      // A fresh trajectory is generated from odom only when the current
      // one expires (see TRAJECTORY_EXPIRED below).  Safety replanning
      // via checkCollisionCallback remains fully active.
      // All other modes (manual target, reference path) keep the original
      // rolling replanning behaviour.
      if (!isWaypointSequenceMode() &&
          (periodic_due || trajectory_ending) && retry_ready &&
          (end_pt_ - odom_pos_).norm() > no_replan_thresh_ &&
          !planning_in_progress_)
      {
        planning_in_progress_ = true;
        last_nominal_replan_attempt_time_ = time_now;
        const int old_traj_id = info->traj_id_;
        const ReplanResult result = planFromCurrentTraj();
        planning_in_progress_ = false;
        if (result == ReplanResult::SUCCESS)
        {
          last_successful_replan_time_ = ros::Time::now();
          last_replan_robot_position_ = odom_pos_;
          ROS_DEBUG("SCAN_NOMINAL_REPLAN old_traj_id=%d new_traj_id=%d elapsed=%.3f robot_progress=%.3f remaining_time=%.3f result=SUCCESS",
              old_traj_id, planner_manager_->local_data_.traj_id_, elapsed_since_replan,
              moved_since_replan, remaining_time);
          break;
        }
        double collision_time = std::numeric_limits<double>::infinity();
        const bool old_safe = localTrajectoryIsSafe(collision_time);
        const bool old_trajectory_usable = old_safe &&
            remaining_time > replan_lead_time_sec_;
        if (old_trajectory_usable)
        {
          ROS_WARN_THROTTLE(1.0,
              "SCAN_REPLAN_KEEP_OLD traj_id=%d remaining_time=%.3f collision_time=%.3f",
              old_traj_id, remaining_time, collision_time);
          break;
        }

        initial_plan_attempt_count_ = 0;
        replan_fail_count_ = 0;
        first_replan_failure_time_ = ros::Time(0);
        last_replan_attempt_time_ = ros::Time(0);
        next_target_retry_time_ = ros::Time(0);
        changeFSMExecState(GEN_NEW_TRAJ, "OLD_TRAJ_UNUSABLE");
        ROS_WARN("SCAN_FRESH_PLAN_REQUIRED old_traj_id=%d remaining_time=%.3f old_safe=%d collision_time=%.3f",
            old_traj_id, remaining_time, old_safe ? 1 : 0, collision_time);
        break;
      }

      const double dist_to_goal = (end_pt_ - odom_pos_).norm();
      if (dist_to_goal <= no_replan_thresh_)
      {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "GLOBAL_GOAL_REACHED");
        return;
      }

      if (t_cur > info->duration_ - 1e-2)
      {
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        initial_plan_attempt_count_ = 0;
        replan_fail_count_ = 0;
        first_replan_failure_time_ = ros::Time(0);
        last_replan_attempt_time_ = ros::Time(0);
        next_target_retry_time_ = ros::Time(0);
        changeFSMExecState(GEN_NEW_TRAJ, "TRAJECTORY_EXPIRED");
        ROS_WARN("SCAN_TRAJECTORY_EXPIRED traj_id=%d dist_to_goal=%.3f action=FRESH_PLAN_FROM_ODOM",
            info->traj_id_, dist_to_goal);
        return;
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_)
      {
        // Only publish the emergency stop spline once per continuous
        // failure cycle — do not regenerate a new trajectory_id each time.
        if (!emergency_stop_active_)
        {
          callEmergencyStop(odom_pos_);
          emergency_stop_active_ = true;
        }
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          // Rate-limit emergency recovery: wait at least
          // emergency_retry_interval_sec_ between GEN_NEW_TRAJ attempts.
          if (ros::Time::now() >= next_emergency_retry_time_)
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          ROS_INFO("Exiting EMERGENCY_STOP. Switching to WAIT_TARGET. Need a new target point.");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  // finishProcess：在每个execFSMCallback周期末尾检查是否需要进入应急停止。
  // 当重规划连续失败次数达到上限且持续时长超过1秒时触发。若当前为REFERENCE_PATH
  // 模式且仍有目标（Native接管拥有活跃MQTT任务）则保留目标不需要悬停等待新
  // 目标，否则需要悬停。清零失败计数，置位应急重试定时器并切换到EMERGENCY_STOP。
  void SCANReplanFSM::finishProcess()
  {
    const ros::Time now = ros::Time::now();
    const bool exceeded_count =
        replan_fail_count_ >= max_replan_fail_count_;
    const bool exceeded_duration =
        !first_replan_failure_time_.isZero() &&
        (now - first_replan_failure_time_).toSec() >= 1.0;

    if (exceeded_count && exceeded_duration)
    {
      const bool keep_reference_path =
          navi_mode_ == NAVI_MODE::REFERENCE_PATH && have_target_;
      ROS_WARN("Replan failed %d times over %.1fs. Emergency stop; "
               "keep_reference_path=%d.",
               replan_fail_count_,
               (now - first_replan_failure_time_).toSec(),
               keep_reference_path ? 1 : 0);
      replan_fail_count_ = 0;
      first_replan_failure_time_ = ros::Time(0);
      // Native takeover owns an active MQTT task.  Do not discard that path
      // and wait for a second /initial_path message after a temporary local
      // planning failure; stop first, then retry against the updated map.
      need_hover_stop_ = !keep_reference_path;
      flag_escape_emergency_ = true;
      emergency_stop_active_ = false;
      next_emergency_retry_time_ =
          ros::Time::now() + ros::Duration(emergency_retry_interval_sec_);
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  // replanRetryReady：频率限制工具函数。若距上次重规划尝试时间未达到interval_sec
  // 则返回false拒绝本次尝试，否则更新时间戳并返回true允许本次尝试。
  bool SCANReplanFSM::replanRetryReady(
      double interval_sec)
  {
    const ros::Time now = ros::Time::now();

    if (!last_replan_attempt_time_.isZero() &&
        (now - last_replan_attempt_time_).toSec() <
            interval_sec)
    {
      return false;
    }

    last_replan_attempt_time_ = now;
    return true;
  }

  // planFromCurrentTraj：EXEC_TRAJ阶段的滚动重规划入口，从当前正在执行的
  // 局部轨迹中推导新的起始状态并尝试重规划。步骤：1.取当前局部轨迹在现在时刻
  // t_cur处的速度/加速度作为新起始状态（位置直接用里程计）；2.若起始速度与
  // 目标方向反向则清零（避免倒退）；3.非REFERENCE_PATH模式下重新规划全局轨迹
  // （参考路径模式下global_data_是剩余的MQTT路径，不能用直线重新生成而丢失
  // 进度），并在必要时回退终点；4.优先以非多项式初始化方式延续当前B样条（保
  // 留已选定的绕障方向）调用callReboundReplan；成功则清零连续失败计数并返回；
  // 5.若失败，先回滚局部轨迹并累加连续失败计数，若旧轨迹仍安全且剩余时长充足且
  // 失败次数未达上限则允许本次保留失败结果不强行重初始化；6.否则回退到强制
  // 确定性多项式初始化再试一次；若仍失败则回滚局部轨迹并返回失败结果。
  SCANReplanFSM::ReplanResult SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    start_pt_ = odom_pos_;
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    // In reference-path mode global_data_ is the remaining MQTT route.  Do
    // not replace it with a straight odom-to-goal line during a safety
    // replan: doing so loses route progress and can make the next escape
    // target jump behind the robot.
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH &&
        !planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      ROS_ERROR("[navi_mode=%d] Unable to refresh global trajectory from odom to current target.", navi_mode_);
      return ReplanResult::OPTIMIZATION_FAILED;
    }

    if (!adjustGlobalTargetIfOccupied())
      return ReplanResult::OPTIMIZATION_FAILED;

    // Preserve the previous trajectory so a failed replan does not
    // destroy a still-executable B-spline.
    const LocalTrajData previous_local =
        planner_manager_->local_data_;

    const double remaining_time = std::max(0.0, info->duration_ - t_cur);
    // Normal rolling replans extend the current B-spline first. This retains
    // its selected obstacle side instead of reconstructing a straight-line
    // polynomial seed at every 5 Hz refresh.
    ReplanResult result = callReboundReplan(false, false, planning_horizon_);
    if (result == ReplanResult::SUCCESS)
    {
      continuation_failure_count_ = 0;
      return result;
    }

    planner_manager_->local_data_ = previous_local;
    ++continuation_failure_count_;
    double old_collision_time = std::numeric_limits<double>::infinity();
    const bool old_trajectory_safe = localTrajectoryIsSafe(old_collision_time);
    if (old_trajectory_safe && remaining_time > replan_lead_time_sec_ &&
        continuation_failure_count_ < 3)
      return result;

    // A new deterministic seed is justified only once continuity is no
    // longer safe/useful: the old path is near its end, unsafe, or has failed
    // continuation repeatedly. Random initialization remains out of the
    // nominal rolling path.
    result = callReboundReplan(true, false, planning_horizon_);
    if (result == ReplanResult::SUCCESS)
    {
      continuation_failure_count_ = 0;
      return result;
    }

    planner_manager_->local_data_ = previous_local;
    return result;
  }

  // localTrajectoryIsSafe：测试当前局部轨迹前2/3时长段是否仍无碰撞（后1/3不
  // 检查是因为即将被重规划覆盖）。按固定步长采样位置并查询地图占据，一旦发现
  // 碰撞或无效值即记录碰撞时刻并返回false。输出ctollision_time_sec为碰撞
  // 发生的相对时刻（无碰撞则为无穷）。
  bool SCANReplanFSM::localTrajectoryIsSafe(
      double &collision_time_sec)
  {
    collision_time_sec = std::numeric_limits<double>::infinity();
    if (!planner_manager_ || !planner_manager_->grid_map_)
      return false;

    // UniformBspline's legacy evaluation API is not const-qualified even
    // though evaluation does not modify the trajectory.
    LocalTrajData &info = planner_manager_->local_data_;
    if (!std::isfinite(info.duration_) || info.duration_ <= 1e-5)
      return false;

    constexpr double kValidationStepSec = 0.01;
    const double validation_end = info.duration_ * 2.0 / 3.0;
    for (double t = 0.0; t <= validation_end + 1e-9;
         t += kValidationStepSec)
    {
      const double sample_t = std::min(t, validation_end);
      const Eigen::Vector3d pos =
          info.position_traj_.evaluateDeBoorT(sample_t);
      const Eigen::Vector3d next = info.position_traj_.evaluateDeBoorT(
          std::min(sample_t + kValidationStepSec, info.duration_));
      if (!pos.allFinite() || !next.allFinite() ||
          planner_manager_->grid_map_->getInflateOccupancy(
              pos, estimateYawFromSegment(pos, next)) != 0)
      {
        collision_time_sec = sample_t;
        return false;
      }
    }
    return true;
  }

  // setStartStateFromOdomOrCurrentTraj：确定下一次全局/局部规划的起始状态。
  // 位置总是用里程计当前值；若当前没有有效的局部轨迹或采样时刻raw_t_cur超出
  // 合法范围，则直接使用里程计速度并将加速度置零；否则从局部轨迹在t_cur处推
  // 导速度/加速度，若该速度与目标方向相反则清零（避免倒退）。
  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.toSec() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (ros::Time::now() - info->start_time_).toSec();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  // checkCollisionCallback：安全监控定时器（20Hz），实时检查当前执行中的局部
  // 轨迹是否将发生碰撞，并根据临近程度分级采取不同安全处置。步骤：1.仅在
  // EXEC_TRAJ状态且轨迹有效时才检测（避免递归地把静止应急轨迹又变成移动轨迹）；
  // 2.从当前时刻t_cur开始沿轨迹前2/3段按固定步长采样查询占据，一旦发现碰撞
  // 则计算碰撞前置时间collision_time_ahead并分类处理：
  //   - A类紧急碰撞（前置时间<=safety_immediate_replan_sec_）：不受冷却限制，
  //     立即调用planFromCurrentTraj直接重规划，成功则保持EXEC_TRAJ，失败则进入
  //     EMERGENCY_STOP；
  //   - 若处于安全重规划冷却期内，非A类碰撞直接抑制本次处理（避免与正常
  //     滚动重规划争抢）；
  //   - B类近碰撞（<=safety_direct_replan_sec_）：冷却结束后立即尝试从当前轨迹
  //     重规划，成功保持EXEC_TRAJ，失败进入EMERGENCY_STOP；
  //   - C类较远未来碰撞：尝试一次重规划但不强制切换状态，交由正常EXEC_TRAJ
  //     滚动更新循环稍后重试。
  void SCANReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    updateLocalTrajTimeFreeze();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    // Only an actively executing trajectory may trigger a safety replan.
    // In particular, never inspect the stationary emergency-stop spline and
    // recursively turn it into another moving trajectory.
    if (exec_state_ != EXEC_TRAJ || info->start_time_.toSec() < 1e-5 ||
        info->duration_ <= 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    const ros::Time now = ros::Time::now();
    double t_cur = (now - info->start_time_).toSec();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3)
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        const double collision_time_ahead = t - t_cur;

        // 上一次安全重规划距离当前的时间。
        // 从未执行过安全重规划时，视为冷却已经结束。
        const double cooldown_elapsed = 
            last_safety_replan_time_.isZero()
            ? std::numeric_limits<double>::infinity()
            : (now - last_safety_replan_time_).toSec();
        
        const bool cooldown_active = 
            cooldown_elapsed < safety_replan_cooldown_sec_;

        // ============================================================
        // A类：紧急碰撞
        //
        // 紧急碰撞不受冷却限制。因为此时继续等待的风险大于重复规划。
        // ============================================================

        if (collision_time_ahead <= safety_immediate_replan_sec_)
        {
          ROS_WARN(
              "SCAN_SAFETY_REPLAN "
              "urgency=EMERGENCY "
              "collision_time_ahead=%.2f "
              "action=DIRECT_REPLAN",
              collision_time_ahead);

          if (planFromCurrentTraj() ==
              ReplanResult::SUCCESS)
          {
            last_safety_replan_time_ = now;

            changeFSMExecState(
                EXEC_TRAJ,
                "SAFETY_EMERGENCY_SUCCESS");
          }
          else
          {
            changeFSMExecState(
                EMERGENCY_STOP,
                "SAFETY_EMERGENCY_FAILED");
          }

          return;
        }

        // ============================================================
        // B类和C类：非紧急碰撞。冷却期间继续当前轨迹，安全检查
        // 不再排队第二套 FSM 重规划循环。
        // ============================================================
        if (cooldown_active)
        {
          ROS_DEBUG_THROTTLE(
              1.0,
              "SCAN_SAFETY_REPLAN_SUPPRESSED "
              "collision_time_ahead=%.2f "
              "cooldown_remaining=%.2f",
              collision_time_ahead,
              safety_replan_cooldown_sec_ -
                  cooldown_elapsed);

          return;
        }
        // This attempt consumes the current safety-replan opportunity.
        last_safety_replan_time_ = now;

        // ============================================================
        // B类：近距离碰撞
        //
        // 冷却结束后直接尝试从当前轨迹重规划。
        // A near collision cannot continue on an unsafe old trajectory.
        // ============================================================
        if (collision_time_ahead <= safety_direct_replan_sec_)
        {
          ROS_WARN(
              "SCAN_SAFETY_REPLAN "
              "urgency=NEAR "
              "collision_time_ahead=%.2f "
              "action=DIRECT_REPLAN",
              collision_time_ahead);

          if (planFromCurrentTraj() ==
              ReplanResult::SUCCESS)
          {
            changeFSMExecState(
                EXEC_TRAJ,
                "SAFETY_NEAR_SUCCESS");
          }
          else
          {
            changeFSMExecState(
                EMERGENCY_STOP,
                "SAFETY_NEAR_FAILED");
          }

          return;
        }

        // ============================================================
        // C类：较远的未来碰撞。 Attempt once and let the normal EXEC_TRAJ
        // rolling update retry later; do not enqueue another FSM state.
        // ============================================================
        ROS_WARN(
            "SCAN_SAFETY_REPLAN "
            "urgency=FAR "
            "collision_time_ahead=%.2f "
            "action=DIRECT_ONCE",
            collision_time_ahead);

        if (planFromCurrentTraj() == ReplanResult::SUCCESS)
          last_safety_replan_time_ = now;
        else
          ROS_WARN_THROTTLE(1.0,
              "SCAN_SAFETY_KEEP_OLD collision_time_ahead=%.2f",
              collision_time_ahead);

        return;
      }
    }
  }

  // callReboundReplan：封装对SCANPlannerManager::reboundReplan的前后处理。步骤：
  // 1.调用getLocalTarget选取本轮的局部目标点（受target_distance_cap_m限制），
  // 无可用目标则返回TARGET_UNAVAILABLE；2.校验起始/目标状态均为有限数，非法则
  // 拒绝传入优化器；3.保存重规划前的旧局部轨迹作为回滚备份，调用
  // planner_manager_->reboundReplan执行实际优化，耗时过长（>=200ms）打印警告；
  // 4.若优化成功但localTrajectoryIsSafe检测到碰撞，则回滚并标记失败；5.若
  // 最终成功，将新轨迹封装为Bspline消息并发布，可视化最优轨迹，对比新旧
  // 轨迹在compare_time时刻的朝向差异并输出连续性日志，最后更新重规划成功时间/
  // 位置并返回SUCCESS；否则返回OPTIMIZATION_FAILED。
  SCANReplanFSM::ReplanResult SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj,
                                                              double target_distance_cap_m)
  {

    if (!getLocalTarget(target_distance_cap_m))
    {
      ROS_WARN_THROTTLE(
          2.0,
          "SCAN_LOCAL_TARGET_UNAVAILABLE "
          "start=(%.2f,%.2f) dist_to_goal=%.3f",
          start_pt_(0), start_pt_(1),
          (end_pt_ - start_pt_).head<2>().norm());
      return ReplanResult::TARGET_UNAVAILABLE;
    }

    // Never pass NaN/Inf into SCAN optimizer.
    if (!start_pt_.allFinite() ||
        !start_vel_.allFinite() ||
        !start_acc_.allFinite() ||
        !local_target_pt_.allFinite() ||
        !local_target_vel_.allFinite())
    {
      ROS_ERROR_THROTTLE(
          1.0,
          "[SCANReplanFSM] Invalid planning state: "
          "start=(%.3f %.3f %.3f), "
          "target=(%.3f %.3f %.3f)",
          start_pt_(0), start_pt_(1), start_pt_(2),
          local_target_pt_(0),
          local_target_pt_(1),
          local_target_pt_(2));
      return ReplanResult::TARGET_UNAVAILABLE;
    }

    const LocalTrajData previous_local_trajectory =
        planner_manager_->local_data_;
    const ros::WallTime replan_started = ros::WallTime::now();
    ROS_DEBUG("SCAN_REPLAN_BEGIN start=(%.3f,%.3f) target=(%.3f,%.3f)",
        start_pt_(0), start_pt_(1), local_target_pt_(0), local_target_pt_(1));
    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    const double replan_elapsed_ms =
        (ros::WallTime::now() - replan_started).toSec() * 1000.0;
    if (replan_elapsed_ms >= 200.0)
      ROS_WARN("SCAN_REPLAN_SLOW elapsed_ms=%.1f success=%d",
          replan_elapsed_ms, plan_success ? 1 : 0);
    else
      ROS_DEBUG("SCAN_REPLAN_END elapsed_ms=%.1f success=%d",
          replan_elapsed_ms, plan_success ? 1 : 0);
    have_new_target_ = false;

    if (plan_success)
    {
      double collision_time_sec = 0.0;
      if (!localTrajectoryIsSafe(collision_time_sec))
      {
        ROS_WARN("SCAN_LOCAL_PLAN_REJECTED target=(%.2f,%.2f) "
                 "collision_time=%.2f reason=FOOTPRINT_COLLISION",
                 local_target_pt_(0), local_target_pt_(1),
                 collision_time_sec);
        planner_manager_->local_data_ = previous_local_trajectory;
        plan_success = false;
      }
    }

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      ROS_DEBUG("SCAN_LOCAL_PLAN_SUCCESS target=(%.2f,%.2f) "
               "trajectory_id=%d duration=%.3f",
               local_target_pt_(0), local_target_pt_(1),
               info->traj_id_, info->duration_);

      /* publish traj */
      scan_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);

      visualization_->displayOptimalTraj(info->position_traj_, 0);

      const double compare_time = std::min(0.5,
          std::min(previous_local_trajectory.duration_, info->duration_));
      const double old_elapsed = previous_local_trajectory.start_time_.isZero()
          ? 0.0
          : std::max(0.0, (ros::Time::now() - previous_local_trajectory.start_time_).toSec());
      const double old_remaining_time = std::max(
          0.0, previous_local_trajectory.duration_ - old_elapsed);
      UniformBspline previous_position = previous_local_trajectory.position_traj_;
      const double old_yaw = previous_local_trajectory.duration_ > 1e-5
          ? estimateTrajectoryYaw(previous_position, std::max(0.0, compare_time))
          : getOdomYaw();
      const double new_yaw = estimateTrajectoryYaw(
          info->position_traj_, std::max(0.0, compare_time));
      const char* init_type = flag_use_poly_init
          ? (flag_randomPolyTraj ? "RANDOM" : "POLYNOMIAL")
          : "CONTINUATION";
      const double yaw_delta = normalizeAngle(new_yaw - old_yaw);
      const bool important_continuity_event =
          std::string(init_type) != "CONTINUATION" || std::abs(yaw_delta) >= 0.10;
      if (important_continuity_event)
      {
      ROS_INFO("SCAN_TRAJ_CONTINUITY old_traj_id=%d new_traj_id=%d init_type=%s yaw_delta=%.3f old_remaining_time=%.3f old_collision_time=%.3f",
          previous_local_trajectory.traj_id_, info->traj_id_,
          init_type, yaw_delta, old_remaining_time,
          std::numeric_limits<double>::infinity());
      }
      else
      {
      ROS_DEBUG_THROTTLE(2.0, "SCAN_TRAJ_CONTINUITY old_traj_id=%d new_traj_id=%d init_type=%s yaw_delta=%.3f old_remaining_time=%.3f old_collision_time=%.3f",
          previous_local_trajectory.traj_id_, info->traj_id_, init_type,
          yaw_delta, old_remaining_time, std::numeric_limits<double>::infinity());
      }

      last_successful_replan_time_ = ros::Time::now();
      last_replan_robot_position_ = odom_pos_;

      return ReplanResult::SUCCESS;
    }

    return ReplanResult::OPTIMIZATION_FAILED;
  }

  // callEmergencyStop：生成并发布一条静止在stop_pos的应急停止B样条轨迹。先调用
  // planner_manager_->EmergencyStop生成控制点均为stop_pos的静止轨迹，再将其封装为
  // Bspline消息并发布。
  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    ROS_WARN("SCAN_EMERGENCY_STOP_SPLINE trajectory_id=%d position=(%.2f,%.2f,%.2f)",
        info->traj_id_, stop_pos(0), stop_pos(1), stop_pos(2));

    /* publish traj */
    scan_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  // getLocalTarget：从全局轨迹/参考路径上选取本轮局部重规划的目标点local_target_pt_（受
  // target_distance_cap_m限制）。默认先将local_target_pt_设为起点（绝不回退到远方
  // 的全局终点，若找不到安全目标必须返回false让调用方回退重试）。
  // 整体三阶段流程：
  // 阶段1 采样：从上次进度时刻progress_t开始沿全局路径按固定时间步长采样，
  //   记录每个采样点到里程计起点的距离，并找到距离最近的采样时刻作为新的
  //   last_progress_time_（路径跟踪进度）；
  // 阶段2 选点：从规划地平线上限处开始向回搜索第一个且不低于最小目标距离
  //   的空闲采样点（仅检查候选点本身是否占据，机人到目标的路径不做占据检查，
  //   绕障交给后续reboundReplan+A*+B样条优化完成），找到则直接采用；
  // 阶段3 近目标回退：若机器人已很接近全局终点（在no_replan_thresh_与
  //   min_target_dist之间的盲区）且终点本身空闲，则直接以终点为局部目标；
  // 若以上三阶段均无法找到合法目标点，则返回false。
  bool SCANReplanFSM::getLocalTarget(double target_distance_cap_m)
  {
    auto &global_data = planner_manager_->global_data_;

    const double duration = global_data.global_duration_;
    const double max_vel = planner_manager_->pp_.max_vel_;

    // Never default to the global endpoint.  If no safe local target can be
    // found the caller must back off and retry later instead of sending the
    // far-away goal into the optimizer.
    local_target_pt_ = start_pt_;
    local_target_vel_.setZero();

    if (!start_pt_.allFinite() ||
        !end_pt_.allFinite() ||
        !std::isfinite(duration) ||
        duration <= 1e-6 ||
        !std::isfinite(max_vel) ||
        max_vel <= 1e-6 ||
        !std::isfinite(planning_horizon_) ||
        planning_horizon_ <= 0.0)
    {
      ROS_ERROR_THROTTLE(
          1.0,
          "[getLocalTarget] invalid global trajectory or parameters");
      return false;
    }

    auto map = planner_manager_->grid_map_;
    if (!map)
    {
      ROS_ERROR_THROTTLE(1.0, "[getLocalTarget] grid map not available");
      return false;
    }

    const double t_step = std::max(
        0.02,
        planning_horizon_ / 20.0 / max_vel);

    double progress_t = global_data.last_progress_time_;

    if (!std::isfinite(progress_t))
      progress_t = 0.0;

    progress_t = std::max(
        0.0,
        std::min(progress_t, duration));

    // --- Phase 1: sample route and record distance from robot ----------
    struct RouteSample
    {
      double t;
      Eigen::Vector3d pos;
      double dist;
    };
    std::vector<RouteSample> samples;

    double dist_min = 1e100;
    double dist_min_t = progress_t;

    for (double t = progress_t;
         t <= duration + 1e-6;
         t += t_step)
    {
      const double eval_t = std::max(
          0.0,
          std::min(t, duration));

      const Eigen::Vector3d pos_t =
          global_data.getPosition(eval_t);

      if (!pos_t.allFinite())
        continue;

      const double dist = (pos_t - start_pt_).norm();

      if (!std::isfinite(dist))
        continue;

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = eval_t;
      }

      samples.push_back({eval_t, pos_t, dist});
    }

    global_data.last_progress_time_ = dist_min_t;

    if (samples.empty())
    {
      ROS_ERROR_THROTTLE(
          1.0,
          "[getLocalTarget] no valid route samples");
      return false;
    }

    // --- occupancy helper: only check the candidate point itself ----------
    // The straight-line path from robot to target is NOT checked.
    // Obstacle avoidance is handled by reboundReplan (A* + B-spline optimisation).
    auto targetIsFree =
        [&](const Eigen::Vector3d &point) -> bool
    {
      if (!point.allFinite() || !map)
        return false;

      const double yaw =
          estimateYawFromSegment(odom_pos_, point);

      if (!std::isfinite(yaw))
        return false;

      return map->getInflateOccupancy(point, yaw) == 0;
    };

    // --- Phase 2: find the farthest free route sample ---------------------
    // Walk backward from planning_horizon_ so we pick the farthest free
    // point first.  Only the candidate point itself is checked; the path
    // from the robot to the target is left to reboundReplan + A* + B-spline
    // optimisation.
    const double min_target_dist = 0.8;
    const double max_target_dist =
        std::max(1.0, std::min(planning_horizon_, target_distance_cap_m));

    bool found_route_target = false;
    for (int i = static_cast<int>(samples.size()) - 1; i >= 0; --i)
    {
      if (!std::isfinite(samples[i].dist))
        continue;

      if (samples[i].dist > max_target_dist + 1e-6)
        continue;

      if (samples[i].dist < min_target_dist - 1e-6)
        break;  // all remaining samples are too close

      if (!targetIsFree(samples[i].pos))
        continue;

      local_target_pt_ = samples[i].pos;
      local_target_vel_.setZero();
      found_route_target = true;
      break;
    }

    if (found_route_target)
    {
      ROS_DEBUG("SCAN_LOCAL_TARGET_SELECTED target=(%.2f,%.2f,%.2f) dist=%.2f target_cap=%.2f",
          local_target_pt_(0), local_target_pt_(1), local_target_pt_(2),
          (local_target_pt_ - start_pt_).norm(), max_target_dist);
      return true;
    }

    // --- Phase 3: near-global-goal fallback --------------------------------
    // When the robot is within (no_replan_thresh_, min_target_dist) of the
    // final global target and the final target itself is free, plan directly
    // to it. This eliminates the 0.1~0.8 m dead zone where no ordinary route
    // sample satisfies the 0.8 m minimum-distance rule.
    const Eigen::Vector3d final_target =
        global_data.getPosition(duration);

    const double final_target_dist =
        (final_target - start_pt_).head<2>().norm();

    if (final_target.allFinite() &&
        final_target_dist > no_replan_thresh_ &&
        final_target_dist < min_target_dist &&
        targetIsFree(final_target))
    {
      local_target_pt_ = final_target;
      local_target_vel_.setZero();

      ROS_DEBUG(
          "SCAN_LOCAL_TARGET_SELECTED "
          "reason=NEAR_GLOBAL_GOAL "
          "target=(%.2f,%.2f,%.2f) dist=%.3f",
          local_target_pt_(0),
          local_target_pt_(1),
          local_target_pt_(2),
          final_target_dist);

      return true;
    }

    // --- no safe target found ----------------------------------------------
    ROS_WARN_THROTTLE(
        2.0,
        "SCAN_LOCAL_TARGET_UNAVAILABLE "
        "start=(%.2f,%.2f) dist_to_goal=%.3f",
        start_pt_(0),
        start_pt_(1),
        (end_pt_ - start_pt_).head<2>().norm());

    return false;
  }

} // namespace scan_planner
