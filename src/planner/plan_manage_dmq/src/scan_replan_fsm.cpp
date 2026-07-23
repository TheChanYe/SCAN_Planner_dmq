
#include <plan_manage_dmq/scan_replan_fsm.h>
#include <cmath>
#include <cstdlib>
#include <limits>

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
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

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

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

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
    publishSelfInflationMarker();
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::BoolConstPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

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

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

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

  double SCANReplanFSM::normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

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
        "REPLAN_TRAJ",
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

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    ROS_DEBUG("[FSM]: state: %s", state_str[int(exec_state_)].c_str());
  }

  void SCANReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    updateLocalTrajTimeFreeze();

    static const char* state_names[] = {
        "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ",
        "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    ROS_DEBUG_THROTTLE(5.0,
        "SCAN_FSM state=%s trigger=%d target=%d",
        state_names[int(exec_state_)],
        static_cast<int>(trigger_),
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

      setStartStateFromOdomOrCurrentTraj();

      // Use independent attempt counter — NOT timesOfConsecutiveStateCalls().
      const int attempt = initial_plan_attempt_count_;

      // Alternate between deterministic and random/A* initialization.
      const bool random_init = (attempt % 2) == 1;

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

    case REPLAN_TRAJ:
    {
      if (!replanRetryReady(0.10))
      {
        break;  // wait for map update, do not count as failure
      }

      {
        const ReplanResult result = planFromCurrentTraj();
        if (result == ReplanResult::SUCCESS)
        {
          initial_plan_attempt_count_ = 0;
          replan_fail_count_ = 0;
          first_replan_failure_time_ = ros::Time(0);
          emergency_stop_active_ = false;
          next_target_retry_time_ = ros::Time(0);
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else if (result == ReplanResult::TARGET_UNAVAILABLE)
        {
          // No target — don't count as optimization failure.
          next_target_retry_time_ =
              ros::Time::now() + ros::Duration(replan_retry_interval_sec_);
        }
        else  // OPTIMIZATION_FAILED
        {
          if (first_replan_failure_time_.isZero())
            first_replan_failure_time_ = ros::Time::now();

          ++replan_fail_count_;

          ROS_WARN_THROTTLE(
              1.0,
              "SCAN_REPLAN_FAILED count=%d",
              replan_fail_count_);
        }
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

      // Normal rolling replanning never leaves EXEC_TRAJ: a failed candidate
      // leaves the currently safe B-spline active and is retried shortly.
      if ((periodic_due || trajectory_ending) && retry_ready &&
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
        ROS_WARN_THROTTLE(1.0, "SCAN_REPLAN_KEEP_OLD traj_id=%d failure_reason=%s remaining_time=%.3f collision_time_ahead=%.3f",
            old_traj_id,
            result == ReplanResult::TARGET_UNAVAILABLE ? "TARGET_UNAVAILABLE" : "OPTIMIZATION_FAILED",
            remaining_time, collision_time);
        if (!old_safe && collision_time <= safety_immediate_replan_sec_)
          changeFSMExecState(EMERGENCY_STOP, "NOMINAL_OLD_TRAJ_UNSAFE");
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

        // Check if the robot has actually reached the global goal.
        // When the local trajectory expires but the global target is still
        // far away (e.g. after a lateral escape manoeuvre), do NOT clear
        // have_target_ and go straight to GEN_NEW_TRAJ so getLocalTarget()
        // can pick the next forward target or another escape point.
        // We should normally have replanned before this point. Keep the
        // previous trajectory data and retry from EXEC_TRAJ rather than
        // clearing it or entering a blocking state.
        return;
      }
      // All non-emergency replans above stay in EXEC_TRAJ.  Do not retain a
      // second distance-only REPLAN_TRAJ path, which would turn a normal
      // rolling failure into a blocking state transition.
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
        // B类和C类：非紧急碰撞
        //
        // 冷却尚未结束时保持当前可执行轨迹，不再次切入REPLAN_TRAJ。
        // 原代码虽然打印cooldown，但仍切到REPLAN_TRAJ，实际上没有限频。
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
        // 从这里开始代表本次确实消费了一次安全重规划机会。
        // 在排队或直接调用规划前记录，避免规划失败后立即被安全定时器重复触发。
        last_safety_replan_time_ = now;

        // ============================================================
        // B类：近距离碰撞
        //
        // 冷却结束后直接尝试从当前轨迹重规划。
        // 失败时保留旧轨迹数据，并交给REPLAN_TRAJ继续尝试。
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
                REPLAN_TRAJ,
                "SAFETY_NEAR_FAILED");
          }

          return;
        }

        // ============================================================
        // C类：较远的未来碰撞
        //
        // 不在安全定时器中连续直接规划，只向FSM排队一次。
        // 后续至少等待cooldown结束，才允许新一轮安全触发。
        // ============================================================
        ROS_WARN(
            "SCAN_SAFETY_REPLAN "
            "urgency=FAR "
            "collision_time_ahead=%.2f "
            "action=QUEUE_REPLAN",
            collision_time_ahead);

        changeFSMExecState(
            REPLAN_TRAJ,
            "SAFETY_FAR");

        return;
      }
    }
  }

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
    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
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
