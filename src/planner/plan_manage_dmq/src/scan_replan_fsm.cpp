
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
    nh.param("fsm/escape_min_radius", escape_min_radius_, 0.5);
    nh.param("fsm/escape_max_radius", escape_max_radius_, 2.0);
    nh.param("fsm/escape_radius_step", escape_radius_step_, 0.25);
    nh.param("fsm/escape_max_lateral_from_route",
             escape_max_lateral_from_route_, 1.2);
    if (!std::isfinite(escape_min_radius_) ||
        !std::isfinite(escape_max_radius_) ||
        !std::isfinite(escape_radius_step_) ||
        !std::isfinite(escape_max_lateral_from_route_) ||
        escape_min_radius_ <= 0.0 ||
        escape_max_radius_ < escape_min_radius_ ||
        escape_radius_step_ <= 0.0 ||
        escape_max_lateral_from_route_ <= 0.0)
    {
      ROS_FATAL("[SCANReplanFSM] invalid escape search parameters");
      ros::shutdown();
      return;
    }
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/max_replan_fail_count", max_replan_fail_count_, 1000);
    nh.param("grid_map/obstacles_inflation_z_up", self_inflation_z_up_, 0.0);
    nh.param("grid_map/obstacles_inflation_z_down", self_inflation_z_down_, 0.0);
    nh.param("grid_map/double_cylinder_radius", self_double_cylinder_radius_, 0.0);
    nh.param("grid_map/double_cylinder_offset", self_double_cylinder_offset_, 0.0);
    nh.param("grid_map/double_cylinder_offset", body_height_, 0.4);
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
      cout << "Wrong navi_mode_ value! navi_mode_=" << navi_mode_ << endl;
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

    cout << "Triggered!" << endl;
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

    trigger_ = true;

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      // Reference-path navigation is planar. Keep every waypoint on the
      // robot's current odometry height; do not add body height a second time.
      wp(2) = odom_pos_(2);
      waypoints.push_back(wp);
    }
    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

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
    last_escape_side_ = 0;
    need_hover_stop_ = false;
    exec_state_ = WAIT_TARGET;
    ROS_WARN("[SCANReplanFSM] native takeover state reset");
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

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void SCANReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    updateLocalTrajTimeFreeze();

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

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
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
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

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

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
        const double dist_to_goal = (end_pt_ - odom_pos_).norm();
        if (dist_to_goal < no_replan_thresh_)
        {
          if (isWaypointSequenceMode())
          {
            active_waypoints_.clear();
            current_wp_ = 0;
          }

          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM");
          return;
        }

        // Still far from the global goal — plan the next segment.
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
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
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      ROS_WARN("Replan failed %d times. Emergency stop and wait for a new target.", replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = odom_pos_;
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    if (!planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      ROS_ERROR("[navi_mode=%d] Unable to refresh global trajectory from odom to current target.", navi_mode_);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
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

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

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
      return false;
    }

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

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
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

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

  void SCANReplanFSM::getLocalTarget()
  {
    auto &global_data = planner_manager_->global_data_;

    const double duration = global_data.global_duration_;
    const double max_vel = planner_manager_->pp_.max_vel_;

    // Always initialize outputs to finite fallback values.
    local_target_pt_ = end_pt_;
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
      return;
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

    double dist_min = 1e100;
    double dist_min_t = progress_t;
    double target_t = duration;
    bool target_selected = false;

    for (double t = progress_t;
         t < duration - 1e-6;
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

      if (dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = eval_t;
        global_data.last_progress_time_ = dist_min_t;
        target_selected = true;
        break;
      }
    }

    // The old code used `t > duration`, which misses t == duration.
    if (!target_selected)
    {
      local_target_pt_ = end_pt_;
      target_t = duration;
      global_data.last_progress_time_ = dist_min_t;
    }

    auto sampleGlobalPosition =
        [&](double query_t, Eigen::Vector3d &point) -> bool
    {
      if (!std::isfinite(query_t))
        return false;

      query_t = std::max(
          0.0,
          std::min(query_t, duration));

      if (query_t >= duration - 1e-6)
        point = end_pt_;
      else
        point = global_data.getPosition(query_t);

      return point.allFinite();
    };

    auto targetIsFree =
        [&](const Eigen::Vector3d &point) -> bool
    {
      if (!point.allFinite() ||
          !planner_manager_->grid_map_)
      {
        return false;
      }

      const double yaw =
          estimateYawFromSegment(odom_pos_, point);

      if (!std::isfinite(yaw))
        return false;

      return planner_manager_->grid_map_
                 ->getInflateOccupancy(point, yaw) == 0;
    };

    bool target_adjusted = false;

    if (!targetIsFree(local_target_pt_))
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      const double lower_t = std::max(
          0.0,
          std::min(dist_min_t, duration));

      const int max_steps = std::max(
          1,
          static_cast<int>(
              std::ceil(duration / t_step)) + 2);

      for (int i = 0; i <= max_steps; ++i)
      {
        const double dt =
            static_cast<double>(i) * t_step;

        const double forward_t = target_t + dt;

        if (forward_t <= duration + 1e-6)
        {
          Eigen::Vector3d candidate;

          if (sampleGlobalPosition(
                  forward_t, candidate) &&
              targetIsFree(candidate))
          {
            local_target_pt_ = candidate;
            adjusted_t = std::min(
                forward_t, duration);
            found_free_target = true;
            break;
          }
        }

        const double backward_t = target_t - dt;

        if (backward_t >= lower_t - 1e-6)
        {
          Eigen::Vector3d candidate;

          if (sampleGlobalPosition(
                  backward_t, candidate) &&
              targetIsFree(candidate))
          {
            local_target_pt_ = candidate;
            adjusted_t = std::max(
                backward_t, lower_t);
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        target_t = adjusted_t;
        target_adjusted = true;

        ROS_WARN_THROTTLE(
            1.0,
            "[getLocalTarget] occupied target adjusted "
            "to finite free target: "
            "(%.3f %.3f %.3f), t=%.3f",
            local_target_pt_(0),
            local_target_pt_(1),
            local_target_pt_(2),
            target_t);
      }
      else
      {
        // The reference route can be blocked for longer than the planning
        // horizon. In that case every sampled point on the route is occupied
        // and sending the occupied endpoint to reboundReplan() causes an
        // endless GEN_NEW_TRAJ loop. Search around the robot, not around the
        // blocked lookahead point: the latter can be inside a large obstacle
        // and consequently has no free neighbouring cells at all.
        //
        const Eigen::Vector3d blocked_target = local_target_pt_;
        const double blocked_yaw = estimateYawFromSegment(
            start_pt_, blocked_target);

        Eigen::Vector2d route_dir(std::cos(blocked_yaw), std::sin(blocked_yaw));
        if (!route_dir.allFinite() || route_dir.norm() < 1e-6)
          route_dir = Eigen::Vector2d::UnitX();
        const Eigen::Vector2d route_normal(-route_dir.y(), route_dir.x());

        const auto routeDistanceAndProgress =
            [&](const Eigen::Vector3d &candidate,
                double &lateral_distance,
                double &route_time) -> bool
        {
          lateral_distance = std::numeric_limits<double>::infinity();
          route_time = dist_min_t;
          const double sample_step = std::max(0.05, t_step * 0.5);
          for (double t = dist_min_t; t <= duration + 1e-6;
               t += sample_step)
          {
            Eigen::Vector3d route_point;
            if (!sampleGlobalPosition(std::min(t, duration), route_point))
              continue;
            const double distance =
                (candidate - route_point).head<2>().norm();
            if (distance < lateral_distance)
            {
              lateral_distance = distance;
              route_time = std::min(t, duration);
            }
          }
          return std::isfinite(lateral_distance);
        };

        const auto pathIsFree = [&](const Eigen::Vector3d &candidate) -> bool
        {
          const double distance =
              (candidate - start_pt_).head<2>().norm();
          const int steps = std::max(2, static_cast<int>(std::ceil(distance / 0.20)));
          for (int step = 1; step <= steps; ++step)
          {
            const double ratio = static_cast<double>(step) / steps;
            const Eigen::Vector3d point =
                start_pt_ + ratio * (candidate - start_pt_);
            if (planner_manager_->grid_map_->getInflateOccupancy(
                    point, estimateYawFromSegment(start_pt_, candidate)) != 0)
              return false;
          }
          return true;
        };

        bool found_escape = false;
        double best_score = std::numeric_limits<double>::infinity();
        Eigen::Vector3d escape = start_pt_;
        int selected_side = 0;

        for (double radius = escape_min_radius_;
             radius <= escape_max_radius_ + 1e-6;
             radius += escape_radius_step_)
        {
          for (int angle_index = 0; angle_index < 24; ++angle_index)
          {
            const double angle = blocked_yaw +
                2.0 * M_PI * static_cast<double>(angle_index) / 24.0;
            Eigen::Vector3d candidate = start_pt_;
            candidate.x() += radius * std::cos(angle);
            candidate.y() += radius * std::sin(angle);
            candidate.z() = odom_pos_(2);
            if (!candidate.allFinite() ||
                (candidate - start_pt_).head<2>().norm() < 0.25 ||
                planner_manager_->grid_map_->getInflateOccupancy(
                    candidate, estimateYawFromSegment(start_pt_, candidate)) != 0)
              continue;
            if (!pathIsFree(candidate))
              continue;

            double lateral_distance = 0.0;
            double candidate_route_time = 0.0;
            if (!routeDistanceAndProgress(
                    candidate, lateral_distance, candidate_route_time) ||
                lateral_distance > escape_max_lateral_from_route_ ||
                candidate_route_time + 1e-6 < dist_min_t)
              continue;

            const Eigen::Vector2d displacement =
                candidate.head<2>() - start_pt_.head<2>();
            const double forward_progress = displacement.dot(route_dir);
            if (forward_progress < -0.05)
              continue;
            const double signed_side = displacement.dot(route_normal);
            const int side = signed_side > 0.05 ? 1 :
                             (signed_side < -0.05 ? -1 : 0);
            const double side_switch_penalty =
                last_escape_side_ != 0 && side != 0 && side != last_escape_side_
                    ? 1.0
                    : 0.0;
            const double score =
                2.0 * lateral_distance + 0.30 * radius -
                0.80 * forward_progress + side_switch_penalty;

            if (score < best_score)
            {
              best_score = score;
              escape = candidate;
              selected_side = side;
              found_escape = true;
            }
          }
        }

        if (found_escape)
        {
          local_target_pt_ = escape;
          local_target_vel_.setZero();
          target_adjusted = true;
          // Do not skip the blocked reference segment. Once the robot has
          // moved sideways, the next cycle projects its new pose and chooses
          // an appropriate forward target from that point.
          target_t = duration;
          if (selected_side != 0)
            last_escape_side_ = selected_side;
          ROS_WARN_THROTTLE(
              1.0,
              "[getLocalTarget] route blocked; use lateral escape "
              "target (%.2f %.2f %.2f)",
              escape.x(), escape.y(), escape.z());
        }
        else
        {
          const int start_occupancy =
              planner_manager_->grid_map_->getInflateOccupancy(
                  start_pt_, blocked_yaw);
          const int target_occupancy =
              planner_manager_->grid_map_->getInflateOccupancy(
                  blocked_target, blocked_yaw);
          ROS_ERROR_THROTTLE(
              1.0,
              "[getLocalTarget] no finite collision-free "
              "target or robot-centred lateral escape point found "
              "(start_occ=%d target_occ=%d)",
              start_occupancy, target_occupancy);
        }
      }
    }

    const double braking_distance =
        max_vel * max_vel /
        (2.0 * planner_manager_->pp_.max_acc_);

    const double remaining =
        (end_pt_ - local_target_pt_).norm();

    if (!std::isfinite(remaining) ||
        remaining <= braking_distance ||
        target_t >= duration - 1e-6)
    {
      local_target_vel_.setZero();
    }
    else
    {
      const double velocity_t = std::max(
          0.0,
          std::min(target_t, duration - 1e-6));

      const Eigen::Vector3d target_velocity =
          global_data.getVelocity(velocity_t);

      if (target_velocity.allFinite())
        local_target_vel_ = target_velocity;
      else
        local_target_vel_.setZero();
    }

    // An adjusted collision-free endpoint may stop safely.
    if (target_adjusted &&
        !local_target_vel_.allFinite())
    {
      local_target_vel_.setZero();
    }

    // Final fail-closed finite guarantee.
    if (!local_target_pt_.allFinite())
    {
      ROS_ERROR_THROTTLE(
          1.0,
          "[getLocalTarget] generated invalid point; "
          "fallback to final route target");

      local_target_pt_ = end_pt_;
      local_target_vel_.setZero();
    }
  }

} // namespace scan_planner
