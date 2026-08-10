#ifndef _SCAN_REPLAN_FSM_DMQ_H_
#define _SCAN_REPLAN_FSM_DMQ_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <scan_planner/Bspline.h>
#include <scan_planner/DataDisp.h>
#include <plan_manage_dmq/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  // SCANReplanFSM：基于有限状态机的重规划调度器。负责接收目标/路径/里程计，
  // 驱动SCANPlannerManager周期性重规划局部B样条轨迹，并对外发布B样条消息与
  // 可视化信息，同时处理碰撞/安全/接管同步等异常情况。
  class SCANReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    // FSM_EXEC_STATE：执行状态机主状态：初始化/等待目标/生成新轨迹/执行轨迹/紧急停止。
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    };
    // NAVI_MODE：目标来源模式：手动选点/预设多路点/外部参考路径。
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFERENCE_PATH = 3,
    };

    /* planning utils */
    SCANPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    scan_planner::DataDisp data_disp_;

    /* parameters */
    int navi_mode_; // 1 manual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    std::vector<Eigen::Vector3d> preset_waypoints_;
    int waypoint_num_;
    double planning_horizon_;
    double emergency_time_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;
    std::string self_inflation_frame_id_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool rviz_height_ready_;
    bool go2_execution_frozen_;
    bool enable_fail_safe_, need_hover_stop_;
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int continuation_failure_count_{0};
    int max_replan_fail_count_{12};
    ros::Time first_replan_failure_time_;
    ros::Time last_freeze_update_time_;
    ros::Time last_odom_time_;  // wall-clock time of the most recent odometry callback,
                                 // used only for the SCAN_FSM_HEARTBEAT liveness log.
    ros::Time last_replan_attempt_time_;
    ros::Time last_successful_replan_time_;
    ros::Time last_nominal_replan_attempt_time_;
    Eigen::Vector3d last_replan_robot_position_{Eigen::Vector3d::Zero()};
    bool planning_in_progress_{false};
    bool takeover_sync_pending_{false};
    bool force_takeover_poly_init_{false};
    double nominal_replan_period_sec_{0.20};
    double min_replan_progress_m_{0.05};
    double replan_retry_interval_sec_{0.10};
    double replan_lead_time_sec_{0.40};
    ros::Time next_emergency_retry_time_;
    ros::Time next_target_retry_time_{0};

    // Independent initial planning attempt counter (not tied to FSM state).
    int initial_plan_attempt_count_{0};
    bool emergency_stop_active_{false};
    double emergency_retry_interval_sec_{0.50};

    // Safety replan triage parameters.
    double safety_immediate_replan_sec_{1.0};
    double safety_direct_replan_sec_{3.0};
    double safety_replan_cooldown_sec_{0.20};
    ros::Time last_safety_replan_time_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> active_waypoints_;
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber goal_sub_, odom_sub_, path_sub_, go2_execution_frozen_sub_, reset_sub_, takeover_sync_sub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_, self_inflation_pub_;

    enum class ReplanResult
    {
      SUCCESS,
      TARGET_UNAVAILABLE,
      OPTIMIZATION_FAILED
    };

    /* helper functions */
    // callReboundReplan：封装对SCANPlannerManager::reboundReplan的调用，处理前后端
    // 转换与结果分类（成功/目标不可用/优化失败）。
    ReplanResult callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj,
                                   double target_distance_cap_m); // front-end and back-end method
    // callEmergencyStop：封装对SCANPlannerManager::EmergencyStop的调用并处理发布。
    bool callEmergencyStop(Eigen::Vector3d stop_pos);             // front-end and back-end method
    // planFromCurrentTraj：以当前正在执行的轨迹为基础重新获取局部目标并发起重规划。
    ReplanResult planFromCurrentTraj();
    // replanRetryReady：判断距上次尝试重规划是否已超过指定重试间隔。
    bool replanRetryReady(double interval_sec);
    // localTrajectoryIsSafe：检查当前局部轨迹没有碰撞，若存在碰撞则输出碰撞时刻。
    bool localTrajectoryIsSafe(double &collision_time_sec);
    // setStartStateFromOdomOrCurrentTraj：根据当前里程计或正在执行的轨迹确定本次重规
    // 划的起始位置/速度/加速度/朝向。
    void setStartStateFromOdomOrCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    // changeFSMExecState：切换到新的执行状态并记录调用位置，打印状态转换日志。
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    // timesOfConsecutiveStateCalls：统计当前状态被连续调用的次数，用于检测卡死在
    // 某一状态的异常情况。
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    // printFSMExecState：打印当前执行状态日志。
    void printFSMExecState();

    // planGlobalTrajbyGivenWps：使用预设路点集规划一条完整全局轨迹。
    void planGlobalTrajbyGivenWps();
    // planGlobalTrajByWaypoints：使用给定路点集规划全局轨迹。
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    // planNextWaypoint：切换到下一个待访路点并重新规划。
    bool planNextWaypoint();
    // isWaypointSequenceMode：判断当前是否处于多路点序列导航模式。
    bool isWaypointSequenceMode() const;
    // adjustGlobalTargetIfOccupied：若全局目标点被地图占据，尝试将其调整到附近空闲位置。
    bool adjustGlobalTargetIfOccupied();
    // escapeInflatedStart：当拟定起点被地图判定为膨胀障碍时（机体贴近障碍导致），
    // 沿机体当前朝向的反方向（来向，通常是安全区）逐步回退搜索最近的空闲点，
    // 作为本次规划的替代起点，避免每次重试都用同一个卡死点导致永久失败。
    bool escapeInflatedStart(Eigen::Vector3d &start_pt) const;
    // getLocalTarget：从全局轨迹上按规划地平线长度（受target_distance_cap_m限制）截取局部
    // 目标位置与速度。
    bool getLocalTarget(double target_distance_cap_m);
    // finishProcess：导航任务结束后的收尾处理（重置标志、发布相关信息）。
    void finishProcess();
    // publishSelfInflationMarker：发布自身膨胀区域可视化标记。
    void publishSelfInflationMarker();
    // getOdomYaw：从里程计四元数提取当前朝向角。
    double getOdomYaw() const;
    // estimateYawFromSegment：根据两点连线估计朝向角。
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    // estimateTrajectoryYaw：根据轨迹在指定时刻附近的切线方向估计朝向角。
    double estimateTrajectoryYaw(UniformBspline &trajectory, double time_sec) const;
    // normalizeAngle：将角度归一化到[-pi, pi]区间。
    static double normalizeAngle(double angle);
    // updateLocalTrajTimeFreeze：处理执行冻结（冻时）情况下的局部轨迹时间推进逻辑。
    void updateLocalTrajTimeFreeze();

    /* ROS functions */
    // execFSMCallback：主状态机循环，根据当前状态驱动目标获取/重规划/发布等流程。
    void execFSMCallback(const ros::TimerEvent &e);
    // checkCollisionCallback：定时检查当前轨迹是否碰撞，必要时触发安全重规划。
    void checkCollisionCallback(const ros::TimerEvent &e);
    // rvizGoalCallback：接收RViz中手动设置的目标点。
    void rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    // waypointCallback：接收多路点路径消息。
    void waypointCallback(const nav_msgs::PathConstPtr &msg);
    // pathCallback：接收外部参考路径消息。
    void pathCallback(const nav_msgs::PathConstPtr &msg);
    // odometryCallback：接收里程计，更新当前位置/速度/加速度/姿态。
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    // go2ExecutionFrozenCallback：接收Go2执行冻结信号，控制是否暂停时间推进。
    void go2ExecutionFrozenCallback(const std_msgs::BoolConstPtr &msg);
    // resetCallback：接收外部重置信号，清空当前任务状态。
    void resetCallback(const std_msgs::EmptyConstPtr &msg);
    // takeoverSyncCallback：接收接管同步信号，标记待处理接管。
    void takeoverSyncCallback(const std_msgs::EmptyConstPtr &msg);

    // checkCollision：对当前局部轨迹做碰撞检测。
    bool checkCollision();

  public:
    SCANReplanFSM(/* args */)
    {
    }
    ~SCANReplanFSM()
    {
    }

    // init：初始化FSM，加载全部参数、创建规划器与可视化模块、注册全部ROS订阅/
    // 发布者与定时器。
    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
