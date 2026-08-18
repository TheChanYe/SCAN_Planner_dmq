#ifndef _PLANNER_MANAGER_DMQ_H_
#define _PLANNER_MANAGER_DMQ_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <scan_planner/DataDisp.h>
#include <plan_env/grid_map.h>
#include <plan_manage_dmq/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>

namespace scan_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  // SCANPlannerManager：规划模块的总入口，负责调度地图/B样条优化等核心算法，
  // 对外提供重规划（reboundReplan）、紧急停止（EmergencyStop）与全局轨迹
  // 规划（planGlobalTraj*）三类接口。
  class SCANPlannerManager
  {
    // SECTION stable
  public:
    SCANPlannerManager();
    ~SCANPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    // reboundReplan：核心局部重规划接口，以起始位置/速度/加速度与目标位置/速度为
    // 输入，先生成初始B样条控制点（直线或多项式/随机初始化）再进行rebound
    // 优化，成功则写入local_data_。flag_polyInit/flag_randomPolyTraj控制初始化方式。
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    // EmergencyStop：生成一段从当前位置到指定停止位置的平滑制动轨迹，用于紧急情况下
    // 快速停车。
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    // planGlobalTraj：根据单个起、终点规划一条完整的全局多项式轨迹。
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    // planGlobalTrajWaypoints：根据一系列中间路点规划一条途经全部路点的全局多项式轨迹。
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    // initPlanModules：从ROS参数服务器加载规划参数，创建GridMap与B样条优化器并注入
    // 可选的可视化模块。
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;
    bool stair_route_active_{false};

  private:
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    BsplineOptimizer::Ptr bspline_optimizer_rebound_;

    int continuous_failures_count_{0};

    // updateTrajInfo：用新生成的位置B样条更新local_data_（求导得速度/加速度
    // 轨迹并记录时长/开始时刻等元信息）。
    void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);
    // checkDynamicFeasibility：检查给定位置B样条求导得到的速度/加速度是否超出
    // 物理限制（允许一定比例的超限容差）。
    bool checkDynamicFeasibility(UniformBspline position_traj);

    // reparamBspline：根据速度/加速度超限比例ratio重新参数化B样条（拉长时间轴降低
    // 速度/加速度需求），得到新的控制点/时间步长与时长增量。
    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    // refineTrajAlgo：循环调用reparamBspline与重新优化，直到轨迹满足动力学可行性
    // 或达到重试上限，用于“先优化、后拉伸时间微调”的两阶段策略。
    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<SCANPlannerManager> Ptr;

    // !SECTION
  };
} // namespace scan_planner

#endif
