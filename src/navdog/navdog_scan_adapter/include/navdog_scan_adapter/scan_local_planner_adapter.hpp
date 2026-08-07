#pragma once

#include "navdog_scan_adapter/inflated_grid_query_3d.hpp"

#include <navdog_core/config.hpp>
#include <navdog_core/types.hpp>

#include <plan_manage_dmq/planner_manager.h>

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace navdog_scan_adapter
{

// ScanAdapterConfig
// 适配层配置，目前仅包含规划器触发相关参数。
struct ScanAdapterConfig
{
  navdog::PlannerTriggerConfig planner_trigger{};
};

// =============================================================================
// ScanLocalPlannerAdapter
//
// 将 SCANPlannerManager::reboundReplan 包装为 navdog_core::LocalPlannerAdapter。
// 负责把 SCAN LocalTrajData 采样为 navdog::LocalTrajectory。
//
// 规划在单独工作线程中执行，控制线程只提交请求并立即返回。
// =============================================================================

class ScanLocalPlannerAdapter
    : public navdog::LocalPlannerAdapter
{
public:
  // 构造函数：保存规划触发配置、膨胀地图查询接口与SCAN规划器管理器，
  // 并启动后台规划工作线程（planningLoop）。
  ScanLocalPlannerAdapter(
      const navdog::PlannerTriggerConfig& config,
      const std::shared_ptr<InflatedGridQuery3D>& grid_query,
      const std::shared_ptr<scan_planner::SCANPlannerManager>&
          planner_manager);

  // 析构函数：设置关闭标志并唤醒工作线程，等待其退出后回收。
  ~ScanLocalPlannerAdapter();

  // requestLocalPlan：控制线程提交一个局部规划请求并立即返回，实际重规划在
  // 工作线程中异步执行。新请求会覆盖尚未处理的旧待处理请求。
  bool requestLocalPlan(
      const navdog::LocalPlanRequest& request) override;

  // getLocalTrajectory：获取与指定导航模式/任务序号匹配的最新已完成局部轨迹，
  // 不匹配则返回无效轨迹。
  navdog::LocalTrajectory getLocalTrajectory(
      navdog::NavigationMode purpose,
      std::uint64_t task_sequence) const override;

  // hasValidTrajectory：判断当前是否存在与指定模式/任务匹配的有效已完成轨迹。
  bool hasValidTrajectory(
      navdog::NavigationMode purpose,
      std::uint64_t task_sequence) const override;

  // localPlanState：查询指定任务/规划序号的局部规划当前状态（IDLE/PENDING/
  // SUCCEEDED/FAILED等）。
  navdog::LocalPlanState localPlanState(
      navdog::NavigationMode purpose,
      std::uint64_t task_sequence,
      std::uint64_t plan_sequence) const override;

  // isTrajectoryColliding：从指定时刻起检查轨迹剩余部分是否与当前膨胀地图碰撞。
  bool isTrajectoryColliding(
      const navdog::LocalTrajectory& trajectory,
      double from_time_sec) const override;

private:
  friend class ScanLocalPlannerAdapterTestPeer;

  // planningLoop：后台工作线程主循环：等待待处理请求或关闭信号，取出待处理请求
  // 后调用doReboundReplan执行实际重规划，并将结果写入completed_*供读取。
  void planningLoop();

  // sampleLocalTrajData：从SCANPlannerManager的LocalTrajData采样生成
  // navdog::LocalTrajectory，并绑定任务/规划序号、导航用途与来源时间戳。
  navdog::LocalTrajectory sampleLocalTrajData(
      std::uint64_t task_sequence,
      std::uint64_t plan_sequence,
      navdog::NavigationMode purpose,
      double source_stamp_sec);

  // doReboundReplan：实际调用SCANPlannerManager::reboundReplan执行一次重规划，
  // 输出deterministic_success/random_success两种尝试的成功标志。
  bool doReboundReplan(
      const navdog::LocalPlanRequest& request,
      bool& deterministic_success,
      bool& random_success);

  // storePlanResult：将本次规划请求与结果轨迹写入已完成缓存，供控制线程读取。
  void storePlanResult(
      const navdog::LocalPlanRequest& request,
      const navdog::LocalTrajectory& trajectory);

  // checkTrajectoryCollision：对轨迹从指定时刻起的剩余部分逐点采样并查询膨胀地图，
  // 任一采样点非空闲则判定为碰撞。
  bool checkTrajectoryCollision(
      const navdog::LocalTrajectory& trajectory,
      double from_time_sec) const;

  // isSampledTrajectoryValid：校验采样后的轨迹是否合法（非空、时长为正、各点坐标
  // 为有限数等）。
  static bool isSampledTrajectoryValid(
      const navdog::LocalTrajectory& trajectory) noexcept;

  navdog::PlannerTriggerConfig config_{};              // 规划触发配置
  std::shared_ptr<InflatedGridQuery3D> grid_query_{};  // 膨胀地图查询接口（用于碰撞检查）
  std::shared_ptr<scan_planner::SCANPlannerManager>
      planner_manager_{};                              // SCAN规划器管理器

  mutable std::mutex mutex_;         // 保护以下所有共享状态的互斥锁
  std::condition_variable cv_;       // 用于唤醒工作线程处理新请求或关闭
  std::thread worker_thread_;        // 后台规划工作线程
  bool shutdown_{false};             // 是否请求工作线程退出

  navdog::LocalPlanRequest pending_request_{};   // 待处理的最新请求（新请求会覆盖旧的）
  bool has_pending_request_{false};              // 是否存在待处理请求

  navdog::LocalPlanRequest active_request_{};    // 当前工作线程正在处理的请求
  bool has_active_request_{false};               // 是否有正在处理的请求

  navdog::LocalPlanRequest completed_request_{};       // 最近一次已完成的请求
  navdog::LocalPlanState completed_state_{
      navdog::LocalPlanState::IDLE};                   // 最近一次完成的规划状态
  navdog::LocalTrajectory completed_candidate_{};      // 最近一次完成的候选轨迹

  // 为测试预留的两种初始化选择的seam。生产代码总是直接调用
  // SCANPlannerManager。
  std::function<bool(bool)> replan_attempt_for_test_{};
};

}  // namespace navdog_scan_adapter
