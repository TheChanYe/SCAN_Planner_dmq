#pragma once

#include "navdog_scan_adapter/inflated_grid_query_3d.hpp"

#include <navdog_core/config.hpp>
#include <navdog_core/types.hpp>

#include <plan_manage_dmq/planner_manager.h>

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

namespace navdog_scan_adapter
{

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
  ScanLocalPlannerAdapter(
      const navdog::PlannerTriggerConfig& config,
      const std::shared_ptr<InflatedGridQuery3D>& grid_query,
      const std::shared_ptr<scan_planner::SCANPlannerManager>&
          planner_manager);

  ~ScanLocalPlannerAdapter();

  bool requestLocalPlan(
      const navdog::LocalPlanRequest& request) override;

  navdog::LocalTrajectory getLocalTrajectory(
      navdog::NavigationMode purpose,
      std::uint64_t task_sequence) const override;

  bool hasValidTrajectory(
      navdog::NavigationMode purpose,
      std::uint64_t task_sequence) const override;

  navdog::LocalPlanState localPlanState(
      navdog::NavigationMode purpose,
      std::uint64_t task_sequence,
      std::uint64_t plan_sequence) const override;

  bool isTrajectoryColliding(
      navdog::NavigationMode purpose,
      std::uint64_t task_sequence,
      std::uint64_t plan_sequence,
      double from_time_sec) const override;

private:
  void planningLoop();

  navdog::LocalTrajectory sampleLocalTrajData(
      std::uint64_t task_sequence,
      std::uint64_t plan_sequence,
      navdog::NavigationMode purpose);

  bool doReboundReplan(
      const navdog::LocalPlanRequest& request);

  bool checkTrajectoryCollision(
      const navdog::LocalTrajectory& trajectory,
      double from_time_sec) const;

  bool shouldReplan(
      const navdog::LocalPlanRequest& request) const;

  navdog::PlannerTriggerConfig config_{};
  std::shared_ptr<InflatedGridQuery3D> grid_query_{};
  std::shared_ptr<scan_planner::SCANPlannerManager>
      planner_manager_{};

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_thread_;
  bool shutdown_{false};

  navdog::LocalPlanRequest pending_request_{};
  bool has_pending_request_{false};

  navdog::LocalPlanRequest last_request_{};
  double last_plan_stamp_sec_{0.0};
  navdog::LocalTrajectory cached_trajectory_{};
  navdog::LocalPlanState state_{navdog::LocalPlanState::IDLE};
};

}  // namespace navdog_scan_adapter
