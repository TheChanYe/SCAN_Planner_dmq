#pragma once

#include "navdog_scan_adapter/inflated_grid_query_3d.hpp"

#include <navdog_core/config.hpp>
#include <navdog_core/types.hpp>

#include <plan_manage_dmq/planner_manager.h>

#include <cstdint>
#include <memory>
#include <mutex>

namespace navdog_scan_adapter
{

// =============================================================================
// ScanLocalPlannerAdapter
//
// 将 SCANPlannerManager::reboundReplan 包装为 navdog_core::LocalPlannerAdapter。
// 负责把 SCAN LocalTrajData 采样为 navdog::LocalTrajectory。
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

  bool requestLocalPlan(
      const navdog::LocalPlanRequest& request) override;

  navdog::LocalTrajectory getLocalTrajectory(
      navdog::NavigationMode purpose,
      std::uint64_t task_sequence) const override;

  bool hasValidTrajectory(
      navdog::NavigationMode purpose,
      std::uint64_t task_sequence) const override;

  bool isTrajectoryColliding(
      navdog::NavigationMode purpose,
      std::uint64_t task_sequence) const override;

  bool checkTrajectoryCollision(
      const navdog::LocalTrajectory& trajectory) const;

private:
  navdog::LocalTrajectory sampleLocalTrajData(
      std::uint64_t task_sequence,
      std::uint64_t plan_sequence,
      navdog::NavigationMode purpose);

  bool shouldReplan(
      const navdog::LocalPlanRequest& request) const;

  navdog::PlannerTriggerConfig config_{};
  std::shared_ptr<InflatedGridQuery3D> grid_query_{};
  std::shared_ptr<scan_planner::SCANPlannerManager>
      planner_manager_{};

  mutable std::mutex mutex_;

  navdog::LocalPlanRequest last_request_{};
  double last_plan_stamp_sec_{0.0};
  bool planning_in_progress_{false};
  navdog::LocalTrajectory cached_trajectory_{};
};

}  // namespace navdog_scan_adapter
