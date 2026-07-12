#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/route_corridor_observation_gate.hpp"
#include "navdog_core/types.hpp"

#include <cstdint>

namespace navdog
{

// =============================================================================
// NavigationModeUpdateResult
// =============================================================================

enum class NavigationModeUpdateResult : std::uint8_t
{
  IDLE = 0,

  UPDATED,
  WAITING_FOR_CORRIDOR,
  WAITING_FOR_ROBOT,

  INVALID_TIME,
  INVALID_CONFIG,
  INVALID_TASK,
  INVALID_PROGRESS,
  INVALID_ROBOT,
  INVALID_CORRIDOR_RESULT
};

// =============================================================================
// NavigationModeOutput
// =============================================================================

struct NavigationModeOutput
{
  NavigationModeUpdateResult result{
      NavigationModeUpdateResult::IDLE};

  NavigationModeStatus status{};
};

// =============================================================================
// NavigationModeManager
// =============================================================================

class NavigationModeManager
{
public:
  explicit NavigationModeManager(
      const NavigationModeConfig& config =
          NavigationModeConfig{});

  void reset() noexcept;

  NavigationModeOutput update(
      const NavigationTask& task,
      const RobotState& robot,
      const RouteProgress& progress,
      const RouteCorridorObservationOutput& corridor,
      double now_sec);

  const NavigationModeStatus& status() const noexcept;

private:
  bool isConfigValid() const noexcept;

  bool isTaskValid(
      const NavigationTask& task) const noexcept;

  bool isProgressValid(
      const NavigationTask& task,
      const RouteProgress& progress) const noexcept;

  bool isRobotNumericValid(
      const RobotState& robot) const noexcept;

  bool taskAllowsAvoidance(
      TaskMode task_mode) const noexcept;

  void initializeForTask(
      const NavigationTask& task,
      double now_sec);

  void transitionTo(
      NavigationMode new_mode,
      NavigationModeReason reason,
      const RouteProgress& progress,
      double now_sec);

  void resetBlockedEvidence() noexcept;
  void resetClearEvidence() noexcept;
  void resetRejoinEvidence() noexcept;

  double shortestAngularDistance(
      double from,
      double to) const noexcept;

  NavigationModeConfig config_{};
  NavigationModeStatus status_{};

  std::uint64_t active_task_sequence_{0};

  double last_update_stamp_sec_{0.0};
  bool has_last_update_stamp_{false};

  double blocked_since_sec_{0.0};
  bool blocked_timer_active_{false};

  double clear_since_sec_{0.0};
  bool clear_timer_active_{false};

  double rejoin_since_sec_{0.0};
  bool rejoin_timer_active_{false};
};

}  // namespace navdog
