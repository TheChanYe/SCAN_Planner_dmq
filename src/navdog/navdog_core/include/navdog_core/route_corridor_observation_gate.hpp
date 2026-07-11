#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

#include <cstdint>

namespace navdog
{

enum class RouteCorridorObservationResult : std::uint8_t
{
  IDLE = 0,

  CLEAR,
  BLOCKED,

  WAITING_FOR_OBSERVATION,
  TASK_MISMATCH,
  STALE_MAP,
  FUTURE_MAP,
  STALE_PROGRESS,
  FUTURE_PROGRESS,
  OUT_OF_MAP,
  INVALID_OBSERVATION,

  INVALID_TIME,
  INVALID_CONFIG,
  INVALID_PROGRESS
};

struct RouteCorridorObservationOutput
{
  RouteCorridorObservationResult result{
      RouteCorridorObservationResult::IDLE};

  RouteCorridorAssessment assessment{};
};

class RouteCorridorObservationGate
{
public:
  explicit RouteCorridorObservationGate(
      const RouteCorridorObservationConfig& config =
          RouteCorridorObservationConfig{});

  RouteCorridorObservationOutput evaluate(
      const RouteProgress& current_progress,
      const RouteCorridorAssessment& observation,
      double now_sec) const;

private:
  bool isConfigValid() const noexcept;
  bool isProgressValid(
      const RouteProgress& progress) const noexcept;
  bool isObservationNumericValid(
      const RouteCorridorAssessment& observation) const noexcept;

  RouteCorridorObservationConfig config_{};
};

}  // namespace navdog
