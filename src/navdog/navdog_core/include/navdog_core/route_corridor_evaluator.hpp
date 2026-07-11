#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

#include <cstddef>
#include <cstdint>

namespace navdog
{

enum class RouteCorridorResult : std::uint8_t
{
  IDLE = 0,

  CLEAR,
  BLOCKED,

  WAITING_FOR_OBSTACLES,
  STALE_OBSTACLES,
  FUTURE_OBSTACLES,
  INVALID_OBSTACLES,

  INVALID_TIME,
  INVALID_CONFIG,
  INVALID_TASK,
  INVALID_PROGRESS,
  INVALID_ROBOT
};

struct RouteCorridorOutput
{
  RouteCorridorResult result{
      RouteCorridorResult::IDLE};

  RouteCorridorAssessment assessment{};
};

class RouteCorridorEvaluator
{
public:
  RouteCorridorEvaluator(
      const RouteCorridorConfig& corridor_config =
          RouteCorridorConfig{},
      const RouteProgressConfig& progress_config =
          RouteProgressConfig{},
      const SafetyConfig& safety_config =
          SafetyConfig{});

  RouteCorridorOutput evaluate(
      const NavigationTask& task,
      const RouteProgress& progress,
      const RobotState& robot,
      const ObstacleField& obstacle_field,
      double now_sec) const;

private:
  struct SegmentPiece
  {
    double x0{0.0};
    double y0{0.0};
    double x1{0.0};
    double y1{0.0};

    double dx{0.0};
    double dy{0.0};
    double length{0.0};

    // 此段起点距离当前路线进度的前向距离。
    double distance_from_progress_m{0.0};
  };

  struct DistanceResult
  {
    double distance_sq{0.0};

    // 障碍物投影到线段上的比例 [0,1]。
    double ratio{0.0};

    // 障碍物投影到线段上的原始比例（未 clamp）。
    // < 0 表示障碍物在线段起点后方。
    double unclamped_ratio{0.0};
  };

  bool isConfigValid() const noexcept;

  bool isTaskValid(
      const NavigationTask& task) const noexcept;

  bool isProgressValid(
      const NavigationTask& task,
      const RouteProgress& progress) const noexcept;

  bool isRobotValid(
      const RobotState& robot) const noexcept;

  bool isObstacleFieldNumericValid(
      const ObstacleField& obstacle_field) const noexcept;

  DistanceResult pointToSegmentDistance(
      double point_x,
      double point_y,
      const SegmentPiece& segment) const noexcept;

  void evaluateSegment(
      const SegmentPiece& segment,
      const ObstacleField& obstacle_field,
      RouteCorridorAssessment& assessment) const noexcept;

  RouteCorridorOutput evaluatePolylineRoute(
      const NavigationTask& task,
      const RouteProgress& progress,
      const ObstacleField& obstacle_field,
      double now_sec) const;

  RouteCorridorOutput evaluateSinglePointRoute(
      const NavigationTask& task,
      const RouteProgress& progress,
      const RobotState& robot,
      const ObstacleField& obstacle_field,
      double now_sec) const;

  static double clamp(
      double value,
      double lower,
      double upper) noexcept;

  RouteCorridorConfig corridor_config_{};
  RouteProgressConfig progress_config_{};
  SafetyConfig safety_config_{};
};

}  // namespace navdog
