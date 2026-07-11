#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace navdog
{

enum class RouteProgressResult : std::uint8_t
{
  IDLE = 0,
  VALID,
  WAITING_FOR_ROBOT,
  INVALID_TIME,
  INVALID_CONFIG,
  INVALID_TASK
};

struct RouteProgressOutput
{
  RouteProgressResult result{
      RouteProgressResult::IDLE};

  RouteProgress progress{};
};

class RouteProgressTracker
{
public:
  explicit RouteProgressTracker(
      const RouteProgressConfig& config =
          RouteProgressConfig{});

  void reset() noexcept;

  RouteProgressOutput update(
      const NavigationTask& task,
      const RobotState& robot,
      double now_sec);

  bool initialized() const noexcept;

private:
  struct Segment
  {
    std::size_t original_index{0};

    double x0{0.0};
    double y0{0.0};
    double x1{0.0};
    double y1{0.0};

    double dx{0.0};
    double dy{0.0};
    double length{0.0};

    double cumulative_start_m{0.0};
  };

  struct ProjectionCandidate
  {
    bool valid{false};

    std::size_t segment_vector_index{0};
    std::size_t original_segment_index{0};

    double ratio{0.0};
    double arc_length_m{0.0};

    double projected_x{0.0};
    double projected_y{0.0};

    double distance_sq{0.0};
    double route_yaw{0.0};
  };

  bool isConfigValid() const noexcept;

  bool isTaskUsable(
      const NavigationTask& task) const noexcept;

  bool isRobotUsable(
      const RobotState& robot) const noexcept;

  bool rebuildRoute(
      const NavigationTask& task);

  ProjectionCandidate findInitialProjection(
      const RobotState& robot) const noexcept;

  ProjectionCandidate findForwardProjection(
      const RobotState& robot) const noexcept;

  ProjectionCandidate projectToSegment(
      const Segment& segment,
      std::size_t segment_vector_index,
      const RobotState& robot,
      double minimum_arc_length_m) const noexcept;

  bool isBetterCandidate(
      const ProjectionCandidate& candidate,
      const ProjectionCandidate& best) const noexcept;

  RouteProgress makeProgress(
      const ProjectionCandidate& candidate,
      const RobotState& robot,
      double now_sec) const noexcept;

  RouteProgress makeSinglePointProgress(
      const NavigationTask& task,
      const RobotState& robot,
      double now_sec) const noexcept;

  static double clamp(
      double value,
      double lower,
      double upper) noexcept;

  RouteProgressConfig config_{};

  std::uint64_t active_task_sequence_{0};

  std::vector<Segment> segments_;

  bool initialized_{false};
  bool single_point_route_{false};

  double total_length_m_{0.0};
  double current_arc_length_m_{0.0};

  std::size_t current_segment_vector_index_{0};

  RouteProgress last_progress_{};
};

}  // namespace navdog
