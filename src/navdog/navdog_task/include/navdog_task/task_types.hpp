#pragma once

#include <cstdint>
#include <vector>

namespace navdog_task
{

enum class TaskMode : std::uint8_t
{
  NORMAL_AVOID = 1,
  ROUTE_ONLY = 2,
  CHARGING = 3
};

struct RoutePoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  bool has_yaw{false};
};

struct NavigationTask
{
  std::uint64_t sequence{0};
  TaskMode mode{TaskMode::NORMAL_AVOID};
  double max_vx{0.4};
  std::vector<RoutePoint> points;
};

enum class NavigationEventType : std::uint8_t
{
  NONE = 0,
  START_TASK,
  CANCEL_TASK,
  PAUSE,
  RESUME,
  UPDATE_MAX_VX,
  DYNAMIC_OBSTACLE_UPDATE
};

struct NavigationEvent
{
  NavigationEventType type{NavigationEventType::NONE};
  NavigationTask task{};
  double max_vx{0.0};
};

enum class TaskHandleResult : std::uint8_t
{
  NONE = 0,
  STARTED,
  REJECTED_BUSY,
  REJECTED_INVALID_TASK,
  CANCELLED,
  CANCEL_IGNORED,
  PAUSED,
  RESUMED,
  PAUSE_RESUME_IGNORED,
  MAX_VX_UPDATED,
  MAX_VX_UNCHANGED,
  MAX_VX_UPDATE_IGNORED,
  REJECTED_INVALID_MAX_VX,
  UNSUPPORTED_EVENT
};

struct TaskConfig
{
  double default_max_vx{0.4};
  double min_max_vx{0.15};
  double max_max_vx{1.0};
};

struct TaskSession
{
  std::uint64_t sequence{0};
  TaskMode mode{TaskMode::NORMAL_AVOID};
  double max_vx{0.4};
  bool active{false};
  bool paused{false};
};

struct TaskTransition
{
  TaskHandleResult result{TaskHandleResult::NONE};
  TaskSession session{};
  std::vector<RoutePoint> accepted_route;
  bool route_changed{false};
};

}  // namespace navdog_task
