#pragma once

#include <cstdint>
#include <vector>

/**
 * 这个文件定义了导航任务管理器的相关类型，包括任务模式、路点、导航任务、导航事件、任务处理结果、任务配置、任务会话和任务转换结构体。
 * 这些类型用于描述导航任务的状态、事件和处理结果，便于任务管理器进行任务的调度和处理。
 */
namespace navdog_task
{

/**
 * @brief 任务模式，包括普通避障、仅路径规划和充电模式
 */
enum class TaskMode : std::uint8_t
{
  NORMAL_AVOID = 1,
  ROUTE_ONLY = 2,
  CHARGING = 3
};

/**
 * @brief 全局 Route 路点；x/y/z 单位为米，yaw 单位为 rad，均在 Runtime 约定的世界坐标系。
 */
struct RoutePoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  bool has_yaw{false};
};

/**
 * @brief 导航任务。sequence 是 TaskManager 分配的单调会话关联号，max_vx 单位 m/s。
 */
struct NavigationTask
{
  std::uint64_t sequence{0};
  TaskMode mode{TaskMode::NORMAL_AVOID};
  double max_vx{0.4};
  std::vector<RoutePoint> points;
};

/**
 * @brief 导航事件类型，包括无、开始任务、取消任务、暂停、恢复、更新最大速度、动态障碍物更新
 */
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
/**
 * @brief 已由协议层解析的纯 C++ 事件；任务层刻意不知道 ROS、MQTT 或 JSON。
 */
struct NavigationEvent
{
  NavigationEventType type{NavigationEventType::NONE};
  NavigationTask task{};
  double max_vx{0.0};
};

/**
 * @brief 任务处理结果，包括无、已开始、拒绝忙碌、拒绝无效任务、已取消、取消忽略、已暂停、已恢复、暂停/恢复忽略、最大速度已更新、最大速度未改变、最大速度更新忽略、拒绝无效最大速度、不支持的事件
 */
enum class TaskHandleResult : std::uint8_t
{
  NONE = 0,                             // 无
  STARTED,                              // 已开始
  REJECTED_BUSY,                        // 拒绝忙碌
  REJECTED_INVALID_TASK,                // 拒绝无效任务
  CANCELLED,                            // 已取消
  CANCEL_IGNORED,                       // 取消忽略
  PAUSED,                               // 已暂停
  RESUMED,                              // 已恢复
  PAUSE_RESUME_IGNORED,                 // 暂停/恢复忽略
  MAX_VX_UPDATED,                       // 最大速度已更新
  MAX_VX_UNCHANGED,                     // 最大速度未改变
  MAX_VX_UPDATE_IGNORED,                // 最大速度更新忽略
  REJECTED_INVALID_MAX_VX,              // 拒绝无效最大速度
  UNSUPPORTED_EVENT                     // 不支持的事件
};

/**
 * @brief 任务配置，包含默认最大速度、最小最大速度和最大最大速度
 */
struct TaskConfig
{
  double default_max_vx{0.4};
  double min_max_vx{0.15};
  double max_max_vx{1.0};
};
/**
 * @brief 任务会话，包含任务序列号、任务模式和最大速度
 */
struct TaskSession
{
  std::uint64_t sequence{0};
  TaskMode mode{TaskMode::NORMAL_AVOID};
  double max_vx{0.4};
  bool active{false};
  bool paused{false};
};
/**
 * @brief 任务转换结构体，包含处理结果、任务会话和已接受的路径和路径是否已改变
 */
struct TaskTransition
{
  TaskHandleResult result{TaskHandleResult::NONE};
  TaskSession session{};
  std::vector<RoutePoint> accepted_route;
  bool route_changed{false};
};

}  // namespace navdog_task
