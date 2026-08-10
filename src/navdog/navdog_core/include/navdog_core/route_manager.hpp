#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/route_progress_tracker.hpp"
#include "navdog_core/types.hpp"

#include <cstdint>
#include <vector>

namespace navdog
{

class RouteManager
{
public:
  /** @brief 纯 C++ 原始路线与单调进度所有者；Coordinator 调用它但它不发布 ROS。 */
  explicit RouteManager(const RouteProgressConfig& config = RouteProgressConfig{});

  /** @brief 清除路线和进度，供取消或失败清理使用。 */
  void reset() noexcept;
  /** @brief 接收已校验路线（拷贝版）并以任务 sequence 初始化进度关联，成功则返回true。 */
  bool acceptRoute(std::uint64_t task_sequence,
      const std::vector<navdog_task::RoutePoint>& points);
  /** @brief 接收已校验路线（移动版，避免拷贝）并以任务 sequence 初始化进度关联。 */
  bool acceptRoute(std::uint64_t task_sequence,
      std::vector<navdog_task::RoutePoint>&& points);
  /** @brief 返回当前是否持有一条有效路线。 */
  bool hasRoute() const noexcept;
  /** @brief 返回当前路线关联的任务 sequence（未有路线时为0）。 */
  std::uint64_t taskSequence() const noexcept;
  /** @brief 用世界系机器人位置更新只能前进的路线投影；时间必须与 Core 同源。 */
  RouteProgressOutput updateProgress(const RobotState& robot, double now_sec);
  /** @brief 查询累计弧长（米）处的路线点，不改变当前进度。未命中则返回false。 */
  bool pointAtArcLength(double arc_length_m,
      navdog_task::RoutePoint& output) const noexcept;
  /** @brief 查询从给定弧长向前指定米数的跟随目标点（不改变当前进度），用于 pure pursuit 前瞻。 */
  bool forwardTarget(double from_arc_length_m, double forward_distance_m,
      navdog_task::RoutePoint& output) const noexcept;
  /** @brief 按XY弧长采样前方路线高度，判断是否存在达到阈值的正向抬升。 */
  RouteElevationAssessment assessElevation(
      const RouteProgress& progress,
      const StairUpConfig& config) const noexcept;
  /** @brief 返回路线终点指针，无路线时为nullptr。 */
  const navdog_task::RoutePoint* goal() const noexcept;
  /** @brief 返回当前路线点集的只读引用。 */
  const std::vector<navdog_task::RoutePoint>& route() const noexcept;
  /** @brief 返回最新一次 updateProgress 的进度结果。 */
  const RouteProgress& progress() const noexcept;
  // Read-only compatibility view; the vector storage remains owned here.
  /** @brief 返回用于兼容旧接口的任务视图（NavigationTask），底层数据仍由本类持有。 */
  const NavigationTask& taskView() const noexcept;

private:
  // canAccept：判断能否接受这条新路线：任务sequence非零且点集非空。
  bool canAccept(std::uint64_t task_sequence,
      const std::vector<navdog_task::RoutePoint>& points) const noexcept;
  NavigationTask task_view_{};                 // 兼容旧接口的任务视图（包含路线点集）
  RouteProgressTracker progress_tracker_{};    // 单调进度追踪器
  RouteProgress last_progress_{};              // 最近一次成功的进度结果缓存
};

}  // namespace navdog
