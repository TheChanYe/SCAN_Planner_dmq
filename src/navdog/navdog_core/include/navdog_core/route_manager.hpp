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
  /** @brief 接收已校验路线并以任务 sequence 初始化进度关联。 */
  bool acceptRoute(std::uint64_t task_sequence,
      const std::vector<navdog_task::RoutePoint>& points);
  bool acceptRoute(std::uint64_t task_sequence,
      std::vector<navdog_task::RoutePoint>&& points);
  bool hasRoute() const noexcept;
  std::uint64_t taskSequence() const noexcept;
  /** @brief 用世界系机器人位置更新只能前进的路线投影；时间必须与 Core 同源。 */
  RouteProgressOutput updateProgress(const RobotState& robot, double now_sec);
  /** @brief 查询累计弧长（米）处的路线点，不改变当前进度。 */
  bool pointAtArcLength(double arc_length_m,
      navdog_task::RoutePoint& output) const noexcept;
  /** @brief 查询从给定弧长向前指定米数的跟随目标，不改变当前进度。 */
  bool forwardTarget(double from_arc_length_m, double forward_distance_m,
      navdog_task::RoutePoint& output) const noexcept;
  const navdog_task::RoutePoint* goal() const noexcept;
  const std::vector<navdog_task::RoutePoint>& route() const noexcept;
  const RouteProgress& progress() const noexcept;
  // Read-only compatibility view; the vector storage remains owned here.
  const NavigationTask& taskView() const noexcept;

private:
  bool canAccept(std::uint64_t task_sequence,
      const std::vector<navdog_task::RoutePoint>& points) const noexcept;
  NavigationTask task_view_{};
  RouteProgressTracker progress_tracker_{};
  RouteProgress last_progress_{};
};

}  // namespace navdog
