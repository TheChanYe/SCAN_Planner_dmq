#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace navdog
{

// RouteProgressResult：路线进度更新结果枚举。VALID为成功；
// WAITING_FOR_ROBOT表示机器人位姿尚不可用；其余为时间/配置/任务本身无效。
enum class RouteProgressResult : std::uint8_t
{
  IDLE = 0,
  VALID,
  WAITING_FOR_ROBOT,
  INVALID_TIME,
  INVALID_CONFIG,
  INVALID_TASK
};

// RouteProgressOutput：update() 的返回结果，包含结果枚举与（仅当VALID时有效的）进度详情。
struct RouteProgressOutput
{
  RouteProgressResult result{
      RouteProgressResult::IDLE};

  RouteProgress progress{};
};

// RouteProgressTracker：将机器人位置投影到折线路线上，计算单调不回退的累计弧长进度。
// 仅供 RouteManager 内部使用。
class RouteProgressTracker
{
  friend class RouteManager;

private:
  /**
   * @brief RouteManager 私有使用的单调投影器。
   * 首次投影搜索全路线以支持任意起点；后续只向前搜索，以免定位噪声让
   * 路线进度回退并使控制目标跳回已通过路段。
   */
  explicit RouteProgressTracker(
      const RouteProgressConfig& config =
          RouteProgressConfig{});

  // reset：清空当前活动任务sequence、已构建的直线段列表、当前弧长与当前段索引等内部状态。
  void reset() noexcept;

  /** @brief 更新世界系投影与剩余距离；时间或输入无效时返回不可控制结果。 */
  RouteProgressOutput update(
      const NavigationTask& task,
      const RobotState& robot,
      double now_sec);

  // initialized：返回当前是否已完成过首次全路线搜索初始化。
  bool initialized() const noexcept;

private:
  // Segment：预处理后的直线段，缓存两端点、方向向量、长度与起始累计弧长，避免重复计算。
  struct Segment
  {
    std::size_t original_index{0};   // 在原始路线点集中的起始点索引

    double x0{0.0};   // 段起点x
    double y0{0.0};   // 段起点y
    double x1{0.0};   // 段终点x
    double y1{0.0};   // 段终点y

    double dx{0.0};      // 归一化方向向量x分量
    double dy{0.0};      // 归一化方向向量y分量
    double length{0.0};  // 段长度

    double cumulative_start_m{0.0};  // 该段起点处的累计弧长
  };

  // ProjectionCandidate：一个候选投影结果，记录投影到哪一段、段内比例、对应弧长、
  // 投影点坐标、到机器人的平方距离与路线朝向，用于在多个候选中选最优。
  struct ProjectionCandidate
  {
    bool valid{false};

    std::size_t segment_vector_index{0};    // 在segments_中的下标
    std::size_t original_segment_index{0};  // 对应原始路线点集中的起始点索引

    double ratio{0.0};         // 投影点在该段上的比例[0,1]
    double arc_length_m{0.0};  // 对应的累计弧长

    double projected_x{0.0};  // 投影点x
    double projected_y{0.0};  // 投影点y

    double distance_sq{0.0};  // 机器人到投影点的平方距离
    double route_yaw{0.0};    // 该段的方向角
  };

  // isConfigValid：校验进度追踪器配置本身是否合法。
  bool isConfigValid() const noexcept;

  // isTaskUsable：校验当前任务是否可用于进度计算（sequence、路线点合法性等）。
  bool isTaskUsable(
      const NavigationTask& task) const noexcept;

  // isRobotUsable：校验机器人位姿是否为合理有限数。
  bool isRobotUsable(
      const RobotState& robot) const noexcept;

  // rebuildRoute：当任务sequence发生变化时，重新根据路线点集构建 segments_ 列表（计算每段
  // 方向、长度与累计弧长），并重置内部弧长/初始化状态。
  bool rebuildRoute(
      const NavigationTask& task);

  // findInitialProjection：首次投影，遍历全部直线段找到距机器人最近的投影点（支持任意起点）。
  ProjectionCandidate findInitialProjection(
      const RobotState& robot) const noexcept;

  // findForwardProjection：非首次调用时，仅从当前段开始向前搜索最优投影，避免定位噪声
  // 导致进度回退到已走过的旧段。
  ProjectionCandidate findForwardProjection(
      const RobotState& robot) const noexcept;

  // projectToSegment：将机器人投影到指定直线段上，并且返回的弧长必须落在
  // [minimum_arc_length_m, maximum_arc_length_m] 内。
  ProjectionCandidate projectToSegment(
      const Segment& segment,
      std::size_t segment_vector_index,
      const RobotState& robot,
      double minimum_arc_length_m,
      double maximum_arc_length_m) const noexcept;

  // isBetterCandidate：比较两个投影候选，判断 candidate 是否优于当前最优 best
  // （主要按距离平方比较）。
  bool isBetterCandidate(
      const ProjectionCandidate& candidate,
      const ProjectionCandidate& best) const noexcept;

  // makeProgress：由选定的投影候选构造完整的 RouteProgress结构体（弧长、总长、剩余距离、
  // 横向误差、朝向误差等）。
  RouteProgress makeProgress(
      const ProjectionCandidate& candidate,
      const RobotState& robot,
      double now_sec) const noexcept;

  // makeSinglePointProgress：单点路线（只有一个点）时的特殊进度构造，无需投影直接用
  // 机器人到该点的直线距离作为剩余距离。
  RouteProgress makeSinglePointProgress(
      const NavigationTask& task,
      const RobotState& robot,
      double now_sec) const noexcept;

  // clamp：通用数值限幅工具函数。
  static double clamp(
      double value,
      double lower,
      double upper) noexcept;

  RouteProgressConfig config_{};  // 进度追踪器配置

  std::uint64_t active_task_sequence_{0};  // 当前已构建路线的任务sequence

  std::vector<Segment> segments_;  // 预处理后的直线段列表

  bool initialized_{false};        // 是否已完成首次全路线搜索
  bool single_point_route_{false}; // 当前路线是否为单点路线

  double total_length_m_{0.0};         // 路线总长（米）
  double current_arc_length_m_{0.0};   // 当前累计弧长（单调不回退）
  double forward_arc_budget_m_{0.0};   // 机器人真实平移允许到达的最大弧长

  double last_robot_x_{0.0};
  double last_robot_y_{0.0};
  bool have_last_robot_position_{false};

  std::size_t current_segment_vector_index_{0};  // 当前所在段在segments_中的下标

  RouteProgress last_progress_{};  // 最近一次成功的进度结果
};

}  // namespace navdog
