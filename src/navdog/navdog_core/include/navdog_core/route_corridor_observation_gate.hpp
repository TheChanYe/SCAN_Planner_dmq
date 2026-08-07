#pragma once

#include "navdog_core/config.hpp"
#include "navdog_core/types.hpp"

#include <cstdint>

namespace navdog
{

// RouteCorridorObservationResult：路径走廊观测评估的结果枚举。
// CLEAR/BLOCKED 为有效结果（走廊畅通/被阻），其余均为各种无效/过时原因：
//   WAITING_FOR_OBSERVATION - 尚未收到观测；TASK_MISMATCH - 观测与当前任务不匹配；
//   STALE_MAP/FUTURE_MAP - 地图时间戳过时/超前；STALE_PROGRESS/FUTURE_PROGRESS - 进度与
//   观测相关联的弧长不匹配；OUT_OF_MAP - 超出地图范围；INVALID_OBSERVATION - 观测数据本身无效；
//   INVALID_TIME/INVALID_CONFIG/INVALID_PROGRESS - 输入时间/配置/进度本身无效。
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

// RouteCorridorObservationOutput：evaluate() 的返回结果，包含评估结果与（仅当
// result为CLEAR/BLOCKED时有效的）走廊评估详情。
struct RouteCorridorObservationOutput
{
  RouteCorridorObservationResult result{
      RouteCorridorObservationResult::IDLE};

  RouteCorridorAssessment assessment{};
};

// RouteCorridorObservationGate：路径走廊观测门控。
// 职责：对 SCAN 提供的走廊局部障碍评估(RouteCorridorAssessment)做一系列合法性检查
// （时间有效性、任务匹配、地图时间戳超时、进度与观测的弧长偏差等），只有全部通过后
// 才输出CLEAR或BLOCKED，否则返回对应的无效原因，供上层（NavigationModeManager等）
// 判断是否可以信任该次评估结果。
class RouteCorridorObservationGate
{
public:
  // 构造函数：保存门控配置（地图超时阈值、最大进度滞后距离等）。
  explicit RouteCorridorObservationGate(
      const RouteCorridorObservationConfig& config =
          RouteCorridorObservationConfig{});

  // evaluate：根据当前路线进度与本周期的走廊观测，评估并返回可信的走廊结果。
  // 输入：current_progress - 当前路线进度；observation - SCAN上报的走廊评估；
  // now_sec - 当前时间。输出：包含结果枚举与（若有效）评估详情的输出结构体。
  RouteCorridorObservationOutput evaluate(
      const RouteProgress& current_progress,
      const RouteCorridorAssessment& observation,
      double now_sec) const;

private:
  // isConfigValid：校验门控配置本身是否合法（超时阈值/最大滞后距离均为有限且合理）。
  bool isConfigValid() const noexcept;
  // isProgressValid：校验路线进度本身是否合法（valid标志/弧长/任务sequence/时间戳）。
  bool isProgressValid(
      const RouteProgress& progress) const noexcept;
  // isObservationNumericValid：校验走廊观测中所有数值字段是否为合理有限数，
  // 并根据blocked标志分别校验阻塞距离字段（阻塞时必须非负有限，畅通时必须为正无穷）。
  bool isObservationNumericValid(
      const RouteCorridorAssessment& observation) const noexcept;

  RouteCorridorObservationConfig config_{};  // 门控配置
};

}  // namespace navdog
