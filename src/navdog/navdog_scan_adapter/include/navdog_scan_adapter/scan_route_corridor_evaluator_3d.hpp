#pragma once

#include "navdog_scan_adapter/inflated_grid_query_3d.hpp"

#include <navdog_core/config.hpp>
#include <navdog_core/types.hpp>

#include <memory>

namespace navdog_scan_adapter
{

// ScanRouteCorridorEvaluator3D
// 基于膨胀地图对机器人当前行进方向前方的路径走廊（环形/矩形采样区域）做障碍评估，
// 供navdog_core层的RouteCorridorObservationGate消费。
class ScanRouteCorridorEvaluator3D
{
public:
  // 构造函数：保存走廊评估配置与膨胀地图查询接口。
  ScanRouteCorridorEvaluator3D(
      const navdog::RouteCorridorConfig& config,
      const std::shared_ptr<InflatedGridQuery3D>& grid);

  // evaluate：对机器人当前位置前方的路径走廊做障碍评估。
  // 输入：task - 导航任务（提供路线点）；progress - 当前路线进度（确定走廊起点）；
  //       robot - 机器人位姿；now_sec - 当前时间戳。
  // 输出：RouteCorridorAssessment（走廊内障碍评估结果）。
  navdog::RouteCorridorAssessment evaluate(
      const navdog::NavigationTask& task,
      const navdog::RouteProgress& progress,
      const navdog::RobotState& robot,
      double now_sec) const;

private:
  navdog::RouteCorridorConfig config_;         // 走廊评估配置
  std::shared_ptr<InflatedGridQuery3D> grid_;  // 底层膨胀地图查询接口
};

}  // namespace navdog_scan_adapter
