#pragma once

#include "navdog_scan_adapter/inflated_grid_query_3d.hpp"

#include <navdog_core/types.hpp>

#include <memory>

namespace navdog_scan_adapter
{

// ScanObstacleSummaryEvaluator3D
// 基于膨胀地图向前/左/右/后四个扇区发射射线来评估机器人四周最近障碍物距离，
// 供安全监控/提示使用。
class ScanObstacleSummaryEvaluator3D
{
public:
  // Config：各方向扇区的最大探测距离、半角度以及每个扇区内发射的射线数量。
  struct Config
  {
    double front_range_m{2.0};        // 前方扇区最大探测距离(m)
    double side_range_m{1.0};         // 左/右方扇区最大探测距离(m)
    double rear_range_m{1.0};         // 后方扇区最大探测距离(m)
    double front_half_angle_deg{30.0};  // 前方扇区半角度(度)
    double side_half_angle_deg{30.0};   // 左/右方扇区半角度(度)
    double rear_half_angle_deg{30.0};   // 后方扇区半角度(度)
    int rays_per_sector{7};           // 每个扇区内均匀分布的射线数量
  };

  // 构造函数：保存扇区配置与膨胀地图查询接口。
  ScanObstacleSummaryEvaluator3D(
      const Config& config,
      const std::shared_ptr<InflatedGridQuery3D>& grid,
      double query_z_offset_m = 0.0);

  // evaluate：计算机器人当前位姿下前/左/右/后四个扇区内的最近障碍物距离汇总。
  // 输入：robot - 机器人位姿；now_sec - 当前时间戳。
  // 输出：ObstacleSummary（各方向最近距离、时间戳与有效标志）。
  navdog::ObstacleSummary evaluate(
      const navdog::RobotState& robot,
      double now_sec) const;

private:
  struct SectorResult
  {
    double nearest;
    bool valid{false};
  };

  // evaluateSector：在一个扇形扇区内均匀发射多条射线，沿每条射线按步长递增距离采样，
  // 只有遇到OCCUPIED才记录命中距离；OUT_OF_MAP/INVALID停止该射线但不伪装成
  // 障碍，整个扇区没有有效查询时返回valid=false。
  // 输入：robot - 机器人位姿；center_angle - 扇区中心方向（相对机体朝向）；
  //       half_angle - 半角度（弧度）；range - 最大探测距离。
  SectorResult evaluateSector(
      const navdog::RobotState& robot,
      double center_angle,
      double half_angle,
      double range) const;

  Config config_{};                     // 扇区配置
  std::shared_ptr<InflatedGridQuery3D> grid_{};  // 底层膨胀地图查询接口
  double query_z_offset_m_{0.0};
};

}  // namespace navdog_scan_adapter
