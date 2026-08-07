#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Eigen>

namespace scan_planner_dmq
{

// RoutePathTrackerConfig：路径跟踪器参数（预看距离、朝向预看、前搜索上限、接近
// 终点减速距离、位置/朝向比例增益与速度上限）。
struct RoutePathTrackerConfig
{
  double lookahead_distance_m{0.60};
  double heading_lookahead_m{0.40};
  double max_forward_search_m{2.50};
  double near_goal_slowdown_m{0.80};
  double kp_position{0.70};
  double kp_yaw{1.50};
  double max_vx{0.30};
  double max_yaw_rate{0.65};
};

// RoutePathTrackerOutput：单次update()计算得到的速度指令与进度信息。
struct RoutePathTrackerOutput
{
  double vx{0.0};
  double vy{0.0};
  double yaw_rate{0.0};
  double progress_m{0.0};
  double remaining_m{0.0};
  double target_x{0.0};
  double target_y{0.0};
  bool valid{false};
};

// RoutePathTracker：纯几何路径跟踪器。根据机器人当前位置将其投影到路径上得到
// 进度弧长，再沿路径前方取预看点作为目标，通过比例控制输出线/角速度。
class RoutePathTracker
{
public:
  explicit RoutePathTracker(const RoutePathTrackerConfig& config);

  // setPath：设置新的路径点列并预计算累计弧长，要求至少2个点，否则返回false。
  bool setPath(const std::vector<Eigen::Vector3d>& points);
  // reset：清空当前路径与进度状态。
  void reset() noexcept;
  // requestReacquire：标记下一次update()需重新全路径搜索投影点（而非从上次段附近
  // 局部搜索），常用于刚切换路径/刚接管时避免错误匹配到旧进度附近。
  void requestReacquire() noexcept;

  // update：根据机器人当前位置与朝向计算本周期的跟踪速度指令与进度。
  RoutePathTrackerOutput update(
      const Eigen::Vector3d& robot_position,
      double robot_yaw) noexcept;

private:
  // projectProgress：将机器人位置投影到路径上，得到投影处的累计弧长与所在段索引。
  bool projectProgress(
      const Eigen::Vector3d& robot_position,
      double& projected_arc,
      std::size_t& projected_segment) const noexcept;
  // sampleAtArc：根据给定累计弧长arc在路径上插值得到对应位置点（可选输出所在段索引）。
  bool sampleAtArc(
      double arc,
      Eigen::Vector3d& point,
      std::size_t* segment = nullptr) const noexcept;

  RoutePathTrackerConfig config_{};
  std::vector<Eigen::Vector3d> points_;
  std::vector<double> cumulative_length_;
  std::size_t segment_index_{0};
  double progress_m_{0.0};
  bool reacquire_requested_{true};
};

}  // namespace scan_planner_dmq
