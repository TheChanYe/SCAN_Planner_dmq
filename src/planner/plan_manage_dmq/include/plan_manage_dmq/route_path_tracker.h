#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Eigen>

namespace scan_planner_dmq
{

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

class RoutePathTracker
{
public:
  explicit RoutePathTracker(const RoutePathTrackerConfig& config);

  bool setPath(const std::vector<Eigen::Vector3d>& points);
  void reset() noexcept;
  void requestReacquire() noexcept;

  RoutePathTrackerOutput update(
      const Eigen::Vector3d& robot_position,
      double robot_yaw) noexcept;

private:
  bool projectProgress(
      const Eigen::Vector3d& robot_position,
      double& projected_arc,
      std::size_t& projected_segment) const noexcept;
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
