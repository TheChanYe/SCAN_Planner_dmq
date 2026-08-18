#include "plan_manage_dmq/reference_trajectory_selector.h"

#include <cmath>
#include <limits>

namespace scan_planner_dmq
{

ReferenceTargetWindow selectForwardReferenceWindow(
    const std::vector<ReferenceTrajectorySample>& samples,
    const Eigen::Vector3d& robot_position,
    double projection_search_arc_m,
    double target_distance_m) noexcept
{
  ReferenceTargetWindow result;
  if (samples.empty() || !robot_position.allFinite() ||
      !std::isfinite(projection_search_arc_m) ||
      !std::isfinite(target_distance_m) ||
      projection_search_arc_m < 0.0 || target_distance_m < 0.0)
    return result;

  double previous_arc = -std::numeric_limits<double>::infinity();
  for (const auto& sample : samples)
  {
    if (!std::isfinite(sample.time_sec) ||
        !std::isfinite(sample.arc_length_m) ||
        !sample.position.allFinite() ||
        sample.arc_length_m + 1e-9 < previous_arc)
      return result;

    previous_arc = sample.arc_length_m;
  }

  double nearest_distance_sq = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < samples.size(); ++i)
  {
    const auto& sample = samples[i];
    if (sample.arc_length_m > projection_search_arc_m + 1e-9)
      break;

    const double distance_sq =
        (sample.position - robot_position).squaredNorm();
    if (distance_sq < nearest_distance_sq)
    {
      nearest_distance_sq = distance_sq;
      result.progress_index = i;
    }
  }

  if (!std::isfinite(nearest_distance_sq))
    return result;

  result.target_index = result.progress_index;
  const double progress_arc = samples[result.progress_index].arc_length_m;
  for (std::size_t i = result.progress_index; i < samples.size(); ++i)
  {
    result.target_index = i;
    if (samples[i].arc_length_m - progress_arc >=
        target_distance_m - 1e-9)
      break;
  }

  result.valid = true;
  return result;
}

}  // namespace scan_planner_dmq
