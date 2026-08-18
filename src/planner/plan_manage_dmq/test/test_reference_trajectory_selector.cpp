#include <gtest/gtest.h>

#include <limits>

#include "plan_manage_dmq/reference_trajectory_selector.h"

namespace
{

using scan_planner_dmq::ReferenceTrajectorySample;
using scan_planner_dmq::selectForwardReferenceWindow;

ReferenceTrajectorySample sample(
    double time_sec, double x, double y, double arc_length_m)
{
  return {time_sec, Eigen::Vector3d(x, y, 0.0), arc_length_m};
}

TEST(ReferenceTrajectorySelectorTest, ReturningBranchCannotStealProjection)
{
  const std::vector<ReferenceTrajectorySample> samples{
      sample(0.0, 0.0, 0.0, 0.0),
      sample(1.0, 1.0, 0.0, 1.0),
      sample(2.0, 2.0, 0.0, 2.0),
      sample(3.0, 3.0, 0.0, 3.0),
      sample(4.0, 4.0, 0.0, 4.0),
      sample(5.0, 5.0, 0.0, 5.0),
      sample(6.0, 4.0, 0.2, 6.0),
      sample(7.0, 3.0, 0.2, 7.0),
      sample(8.0, 2.0, 0.2, 8.0),
      sample(9.0, 1.0, 0.2, 9.0)};

  const auto result = selectForwardReferenceWindow(
      samples, Eigen::Vector3d(1.0, 0.05, 0.0), 5.0, 3.0);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.progress_index, 1u);
  EXPECT_EQ(result.target_index, 4u);
}

TEST(ReferenceTrajectorySelectorTest, TargetUsesRouteArcNotEuclideanDistance)
{
  const std::vector<ReferenceTrajectorySample> samples{
      sample(0.0, 0.0, 0.0, 0.0),
      sample(1.0, 1.0, 0.0, 1.0),
      sample(2.0, 2.0, 0.0, 2.0),
      sample(3.0, 2.0, 1.0, 3.0),
      sample(4.0, 1.0, 1.0, 4.0),
      sample(5.0, 0.0, 1.0, 5.0)};

  const auto result = selectForwardReferenceWindow(
      samples, Eigen::Vector3d::Zero(), 0.5, 4.0);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.progress_index, 0u);
  EXPECT_EQ(result.target_index, 4u);
}

TEST(ReferenceTrajectorySelectorTest, RejectsNonMonotonicSamples)
{
  const std::vector<ReferenceTrajectorySample> samples{
      sample(0.0, 0.0, 0.0, 1.0),
      sample(1.0, 1.0, 0.0, 0.5)};

  EXPECT_FALSE(selectForwardReferenceWindow(
      samples, Eigen::Vector3d::Zero(), 2.0, 1.0).valid);
}

TEST(ReferenceTrajectorySelectorTest, ValidatesSamplesBeyondProjectionWindow)
{
  Eigen::Vector3d invalid_position = Eigen::Vector3d::Zero();
  invalid_position.x() = std::numeric_limits<double>::quiet_NaN();
  const std::vector<ReferenceTrajectorySample> samples{
      sample(0.0, 0.0, 0.0, 0.0),
      {1.0, invalid_position, 1.0}};

  EXPECT_FALSE(selectForwardReferenceWindow(
      samples, Eigen::Vector3d::Zero(), 0.5, 1.0).valid);
}

}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
