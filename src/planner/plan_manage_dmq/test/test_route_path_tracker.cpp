#include <gtest/gtest.h>

#include "plan_manage_dmq/route_path_tracker.h"

namespace
{

using scan_planner_dmq::RoutePathTracker;
using scan_planner_dmq::RoutePathTrackerConfig;

TEST(RoutePathTrackerTest, StraightPathCommandsForwardWithoutLateralVelocity)
{
  RoutePathTracker tracker(RoutePathTrackerConfig{});
  ASSERT_TRUE(tracker.setPath({
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0)}));

  const auto output = tracker.update(Eigen::Vector3d::Zero(), 0.0);
  ASSERT_TRUE(output.valid);
  EXPECT_GT(output.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.vy, 0.0);
  EXPECT_NEAR(output.yaw_rate, 0.0, 1e-12);
}

TEST(RoutePathTrackerTest, CrossTrackErrorProducesOneSteeringCommand)
{
  RoutePathTracker tracker(RoutePathTrackerConfig{});
  ASSERT_TRUE(tracker.setPath({
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0)}));

  const auto output = tracker.update(Eigen::Vector3d(0.5, 0.4, 0.0), 0.0);
  ASSERT_TRUE(output.valid);
  EXPECT_DOUBLE_EQ(output.vy, 0.0);
  EXPECT_LT(output.yaw_rate, 0.0);
}

TEST(RoutePathTrackerTest, ReacquiresRemainingRouteAfterAvoidance)
{
  RoutePathTracker tracker(RoutePathTrackerConfig{});
  ASSERT_TRUE(tracker.setPath({
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0),
      Eigen::Vector3d(4.0, 0.0, 0.0),
      Eigen::Vector3d(6.0, 0.0, 0.0)}));

  ASSERT_TRUE(tracker.update(Eigen::Vector3d(0.5, 0.0, 0.0), 0.0).valid);
  tracker.requestReacquire();
  const auto output = tracker.update(Eigen::Vector3d(5.0, 0.3, 0.0), 0.0);

  ASSERT_TRUE(output.valid);
  EXPECT_GT(output.progress_m, 4.5);
  EXPECT_LT(output.remaining_m, 1.5);
}

TEST(RoutePathTrackerTest, ReacquireNeverSelectsAnAlreadyPassedSegment)
{
  RoutePathTracker tracker(RoutePathTrackerConfig{});
  ASSERT_TRUE(tracker.setPath({
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 5.0, 0.0)}));

  const auto before = tracker.update(
      Eigen::Vector3d(4.0, 0.0, 0.0), 0.0);
  ASSERT_TRUE(before.valid);
  tracker.requestReacquire();
  const auto after = tracker.update(
      Eigen::Vector3d(0.1, 0.0, 0.0), 0.0);

  ASSERT_TRUE(after.valid);
  EXPECT_GE(after.progress_m, before.progress_m);
  EXPECT_GT(after.progress_m, 9.0);
}

TEST(RoutePathTrackerTest, RejectsDegeneratePath)
{
  RoutePathTracker tracker(RoutePathTrackerConfig{});
  EXPECT_FALSE(tracker.setPath({
      Eigen::Vector3d(1.0, 1.0, 0.0),
      Eigen::Vector3d(1.0, 1.0, 0.0)}));
}

}  // namespace
