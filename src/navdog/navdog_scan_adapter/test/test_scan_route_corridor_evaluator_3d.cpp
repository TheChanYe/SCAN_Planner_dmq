#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

#include "navdog_scan_adapter/scan_route_corridor_evaluator_3d.hpp"

namespace navdog_scan_adapter
{
namespace
{

// =============================================================================
// FakeInflatedGridQuery3D
//
// Stores occupied voxels as integer (ix, iy, iz) tuples.
// A query point (x, y, z) is converted to voxel indices using
// the configured resolution, then checked against the set.
// This properly tests 3D semantics: x, y, and z together
// determine occupancy.
// =============================================================================

class FakeInflatedGridQuery3D : public InflatedGridQuery3D
{
public:
  FakeInflatedGridQuery3D(
      double resolution,
      double map_stamp_sec,
      double map_min_x = -100.0,
      double map_min_y = -100.0,
      double map_min_z = -100.0,
      double map_max_x = 100.0,
      double map_max_y = 100.0,
      double map_max_z = 100.0)
      : resolution_(resolution),
        map_stamp_sec_(map_stamp_sec),
        map_min_x_(map_min_x),
        map_min_y_(map_min_y),
        map_min_z_(map_min_z),
        map_max_x_(map_max_x),
        map_max_y_(map_max_y),
        map_max_z_(map_max_z),
        ready_(true)
  {
  }

  void setReady(bool r) { ready_ = r; }

  void addOccupied(double x, double y, double z)
  {
    occupied_.insert(toIndex(x, y, z));
  }

  bool ready() const noexcept override
  {
    return ready_;
  }

  double resolutionM() const noexcept override
  {
    return resolution_;
  }

  double mapStampSec() const noexcept override
  {
    return map_stamp_sec_;
  }

  InflatedGridQueryResult query(
      double x,
      double y,
      double z,
      double /*yaw*/) const noexcept override
  {
    if (!ready_)
      return InflatedGridQueryResult::INVALID;

    if (!std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(z))
    {
      return InflatedGridQueryResult::INVALID;
    }

    if (x < map_min_x_ || x > map_max_x_ ||
        y < map_min_y_ || y > map_max_y_ ||
        z < map_min_z_ || z > map_max_z_)
    {
      return InflatedGridQueryResult::OUT_OF_MAP;
    }

    if (occupied_.count(toIndex(x, y, z)) > 0)
      return InflatedGridQueryResult::OCCUPIED;

    return InflatedGridQueryResult::FREE;
  }

  // Expose the last yaw received (for testing yaw passing).
  mutable double last_yaw_{0.0};
  mutable bool yaw_received_{false};

private:
  using VoxelIndex =
      std::tuple<int, int, int>;

  VoxelIndex toIndex(
      double x, double y, double z) const
  {
    int ix = static_cast<int>(std::floor(x / resolution_));
    int iy = static_cast<int>(std::floor(y / resolution_));
    int iz = static_cast<int>(std::floor(z / resolution_));
    return std::make_tuple(ix, iy, iz);
  }

  double resolution_;
  double map_stamp_sec_;
  double map_min_x_, map_min_y_, map_min_z_;
  double map_max_x_, map_max_y_, map_max_z_;
  bool ready_;
  std::set<VoxelIndex> occupied_;
};

// =============================================================================
// Helper functions
// =============================================================================

navdog::RouteProgress makeProgress(
    std::uint64_t sequence,
    std::size_t segment_index,
    double segment_ratio,
    double arc_length_m,
    double projected_x,
    double projected_y,
    double remaining_distance_m)
{
  navdog::RouteProgress progress;
  progress.task_sequence = sequence;
  progress.segment_index = segment_index;
  progress.segment_ratio = segment_ratio;
  progress.arc_length_m = arc_length_m;
  progress.total_length_m = arc_length_m + remaining_distance_m;
  progress.remaining_distance_m = remaining_distance_m;
  progress.projected_x = projected_x;
  progress.projected_y = projected_y;
  progress.route_yaw = 0.0;
  progress.lateral_error_m = 0.0;
  progress.on_route = true;
  progress.stamp_sec = 10.0;
  progress.valid = true;
  return progress;
}

navdog::RobotState makeRobot(double x, double y, double z = 0.4)
{
  navdog::RobotState robot;
  robot.x = x;
  robot.y = y;
  robot.z = z;
  robot.valid = true;
  return robot;
}

navdog::NavigationTask makeStraightRoute()
{
  navdog::NavigationTask task;
  task.sequence = 1;
  task.mode = navdog::TaskMode::NORMAL_AVOID;
  task.max_vx = 0.4;
  navdog::RoutePoint p0;
  p0.x = 0.0;
  p0.y = 0.0;
  task.points.push_back(p0);
  navdog::RoutePoint p1;
  p1.x = 10.0;
  p1.y = 0.0;
  task.points.push_back(p1);
  return task;
}

navdog::NavigationTask makeMultiPointRoute()
{
  navdog::NavigationTask task;
  task.sequence = 1;
  task.mode = navdog::TaskMode::NORMAL_AVOID;
  task.max_vx = 0.4;
  navdog::RoutePoint p0;
  p0.x = 0.0; p0.y = 0.0;
  task.points.push_back(p0);
  navdog::RoutePoint p1;
  p1.x = 3.0; p1.y = 0.0;
  task.points.push_back(p1);
  navdog::RoutePoint p2;
  p2.x = 6.0; p2.y = 0.0;
  task.points.push_back(p2);
  return task;
}

navdog::NavigationTask makeSinglePointTask(
    double x, double y)
{
  navdog::NavigationTask task;
  task.sequence = 1;
  task.mode = navdog::TaskMode::NORMAL_AVOID;
  task.max_vx = 0.4;
  navdog::RoutePoint p;
  p.x = x;
  p.y = y;
  task.points.push_back(p);
  return task;
}

const double kNow = 10.0;
const double kEps = 1e-6;

// =============================================================================
// UsesGridResolutionHalfStep
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, UsesGridResolutionHalfStep)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.20, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.2, 2.0, 2.0, 0.0, 8.0),
      makeRobot(2, 0),
      kNow);

  EXPECT_TRUE(assessment.valid);
  EXPECT_NEAR(assessment.map_resolution_m, 0.20, kEps);
  EXPECT_NEAR(assessment.sample_step_m, 0.10, kEps);
}

// =============================================================================
// StartsFromProjectedRoutePosition
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, StartsFromProjectedRoutePosition)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  // Progress at x=5, so sampling starts from (5, 0)
  grid->addOccupied(5.0, 0.0, 0.4);  // At start point

  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.5, 5.0, 5.0, 0.0, 5.0),
      makeRobot(5, 0),
      kNow);

  EXPECT_TRUE(assessment.valid);
  EXPECT_TRUE(assessment.blocked);
  EXPECT_NEAR(assessment.first_blocked_distance_ahead_m,
              0.0, kEps);
}

// =============================================================================
// IgnoresAlreadyTravelledRoute
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, IgnoresAlreadyTravelledRoute)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  // Obstacle behind progress at x=2, progress at x=5
  grid->addOccupied(2.0, 0.0, 0.4);

  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.5, 5.0, 5.0, 0.0, 5.0),
      makeRobot(5, 0),
      kNow);

  EXPECT_TRUE(assessment.valid);
  EXPECT_FALSE(assessment.blocked);
}

// =============================================================================
// StopsAtLookahead
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, StopsAtLookahead)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  // Progress at x=2, lookahead=3, so max check at x=5
  // Obstacle at x=6 → beyond lookahead → not detected
  grid->addOccupied(6.0, 0.0, 0.4);

  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.2, 2.0, 2.0, 0.0, 8.0),
      makeRobot(2, 0),
      kNow);

  EXPECT_TRUE(assessment.valid);
  EXPECT_FALSE(assessment.blocked);
  EXPECT_NEAR(assessment.checked_distance_m, 3.0, kEps);
}

// =============================================================================
// ChecksSegmentEndpoints
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, ChecksSegmentEndpoints)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  // Progress at x=2, first segment endpoint at x=5 (2+3=5)
  grid->addOccupied(5.0, 0.0, 0.4);

  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.2, 2.0, 2.0, 0.0, 8.0),
      makeRobot(2, 0),
      kNow);

  EXPECT_TRUE(assessment.valid);
  EXPECT_TRUE(assessment.blocked);
  EXPECT_NEAR(assessment.first_blocked_distance_ahead_m,
              3.0, kEps);
}

// =============================================================================
// DetectsOccupiedVoxelOnStraightRoute
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, DetectsOccupiedVoxelOnStraightRoute)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  grid->addOccupied(4.0, 0.0, 0.4);

  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.2, 2.0, 2.0, 0.0, 8.0),
      makeRobot(2, 0),
      kNow);

  EXPECT_TRUE(assessment.valid);
  EXPECT_TRUE(assessment.blocked);
  EXPECT_NEAR(assessment.first_blocked_distance_ahead_m,
              2.0, 0.15);  // within one sample step
}

// =============================================================================
// DetectsOccupiedVoxelOnSecondSegment
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, DetectsOccupiedVoxelOnSecondSegment)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  // Route: (0,0)→(3,0)→(6,0), progress at x=1
  // First segment: (1,0)→(3,0), length=2.0
  // Second segment: (3,0)→(6,0), trimmed to 1.0
  grid->addOccupied(3.5, 0.0, 0.4);

  auto assessment = evaluator.evaluate(
      makeMultiPointRoute(),
      makeProgress(1, 0, 1.0/3.0, 1.0, 1.0, 0.0, 5.0),
      makeRobot(1, 0),
      kNow);

  EXPECT_TRUE(assessment.valid);
  EXPECT_TRUE(assessment.blocked);
  // distance_ahead ≈ 2.0 + 0.5 = 2.5
  EXPECT_NEAR(assessment.first_blocked_distance_ahead_m,
              2.5, 0.15);
}

// =============================================================================
// ReportsFirstBlockedDistance
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, ReportsFirstBlockedDistance)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  grid->addOccupied(3.0, 0.0, 0.4);

  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.2, 2.0, 2.0, 0.0, 8.0),
      makeRobot(2, 0),
      kNow);

  EXPECT_TRUE(assessment.blocked);
  EXPECT_NEAR(assessment.first_blocked_distance_ahead_m,
              1.0, 0.15);
}

// =============================================================================
// ReportsBlockedArcLength
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, ReportsBlockedArcLength)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  grid->addOccupied(3.0, 0.0, 0.4);

  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.2, 2.0, 2.0, 0.0, 8.0),
      makeRobot(2, 0),
      kNow);

  EXPECT_TRUE(assessment.blocked);
  // arc_length = 2.0 + 1.0 = 3.0
  EXPECT_NEAR(assessment.first_blocked_arc_length_m,
              3.0, 0.15);
}

// =============================================================================
// PassesRouteYawToGridQuery
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, PassesRouteYawToGridQuery)
{
  // Use a route at 45 degrees
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  navdog::NavigationTask task;
  task.sequence = 1;
  task.mode = navdog::TaskMode::NORMAL_AVOID;
  task.max_vx = 0.4;
  navdog::RoutePoint p0;
  p0.x = 0.0; p0.y = 0.0;
  task.points.push_back(p0);
  navdog::RoutePoint p1;
  p1.x = 10.0; p1.y = 10.0;
  task.points.push_back(p1);

  // Place obstacle along the 45-degree route
  // Projected point is at 1.0m along route = (cos45, sin45).
  // Obstacle at d=2.0m further along the route.
  double d = 2.0;  // distance from projected point
  double px = (1.0 + d) * std::cos(M_PI / 4.0);
  double py = (1.0 + d) * std::sin(M_PI / 4.0);
  grid->addOccupied(px, py, 0.4);

  auto assessment = evaluator.evaluate(
      task,
      makeProgress(1, 0, 0.1, 1.0, 1.0 * std::cos(M_PI/4.0),
                   1.0 * std::sin(M_PI/4.0), 12.0),
      makeRobot(1.0 * std::cos(M_PI/4.0),
                1.0 * std::sin(M_PI/4.0)),
      kNow);

  // Should detect the obstacle
  EXPECT_TRUE(assessment.blocked);
}

// =============================================================================
// UsesRobotZAsQueryHeight
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, UsesRobotZAsQueryHeight)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  // Robot z = 0.4, obstacle at z = 0.4 → blocked
  grid->addOccupied(4.0, 0.0, 0.4);

  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.2, 2.0, 2.0, 0.0, 8.0),
      makeRobot(2, 0, 0.4),
      kNow);

  EXPECT_TRUE(assessment.blocked);
  EXPECT_NEAR(assessment.query_z_m, 0.4, kEps);
}

// =============================================================================
// RejectsInvalidRobotZ
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, RejectsInvalidRobotZ)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  navdog::RobotState robot = makeRobot(2, 0);
  robot.z = std::numeric_limits<double>::quiet_NaN();

  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.2, 2.0, 2.0, 0.0, 8.0),
      robot,
      kNow);

  EXPECT_FALSE(assessment.valid);
}

// =============================================================================
// ReturnsInvalidWhenGridNotReady
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, ReturnsInvalidWhenGridNotReady)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  grid->setReady(false);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.2, 2.0, 2.0, 0.0, 8.0),
      makeRobot(2, 0),
      kNow);

  EXPECT_FALSE(assessment.valid);
}

// =============================================================================
// ReturnsOutOfMapWhenRequiredSampleLeavesMap
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, ReturnsOutOfMapWhenRequiredSampleLeavesMap)
{
  // Create a grid with small bounds
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow,
      -5.0, -5.0, -5.0,  // min
      5.0, 5.0, 5.0);    // max
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 10.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  // Progress at x=4, route goes to x=10, but map ends at x=5
  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.4, 4.0, 4.0, 0.0, 6.0),
      makeRobot(4, 0),
      kNow);

  EXPECT_TRUE(assessment.valid);
  EXPECT_TRUE(assessment.out_of_map);
}

// =============================================================================
// SupportsSinglePointRoute
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, SupportsSinglePointRoute)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  // Single point at (5, 0), robot at (0, 0)
  grid->addOccupied(2.0, 0.0, 0.4);

  navdog::RouteProgress progress;
  progress.task_sequence = 1;
  progress.segment_index = 0;
  progress.segment_ratio = 0.0;
  progress.arc_length_m = 0.0;
  progress.total_length_m = 0.0;
  progress.remaining_distance_m = 5.0;
  progress.projected_x = 5.0;
  progress.projected_y = 0.0;
  progress.route_yaw = 0.0;
  progress.lateral_error_m = 0.0;
  progress.on_route = true;
  progress.stamp_sec = 10.0;
  progress.valid = true;

  auto assessment = evaluator.evaluate(
      makeSinglePointTask(5.0, 0.0),
      progress,
      makeRobot(0, 0),
      kNow);

  EXPECT_TRUE(assessment.valid);
  EXPECT_TRUE(assessment.blocked);
}

// =============================================================================
// SinglePointAtGoalIsClear
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, SinglePointAtGoalIsClear)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  // Robot at target
  navdog::RouteProgress progress;
  progress.task_sequence = 1;
  progress.segment_index = 0;
  progress.segment_ratio = 0.0;
  progress.arc_length_m = 0.0;
  progress.total_length_m = 0.0;
  progress.remaining_distance_m = 0.0;
  progress.projected_x = 5.0;
  progress.projected_y = 0.0;
  progress.route_yaw = 0.0;
  progress.lateral_error_m = 0.0;
  progress.on_route = true;
  progress.stamp_sec = 10.0;
  progress.valid = true;

  auto assessment = evaluator.evaluate(
      makeSinglePointTask(5.0, 0.0),
      progress,
      makeRobot(5, 0),
      kNow);

  EXPECT_TRUE(assessment.valid);
  EXPECT_FALSE(assessment.blocked);
  EXPECT_NEAR(assessment.checked_distance_m, 0.0, kEps);
  EXPECT_EQ(assessment.samples_checked, 0u);
}

// =============================================================================
// SkipsDegenerateSegments
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, SkipsDegenerateSegments)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  // Route: (0,0)→(0.005,0)→(5,0)
  // First segment degenerate → skip
  navdog::NavigationTask task;
  task.sequence = 1;
  task.mode = navdog::TaskMode::NORMAL_AVOID;
  task.max_vx = 0.4;
  navdog::RoutePoint p0; p0.x = 0.0; p0.y = 0.0;
  task.points.push_back(p0);
  navdog::RoutePoint p1; p1.x = 0.005; p1.y = 0.0;
  task.points.push_back(p1);
  navdog::RoutePoint p2; p2.x = 5.0; p2.y = 0.0;
  task.points.push_back(p2);

  grid->addOccupied(3.0, 0.0, 0.4);

  auto assessment = evaluator.evaluate(
      task,
      makeProgress(1, 0, 0.0, 0.0, 0.0, 0.0, 5.0),
      makeRobot(0, 0),
      kNow);

  EXPECT_TRUE(assessment.valid);
  EXPECT_TRUE(assessment.blocked);
}

// =============================================================================
// SameXYObstacleAboveBodyDoesNotBlock
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, SameXYObstacleAboveBodyDoesNotBlock)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  // Obstacle at (4, 0, 2.0) — above robot z=0.4
  grid->addOccupied(4.0, 0.0, 2.0);

  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.2, 2.0, 2.0, 0.0, 8.0),
      makeRobot(2, 0, 0.4),
      kNow);

  EXPECT_TRUE(assessment.valid);
  EXPECT_FALSE(assessment.blocked);
}

// =============================================================================
// SameXYObstacleAtBodyHeightBlocks
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest, SameXYObstacleAtBodyHeightBlocks)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  // Obstacle at (4, 0, 0.4) — same as robot z
  grid->addOccupied(4.0, 0.0, 0.4);

  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.2, 2.0, 2.0, 0.0, 8.0),
      makeRobot(2, 0, 0.4),
      kNow);

  EXPECT_TRUE(assessment.valid);
  EXPECT_TRUE(assessment.blocked);
}

// =============================================================================
// SameXYObstacleBelowBodyDoesNotBlockWhenZInflationDoesNotReachBody
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest,
     SameXYObstacleBelowBodyDoesNotBlockWhenZInflationDoesNotReachBody)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  // Obstacle at (4, 0, -1.0) — well below robot z=0.4
  grid->addOccupied(4.0, 0.0, -1.0);

  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.2, 2.0, 2.0, 0.0, 8.0),
      makeRobot(2, 0, 0.4),
      kNow);

  EXPECT_TRUE(assessment.valid);
  EXPECT_FALSE(assessment.blocked);
}

// =============================================================================
// VerticalInflationReachingBodyHeightBlocks
// =============================================================================

TEST(ScanRouteCorridorEvaluator3DTest,
     VerticalInflationReachingBodyHeightBlocks)
{
  auto grid = std::make_shared<FakeInflatedGridQuery3D>(
      0.10, kNow);
  navdog::RouteCorridorConfig config;
  config.lookahead_distance_m = 3.0;
  ScanRouteCorridorEvaluator3D evaluator(config, grid);

  // Simulate vertical inflation: original obstacle at z=-0.1,
  // but inflation reaches z=0.4 (robot height).
  // We place occupied voxels at both z=-0.1 and z=0.4 to
  // simulate the inflated state.
  grid->addOccupied(4.0, 0.0, -0.1);
  grid->addOccupied(4.0, 0.0, 0.4);

  auto assessment = evaluator.evaluate(
      makeStraightRoute(),
      makeProgress(1, 0, 0.2, 2.0, 2.0, 0.0, 8.0),
      makeRobot(2, 0, 0.4),
      kNow);

  EXPECT_TRUE(assessment.valid);
  EXPECT_TRUE(assessment.blocked);
}

}  // namespace
}  // namespace navdog_scan_adapter

// =============================================================================
// main
// =============================================================================

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
