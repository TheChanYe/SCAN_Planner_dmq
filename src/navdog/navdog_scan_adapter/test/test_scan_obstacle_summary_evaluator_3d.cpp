#include "navdog_scan_adapter/scan_obstacle_summary_evaluator_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace navdog_scan_adapter
{
namespace
{
class FakeGrid : public InflatedGridQuery3D
{
public:
  bool ready() const noexcept override { return ready_; }
  double resolutionM() const noexcept override { return 0.05; }
  double mapStampSec() const noexcept override { return 1.0; }
  InflatedGridQueryResult query(
      double x, double, double z, double) const noexcept override
  {
    const bool height_matches = !height_sensitive_ ||
        std::abs(z - occupied_z_) < 0.025;
    return x >= 0.5 && height_matches
        ? InflatedGridQueryResult::OCCUPIED
        : InflatedGridQueryResult::FREE;
  }
  bool ready_{true};
  bool height_sensitive_{false};
  double occupied_z_{0.30};
};

TEST(ScanObstacleSummaryEvaluator3DTest, MapNotReadyIsInvalid)
{
  auto grid = std::make_shared<FakeGrid>();
  grid->ready_ = false;
  ScanObstacleSummaryEvaluator3D evaluator({}, grid);
  navdog::RobotState robot{}; robot.valid = true;
  EXPECT_FALSE(evaluator.evaluate(robot, 1.0).valid);
}

TEST(ScanObstacleSummaryEvaluator3DTest, FrontRayFindsInflatedObstacle)
{
  auto grid = std::make_shared<FakeGrid>();
  ScanObstacleSummaryEvaluator3D::Config config{};
  config.rays_per_sector = 1;
  ScanObstacleSummaryEvaluator3D evaluator(config, grid);
  navdog::RobotState robot{}; robot.valid = true;
  const auto result = evaluator.evaluate(robot, 1.0);
  EXPECT_TRUE(result.valid);
  EXPECT_NEAR(result.front_min, 0.5, 0.026);
}

TEST(ScanObstacleSummaryEvaluator3DTest, AppliesBodyQueryZOffset)
{
  auto grid = std::make_shared<FakeGrid>();
  grid->height_sensitive_ = true;
  ScanObstacleSummaryEvaluator3D::Config config{};
  config.rays_per_sector = 1;
  navdog::RobotState robot{};
  robot.valid = true;

  ScanObstacleSummaryEvaluator3D raw_evaluator(config, grid);
  EXPECT_TRUE(std::isinf(raw_evaluator.evaluate(robot, 1.0).front_min));

  ScanObstacleSummaryEvaluator3D body_evaluator(config, grid, 0.30);
  EXPECT_NEAR(body_evaluator.evaluate(robot, 1.0).front_min, 0.5, 0.026);
}
}
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
