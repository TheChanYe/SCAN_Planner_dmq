#include "navdog_scan_adapter/scan_obstacle_summary_evaluator_3d.hpp"

#include <gtest/gtest.h>

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
      double x, double, double, double) const noexcept override
  {
    return x >= 0.5 ? InflatedGridQueryResult::OCCUPIED
                    : InflatedGridQueryResult::FREE;
  }
  bool ready_{true};
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
}
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
