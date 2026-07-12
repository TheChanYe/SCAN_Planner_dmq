#include <gtest/gtest.h>

#include "navdog_scan_adapter/scan_local_planner_adapter.hpp"

#include <navdog_core/config.hpp>
#include <navdog_core/types.hpp>

#include <memory>
#include <type_traits>

namespace navdog_scan_adapter
{
namespace
{

// =============================================================================
// Helpers
// =============================================================================

class FakeInflatedGridQuery3D : public InflatedGridQuery3D
{
public:
  bool ready() const noexcept override
  {
    return true;
  }

  double resolutionM() const noexcept override
  {
    return 0.05;
  }

  double mapStampSec() const noexcept override
  {
    return 0.0;
  }

  InflatedGridQueryResult query(
      double /*x*/,
      double /*y*/,
      double /*z*/,
      double /*yaw*/) const noexcept override
  {
    return InflatedGridQueryResult::FREE;
  }
};

// Compile-time detector for the time_from_start_sec member.
template <typename T, typename = void>
struct HasTimeFromStartSec : std::false_type
{
};

template <typename T>
struct HasTimeFromStartSec<
    T,
    decltype(void(std::declval<T>().time_from_start_sec))>
    : std::true_type
{
};

// =============================================================================
// ScanLocalPlannerAdapterCompiles
// =============================================================================

TEST(ScanLocalPlannerAdapterTest, ScanLocalPlannerAdapterCompiles)
{
  navdog::PlannerTriggerConfig config{};
  auto grid_query = std::make_shared<FakeInflatedGridQuery3D>();
  auto planner_manager =
      std::make_shared<scan_planner::SCANPlannerManager>();

  {
    ScanLocalPlannerAdapter adapter(
        config, grid_query, planner_manager);

    EXPECT_EQ(
        adapter.localPlanState(
            navdog::NavigationMode::NONE, 0u, 0u),
        navdog::LocalPlanState::IDLE);
  }
}

// =============================================================================
// RoutePointHasNoFakeTrajectoryTimeUsage
// =============================================================================

TEST(ScanLocalPlannerAdapterTest, RoutePointHasNoFakeTrajectoryTimeUsage)
{
  // A RoutePoint must not carry a trajectory-relative timestamp; only
  // TimedTrajectoryPoint has time_from_start_sec.
  EXPECT_FALSE((HasTimeFromStartSec<navdog::RoutePoint>::value));
  EXPECT_TRUE(
      (HasTimeFromStartSec<navdog::TimedTrajectoryPoint>::value));
}

}  // namespace
}  // namespace navdog_scan_adapter

// =============================================================================
// main
// =============================================================================

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
