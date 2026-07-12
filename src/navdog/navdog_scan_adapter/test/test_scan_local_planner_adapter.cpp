#include <gtest/gtest.h>

#include "navdog_scan_adapter/scan_local_planner_adapter.hpp"

#include <navdog_core/config.hpp>
#include <navdog_core/types.hpp>

#include <memory>
#include <type_traits>

namespace navdog_scan_adapter
{
class ScanLocalPlannerAdapterTestPeer
{
public:
  static void setPending(
      ScanLocalPlannerAdapter& adapter,
      const navdog::LocalPlanRequest& request)
  {
    std::lock_guard<std::mutex> lock(adapter.mutex_);
    adapter.pending_request_ = request;
    adapter.has_pending_request_ = true;
  }

  static void setActive(
      ScanLocalPlannerAdapter& adapter,
      const navdog::LocalPlanRequest& request)
  {
    std::lock_guard<std::mutex> lock(adapter.mutex_);
    adapter.active_request_ = request;
    adapter.has_active_request_ = true;
  }

  static void setCompleted(
      ScanLocalPlannerAdapter& adapter,
      const navdog::LocalPlanRequest& request,
      navdog::LocalPlanState state)
  {
    std::lock_guard<std::mutex> lock(adapter.mutex_);
    adapter.completed_request_ = request;
    adapter.completed_state_ = state;
  }
};

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

navdog::LocalPlanRequest makeRequest(std::uint64_t plan_sequence)
{
  navdog::LocalPlanRequest request{};
  request.purpose = navdog::NavigationMode::LOCAL_AVOID;
  request.task_sequence = 7;
  request.plan_sequence = plan_sequence;
  request.valid = true;
  return request;
}

TEST(ScanLocalPlannerAdapterTest, PendingRequestReportsQueued)
{
  auto manager = std::make_shared<scan_planner::SCANPlannerManager>();
  ScanLocalPlannerAdapter adapter(
      navdog::PlannerTriggerConfig{},
      std::make_shared<FakeInflatedGridQuery3D>(), manager);
  const auto request = makeRequest(1);
  ScanLocalPlannerAdapterTestPeer::setPending(adapter, request);
  EXPECT_EQ(adapter.localPlanState(request.purpose, 7, 1),
            navdog::LocalPlanState::QUEUED);
}

TEST(ScanLocalPlannerAdapterTest, ActiveRequestReportsPlanning)
{
  auto manager = std::make_shared<scan_planner::SCANPlannerManager>();
  ScanLocalPlannerAdapter adapter(
      navdog::PlannerTriggerConfig{},
      std::make_shared<FakeInflatedGridQuery3D>(), manager);
  const auto request = makeRequest(2);
  ScanLocalPlannerAdapterTestPeer::setActive(adapter, request);
  EXPECT_EQ(adapter.localPlanState(request.purpose, 7, 2),
            navdog::LocalPlanState::PLANNING);
}

TEST(ScanLocalPlannerAdapterTest, FailedRequestReportsFailed)
{
  auto manager = std::make_shared<scan_planner::SCANPlannerManager>();
  ScanLocalPlannerAdapter adapter(
      navdog::PlannerTriggerConfig{},
      std::make_shared<FakeInflatedGridQuery3D>(), manager);
  const auto request = makeRequest(3);
  ScanLocalPlannerAdapterTestPeer::setCompleted(
      adapter, request, navdog::LocalPlanState::FAILED);
  EXPECT_EQ(adapter.localPlanState(request.purpose, 7, 3),
            navdog::LocalPlanState::FAILED);
}

TEST(ScanLocalPlannerAdapterTest, NewestPendingRequestWins)
{
  auto manager = std::make_shared<scan_planner::SCANPlannerManager>();
  ScanLocalPlannerAdapter adapter(
      navdog::PlannerTriggerConfig{},
      std::make_shared<FakeInflatedGridQuery3D>(), manager);
  ScanLocalPlannerAdapterTestPeer::setPending(adapter, makeRequest(1));
  ScanLocalPlannerAdapterTestPeer::setPending(adapter, makeRequest(2));
  EXPECT_EQ(adapter.localPlanState(
                navdog::NavigationMode::LOCAL_AVOID, 7, 1),
            navdog::LocalPlanState::IDLE);
  EXPECT_EQ(adapter.localPlanState(
                navdog::NavigationMode::LOCAL_AVOID, 7, 2),
            navdog::LocalPlanState::QUEUED);
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
