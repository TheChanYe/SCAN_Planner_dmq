#include <gtest/gtest.h>

#include "navdog_scan_adapter/scan_local_planner_adapter.hpp"

#include <navdog_core/config.hpp>
#include <navdog_core/types.hpp>

#include <memory>
#include <limits>
#include <type_traits>
#include <vector>

namespace navdog_scan_adapter
{
class ScanLocalPlannerAdapterTestPeer
{
public:
  static void setReplanResults(
      ScanLocalPlannerAdapter& adapter,
      const std::vector<bool>& results,
      std::vector<bool>& random_flags)
  {
    auto index = std::make_shared<std::size_t>(0);
    adapter.replan_attempt_for_test_ =
        [results, index, &random_flags](bool random) {
          random_flags.push_back(random);
          const bool result = *index < results.size()
              ? results[*index]
              : false;
          ++(*index);
          return result;
        };
  }

  static bool replan(
      ScanLocalPlannerAdapter& adapter,
      const navdog::LocalPlanRequest& request,
      bool& deterministic_success,
      bool& random_success)
  {
    return adapter.doReboundReplan(
        request, deterministic_success, random_success);
  }

  static void storeResult(
      ScanLocalPlannerAdapter& adapter,
      const navdog::LocalPlanRequest& request,
      bool success)
  {
    navdog::LocalTrajectory trajectory{};
    if (success)
    {
      trajectory.task_sequence = request.task_sequence;
      trajectory.plan_sequence = request.plan_sequence;
      trajectory.purpose = request.purpose;
      trajectory.duration_sec = 1.0;
      trajectory.valid = true;
    }
    adapter.storePlanResult(request, trajectory);
  }

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

  static bool validateTrajectory(
      const navdog::LocalTrajectory& trajectory)
  {
    return ScanLocalPlannerAdapter::isSampledTrajectoryValid(trajectory);
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
  request.max_vx = 0.4;
  request.valid = true;
  return request;
}

TEST(ScanLocalPlannerAdapterTest, DeterministicSuccessUsesOneAttempt)
{
  ScanLocalPlannerAdapter adapter(
      navdog::PlannerTriggerConfig{},
      std::make_shared<FakeInflatedGridQuery3D>(), nullptr);
  std::vector<bool> random_flags;
  ScanLocalPlannerAdapterTestPeer::setReplanResults(
      adapter, {true}, random_flags);
  bool deterministic = false;
  bool random = false;

  const auto request = makeRequest(1);
  const bool success = ScanLocalPlannerAdapterTestPeer::replan(
      adapter, request, deterministic, random);
  EXPECT_TRUE(success);
  ScanLocalPlannerAdapterTestPeer::storeResult(
      adapter, request, success);
  EXPECT_TRUE(deterministic);
  EXPECT_FALSE(random);
  ASSERT_EQ(random_flags.size(), 1u);
  EXPECT_FALSE(random_flags[0]);
  EXPECT_EQ(adapter.localPlanState(request.purpose, 7, 1),
            navdog::LocalPlanState::READY);
}

TEST(ScanLocalPlannerAdapterTest, RandomFallbackRunsAfterDeterministicFailure)
{
  ScanLocalPlannerAdapter adapter(
      navdog::PlannerTriggerConfig{},
      std::make_shared<FakeInflatedGridQuery3D>(), nullptr);
  std::vector<bool> random_flags;
  ScanLocalPlannerAdapterTestPeer::setReplanResults(
      adapter, {false, true}, random_flags);
  bool deterministic = false;
  bool random = false;

  const auto request = makeRequest(1);
  const bool success = ScanLocalPlannerAdapterTestPeer::replan(
      adapter, request, deterministic, random);
  EXPECT_TRUE(success);
  ScanLocalPlannerAdapterTestPeer::storeResult(
      adapter, request, success);
  EXPECT_FALSE(deterministic);
  EXPECT_TRUE(random);
  ASSERT_EQ(random_flags.size(), 2u);
  EXPECT_FALSE(random_flags[0]);
  EXPECT_TRUE(random_flags[1]);
  EXPECT_EQ(adapter.localPlanState(request.purpose, 7, 1),
            navdog::LocalPlanState::READY);
}

TEST(ScanLocalPlannerAdapterTest, BothFailuresStopAfterTwoAttempts)
{
  ScanLocalPlannerAdapter adapter(
      navdog::PlannerTriggerConfig{},
      std::make_shared<FakeInflatedGridQuery3D>(), nullptr);
  std::vector<bool> random_flags;
  ScanLocalPlannerAdapterTestPeer::setReplanResults(
      adapter, {false, false, true}, random_flags);
  bool deterministic = false;
  bool random = false;

  const auto request = makeRequest(1);
  const bool success = ScanLocalPlannerAdapterTestPeer::replan(
      adapter, request, deterministic, random);
  EXPECT_FALSE(success);
  ScanLocalPlannerAdapterTestPeer::storeResult(
      adapter, request, success);
  EXPECT_FALSE(deterministic);
  EXPECT_FALSE(random);
  ASSERT_EQ(random_flags.size(), 2u);
  EXPECT_FALSE(random_flags[0]);
  EXPECT_TRUE(random_flags[1]);
  EXPECT_EQ(adapter.localPlanState(request.purpose, 7, 1),
            navdog::LocalPlanState::FAILED);
}

navdog::LocalTrajectory makeSampledTrajectory()
{
  navdog::LocalTrajectory trajectory{};
  trajectory.duration_sec = 0.1;
  for (int i = 0; i < 2; ++i)
  {
    navdog::TimedTrajectoryPoint point{};
    point.time_from_start_sec = 0.1 * i;
    point.x = 0.1 * i;
    point.vx = 1.0;
    trajectory.points.push_back(point);
  }
  return trajectory;
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

TEST(ScanLocalPlannerAdapterTest, RejectsNaNPlanStart)
{
  auto manager = std::make_shared<scan_planner::SCANPlannerManager>();
  ScanLocalPlannerAdapter adapter(navdog::PlannerTriggerConfig{},
      std::make_shared<FakeInflatedGridQuery3D>(), manager);
  auto request = makeRequest(1);
  request.start.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter.requestLocalPlan(request));
}

TEST(ScanLocalPlannerAdapterTest, RejectsNaNPlanVelocity)
{
  auto manager = std::make_shared<scan_planner::SCANPlannerManager>();
  ScanLocalPlannerAdapter adapter(navdog::PlannerTriggerConfig{},
      std::make_shared<FakeInflatedGridQuery3D>(), manager);
  auto request = makeRequest(1);
  request.start_vel.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter.requestLocalPlan(request));
}

TEST(ScanLocalPlannerAdapterTest, RejectsNaNPlanTarget)
{
  auto manager = std::make_shared<scan_planner::SCANPlannerManager>();
  ScanLocalPlannerAdapter adapter(navdog::PlannerTriggerConfig{},
      std::make_shared<FakeInflatedGridQuery3D>(), manager);
  auto request = makeRequest(1);
  request.target.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter.requestLocalPlan(request));
}

TEST(ScanLocalPlannerAdapterTest, RejectsZeroTaskSequence)
{
  auto manager = std::make_shared<scan_planner::SCANPlannerManager>();
  ScanLocalPlannerAdapter adapter(navdog::PlannerTriggerConfig{},
      std::make_shared<FakeInflatedGridQuery3D>(), manager);
  auto request = makeRequest(1);
  request.task_sequence = 0;
  EXPECT_FALSE(adapter.requestLocalPlan(request));
}

TEST(ScanLocalPlannerAdapterTest, RejectsZeroPlanSequence)
{
  auto manager = std::make_shared<scan_planner::SCANPlannerManager>();
  ScanLocalPlannerAdapter adapter(navdog::PlannerTriggerConfig{},
      std::make_shared<FakeInflatedGridQuery3D>(), manager);
  EXPECT_FALSE(adapter.requestLocalPlan(makeRequest(0)));
}

TEST(ScanLocalPlannerAdapterTest, RejectsInvalidMaxVx)
{
  auto manager = std::make_shared<scan_planner::SCANPlannerManager>();
  ScanLocalPlannerAdapter adapter(navdog::PlannerTriggerConfig{},
      std::make_shared<FakeInflatedGridQuery3D>(), manager);
  auto request = makeRequest(1);
  request.max_vx = 0.0;
  EXPECT_FALSE(adapter.requestLocalPlan(request));
  request.max_vx = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter.requestLocalPlan(request));
}

TEST(ScanLocalPlannerAdapterTest, RejectsNaNSampledPoint)
{
  auto trajectory = makeSampledTrajectory();
  trajectory.points[0].x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ScanLocalPlannerAdapterTestPeer::validateTrajectory(trajectory));
}

TEST(ScanLocalPlannerAdapterTest, RejectsNonMonotonicTrajectoryTime)
{
  auto trajectory = makeSampledTrajectory();
  trajectory.points.insert(trajectory.points.begin() + 1,
                           trajectory.points.front());
  trajectory.points[1].time_from_start_sec = 0.08;
  trajectory.points[2].time_from_start_sec = 0.05;
  EXPECT_FALSE(ScanLocalPlannerAdapterTestPeer::validateTrajectory(trajectory));
}

TEST(ScanLocalPlannerAdapterTest, RejectsSinglePointTrajectory)
{
  auto trajectory = makeSampledTrajectory();
  trajectory.points.resize(1);
  EXPECT_FALSE(ScanLocalPlannerAdapterTestPeer::validateTrajectory(trajectory));
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
