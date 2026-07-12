#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "navdog_core/navigation_coordinator.hpp"

namespace navdog
{
namespace
{

// =============================================================================
// Helper
// =============================================================================

NavigationEvent makeValidStartEvent()
{
  NavigationEvent event{};
  event.type = NavigationEventType::START_TASK;
  event.task.mode = TaskMode::NORMAL_AVOID;
  event.task.max_vx = 0.4;

  RoutePoint point{};
  point.x = 1.0;
  point.y = 0.0;
  point.z = 0.0;

  event.task.points.push_back(point);

  return event;
}

CoreInput makePlannerInput(
    PlannerState state,
    std::uint64_t trajectory_id,
    double stamp_sec)
{
  CoreInput input{};
  input.planner.state = state;
  input.planner.trajectory_id = trajectory_id;
  input.planner.stamp_sec = stamp_sec;
  input.planner.valid = true;
  return input;
}

RobotState makeRobotState(
    double x, double y, double yaw)
{
  RobotState robot{};
  robot.x = x;
  robot.y = y;
  robot.yaw = yaw;
  robot.valid = true;
  return robot;
}

CoreInput makeRobotInput(
    double x, double y, double yaw)
{
  CoreInput input{};
  input.robot = makeRobotState(x, y, yaw);
  return input;
}

NavigationEvent makeMultiPointStartEvent()
{
  NavigationEvent event{};
  event.type = NavigationEventType::START_TASK;
  event.task.mode = TaskMode::NORMAL_AVOID;
  event.task.max_vx = 0.4;

  RoutePoint p0{};
  p0.x = 0.0;
  p0.y = 0.0;
  event.task.points.push_back(p0);

  RoutePoint p1{};
  p1.x = 10.0;
  p1.y = 0.0;
  event.task.points.push_back(p1);

  return event;
}

// Create a valid CLEAR SCAN 3D observation for the given
// task sequence, evaluated from the given arc length,
// with map stamp matching now_sec.
RouteCorridorAssessment makeClearScanObservation(
    std::uint64_t sequence,
    double evaluated_from_arc,
    double now_sec)
{
  RouteCorridorAssessment obs;
  obs.source =
      RouteCorridorSource::SCAN_INFLATED_GRID_3D;
  obs.task_sequence = sequence;
  obs.blocked = false;
  obs.evaluated_from_arc_length_m =
      evaluated_from_arc;
  obs.checked_distance_m = 3.0;
  obs.first_blocked_distance_ahead_m =
      std::numeric_limits<double>::infinity();
  obs.first_blocked_arc_length_m =
      std::numeric_limits<double>::infinity();
  obs.map_resolution_m = 0.10;
  obs.sample_step_m = 0.05;
  obs.query_z_m = 0.0;
  obs.samples_checked = 60;
  obs.out_of_map = false;
  obs.map_stamp_sec = now_sec;
  obs.evaluation_stamp_sec = now_sec;
  obs.valid = true;
  return obs;
}

RouteCorridorAssessment makeBlockedScanObservation(
    std::uint64_t sequence,
    double evaluated_from_arc,
    double now_sec)
{
  RouteCorridorAssessment obs =
      makeClearScanObservation(
          sequence, evaluated_from_arc, now_sec);
  obs.blocked = true;
  obs.first_blocked_distance_ahead_m = 1.0;
  obs.first_blocked_arc_length_m =
      evaluated_from_arc + 1.0;
  return obs;
}

RouteCorridorAssessment makeBlockedScanObservationAt(
    std::uint64_t sequence,
    double evaluated_from_arc,
    double now_sec,
    double blocked_distance)
{
  RouteCorridorAssessment obs =
      makeClearScanObservation(
          sequence, evaluated_from_arc, now_sec);
  obs.blocked = true;
  obs.first_blocked_distance_ahead_m = blocked_distance;
  obs.first_blocked_arc_length_m =
      evaluated_from_arc + blocked_distance;
  return obs;
}

RouteCorridorAssessment makeStaleScanObservation(
    std::uint64_t sequence,
    double evaluated_from_arc,
    double now_sec)
{
  RouteCorridorAssessment obs =
      makeClearScanObservation(
          sequence, evaluated_from_arc, now_sec);
  obs.map_stamp_sec = now_sec - 0.50;
  obs.evaluation_stamp_sec = now_sec - 0.50;
  return obs;
}

void setupToTracking(
    NavigationCoordinator& coord,
    TaskMode mode = TaskMode::NORMAL_AVOID)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;
  NavigationEvent event = makeMultiPointStartEvent();
  event.task.mode = mode;
  coord.handleEvent(event);
  coord.update(CoreInput{}, 1.0);
  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coord.update(ready, 1.1);
  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coord.update(robot0, 1.2);
}

CoreInput makeTrackingInput(
    double x, double y, double yaw,
    std::uint64_t seq,
    double arc,
    double now_sec)
{
  CoreInput input = makeRobotInput(x, y, yaw);
  input.route_corridor_observation =
      makeClearScanObservation(seq, arc, now_sec);
  return input;
}

LocalTrajectory makeTestLocalTrajectory(
    std::uint64_t task_sequence,
    std::uint64_t plan_sequence,
    NavigationMode purpose,
    double start_sec,
    double duration_sec)
{
  LocalTrajectory trajectory{};
  trajectory.task_sequence = task_sequence;
  trajectory.plan_sequence = plan_sequence;
  trajectory.purpose = purpose;
  trajectory.source_stamp_sec = start_sec;
  trajectory.duration_sec = duration_sec;
  trajectory.valid = true;

  const std::size_t n = 5;
  for (std::size_t i = 0; i <= n; ++i)
  {
    const double ratio = static_cast<double>(i) / n;
    TimedTrajectoryPoint p{};
    p.time_from_start_sec = ratio * duration_sec;
    p.x = ratio * 1.0;
    p.y = 0.0;
    p.z = 0.0;
    p.vx = 0.5;
    p.vy = 0.0;
    p.yaw = 0.0;
    p.has_yaw = true;
    trajectory.points.push_back(p);
  }

  return trajectory;
}

class FakeLocalPlannerAdapter : public LocalPlannerAdapter
{
public:
  bool requestLocalPlan(
      const LocalPlanRequest& request) override
  {
    last_request_ = request;
    request_count_++;
    return accept_requests_;
  }

  LocalTrajectory getLocalTrajectory(
      NavigationMode purpose,
      std::uint64_t task_sequence) const override
  {
    if (!has_trajectory_ ||
        trajectory_.purpose != purpose ||
        trajectory_.task_sequence != task_sequence)
    {
      return LocalTrajectory{};
    }
    return trajectory_;
  }

  bool hasValidTrajectory(
      NavigationMode purpose,
      std::uint64_t task_sequence) const override
  {
    return has_trajectory_ &&
           trajectory_.purpose == purpose &&
           trajectory_.task_sequence == task_sequence &&
           trajectory_.valid;
  }

  bool isTrajectoryColliding(
      NavigationMode purpose,
      std::uint64_t task_sequence) const override
  {
    (void)purpose;
    (void)task_sequence;
    return collision_flag_;
  }

  void setCollisionFlag(bool collision) noexcept
  {
    collision_flag_ = collision;
  }

  void setTrajectory(const LocalTrajectory& trajectory)
  {
    trajectory_ = trajectory;
    has_trajectory_ = trajectory.valid;
  }

  void clearTrajectory()
  {
    has_trajectory_ = false;
    trajectory_ = LocalTrajectory{};
  }

  void setAcceptRequests(bool accept)
  {
    accept_requests_ = accept;
  }

  std::uint64_t requestCount() const noexcept
  {
    return request_count_;
  }

  const LocalPlanRequest& lastRequest() const noexcept
  {
    return last_request_;
  }

private:
  LocalTrajectory trajectory_{};
  bool has_trajectory_{false};
  bool accept_requests_{true};
  bool collision_flag_{false};
  std::uint64_t request_count_{0};
  LocalPlanRequest last_request_{};
};

// =============================================================================
// DefaultStateIsIdle
// =============================================================================

TEST(NavigationCoordinatorTest, DefaultStateIsIdle)
{
  NavigationCoordinator coordinator;
  EXPECT_EQ(coordinator.state(), NavState::IDLE);
}

// =============================================================================
// DefaultOutputIsSafeZero
// =============================================================================

TEST(NavigationCoordinatorTest, DefaultOutputIsSafeZero)
{
  NavigationCoordinator coordinator;

  CoreOutput output = coordinator.update(CoreInput{}, 12.5);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_EQ(output.task_sequence, 0u);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);

  EXPECT_TRUE(output.final_cmd.valid);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.stamp_sec, 12.5);
  EXPECT_EQ(output.final_cmd.source, CommandSource::IDLE_STOP);
}

// =============================================================================
// DoesNotPassThroughPlannerCmd
// =============================================================================

TEST(NavigationCoordinatorTest, DoesNotPassThroughPlannerCmd)
{
  NavigationCoordinator coordinator;

  CoreInput input;
  input.planner_cmd.vx = 0.5;
  input.planner_cmd.vy = 0.2;
  input.planner_cmd.yaw_rate = 0.3;
  input.planner_cmd.valid = true;
  input.planner_cmd.source = CommandSource::PLANNER;

  CoreOutput output = coordinator.update(input, 20.0);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_TRUE(output.final_cmd.valid);
  EXPECT_EQ(output.final_cmd.source, CommandSource::IDLE_STOP);
}

// =============================================================================
// ResetKeepsIdle
// =============================================================================

TEST(NavigationCoordinatorTest, ResetKeepsIdle)
{
  NavigationCoordinator coordinator;

  coordinator.reset();

  EXPECT_EQ(coordinator.state(), NavState::IDLE);
}

// =============================================================================
// 13.2 StartTaskTransitionsToPlanning
// =============================================================================

TEST(NavigationCoordinatorTest, StartTaskTransitionsToPlanning)
{
  NavigationCoordinator coordinator;

  NavigationEvent event = makeValidStartEvent();
  TaskHandleResult result = coordinator.handleEvent(event);

  EXPECT_EQ(result, TaskHandleResult::STARTED);
  EXPECT_EQ(coordinator.state(), NavState::PLANNING);
  EXPECT_TRUE(coordinator.hasActiveTask());
}

// =============================================================================
// 13.3 StartTaskEmitsSetRouteOnce
// =============================================================================

TEST(NavigationCoordinatorTest, StartTaskEmitsSetRouteOnce)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());

  CoreOutput out1 = coordinator.update(CoreInput{}, 1.0);
  EXPECT_EQ(out1.state, NavState::PLANNING);
  EXPECT_EQ(out1.task_sequence, 1u);
  EXPECT_EQ(out1.planner_action.type, PlannerActionType::SET_ROUTE);
  EXPECT_EQ(out1.planner_action.task.sequence, 1u);
  EXPECT_DOUBLE_EQ(out1.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(out1.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(out1.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(out1.final_cmd.source, CommandSource::PLANNING_STOP);

  CoreOutput out2 = coordinator.update(CoreInput{}, 2.0);
  EXPECT_EQ(out2.planner_action.type, PlannerActionType::NONE);
  EXPECT_EQ(out2.task_sequence, 1u);
  EXPECT_EQ(out2.final_cmd.source, CommandSource::PLANNING_STOP);
}

// =============================================================================
// 13.4 InvalidTaskDoesNotLeaveIdle
// =============================================================================

TEST(NavigationCoordinatorTest, InvalidTaskDoesNotLeaveIdle)
{
  NavigationCoordinator coordinator;

  NavigationEvent event{};
  event.type = NavigationEventType::START_TASK;
  event.task.mode = TaskMode::NORMAL_AVOID;
  event.task.max_vx = 0.4;

  TaskHandleResult result = coordinator.handleEvent(event);

  EXPECT_EQ(result, TaskHandleResult::REJECTED_INVALID_TASK);
  EXPECT_EQ(coordinator.state(), NavState::IDLE);
  EXPECT_FALSE(coordinator.hasActiveTask());

  CoreOutput output = coordinator.update(CoreInput{}, 1.0);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  EXPECT_EQ(output.final_cmd.source, CommandSource::IDLE_STOP);
}

// =============================================================================
// 13.5 BusyTaskDoesNotReplaceActiveTask
// =============================================================================

TEST(NavigationCoordinatorTest, BusyTaskDoesNotReplaceActiveTask)
{
  NavigationCoordinator coordinator;

  NavigationEvent eventA = makeValidStartEvent();
  eventA.task.points[0].x = 10.0;
  coordinator.handleEvent(eventA);

  coordinator.update(CoreInput{}, 1.0);

  NavigationEvent eventB = makeValidStartEvent();
  eventB.task.points[0].x = 20.0;
  TaskHandleResult result = coordinator.handleEvent(eventB);

  EXPECT_EQ(result, TaskHandleResult::REJECTED_BUSY);
  EXPECT_EQ(coordinator.state(), NavState::PLANNING);

  NavigationTask stored{};
  coordinator.copyActiveTask(stored);
  EXPECT_EQ(stored.sequence, 1u);
  EXPECT_DOUBLE_EQ(stored.points[0].x, 10.0);

  CoreOutput output = coordinator.update(CoreInput{}, 2.0);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
}

// =============================================================================
// 13.6 CancelTransitionsToIdleAndEmitsOnce
// =============================================================================

TEST(NavigationCoordinatorTest, CancelTransitionsToIdleAndEmitsOnce)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  TaskHandleResult result = coordinator.handleEvent(cancel);

  EXPECT_EQ(result, TaskHandleResult::CANCELLED);

  CoreOutput out1 = coordinator.update(CoreInput{}, 2.0);
  EXPECT_EQ(out1.state, NavState::IDLE);
  EXPECT_FALSE(coordinator.hasActiveTask());
  EXPECT_EQ(out1.planner_action.type, PlannerActionType::CANCEL);
  EXPECT_EQ(out1.planner_action.task.sequence, 1u);
  EXPECT_EQ(out1.task_sequence, 0u);
  EXPECT_DOUBLE_EQ(out1.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(out1.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(out1.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(out1.final_cmd.source, CommandSource::CANCEL_STOP);

  CoreOutput out2 = coordinator.update(CoreInput{}, 3.0);
  EXPECT_EQ(out2.planner_action.type, PlannerActionType::NONE);
  EXPECT_EQ(out2.final_cmd.source, CommandSource::IDLE_STOP);
}

// =============================================================================
// 13.7 CancelDropsPendingSetRoute
// =============================================================================

TEST(NavigationCoordinatorTest, CancelDropsPendingSetRoute)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);

  CoreOutput out1 = coordinator.update(CoreInput{}, 1.0);
  EXPECT_EQ(out1.state, NavState::IDLE);
  EXPECT_FALSE(coordinator.hasActiveTask());
  EXPECT_EQ(out1.planner_action.type, PlannerActionType::CANCEL);

  CoreOutput out2 = coordinator.update(CoreInput{}, 2.0);
  EXPECT_EQ(out2.planner_action.type, PlannerActionType::NONE);
}

// =============================================================================
// 13.8 MaxVxUpdateEmitsActionOnce
// =============================================================================

TEST(NavigationCoordinatorTest, MaxVxUpdateEmitsActionOnce)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  NavigationEvent speedUp{};
  speedUp.type = NavigationEventType::UPDATE_MAX_VX;
  speedUp.max_vx = 0.6;

  TaskHandleResult result = coordinator.handleEvent(speedUp);
  EXPECT_EQ(result, TaskHandleResult::MAX_VX_UPDATED);

  CoreOutput out1 = coordinator.update(CoreInput{}, 2.0);
  EXPECT_EQ(out1.state, NavState::PLANNING);
  EXPECT_EQ(out1.planner_action.type, PlannerActionType::UPDATE_SPEED_LIMIT);
  EXPECT_EQ(out1.planner_action.task.sequence, 1u);
  EXPECT_DOUBLE_EQ(out1.planner_action.max_vx, 0.6);
  EXPECT_EQ(out1.final_cmd.source, CommandSource::PLANNING_STOP);

  CoreOutput out2 = coordinator.update(CoreInput{}, 3.0);
  EXPECT_EQ(out2.planner_action.type, PlannerActionType::NONE);
}

// =============================================================================
// 13.9 UnchangedMaxVxEmitsNoAction
// =============================================================================

TEST(NavigationCoordinatorTest, UnchangedMaxVxEmitsNoAction)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  NavigationEvent sameSpeed{};
  sameSpeed.type = NavigationEventType::UPDATE_MAX_VX;
  sameSpeed.max_vx = 0.4;

  TaskHandleResult result = coordinator.handleEvent(sameSpeed);
  EXPECT_EQ(result, TaskHandleResult::MAX_VX_UNCHANGED);

  CoreOutput output = coordinator.update(CoreInput{}, 2.0);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  EXPECT_EQ(coordinator.state(), NavState::PLANNING);
}

// =============================================================================
// 13.10 UnsupportedEventDoesNotChangeCoordinator
// =============================================================================

TEST(NavigationCoordinatorTest, UnsupportedEventDoesNotChangeCoordinator)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  NavigationEvent pause{};
  pause.type = NavigationEventType::PAUSE;
  TaskHandleResult result = coordinator.handleEvent(pause);

  EXPECT_EQ(result, TaskHandleResult::UNSUPPORTED_EVENT);
  EXPECT_EQ(coordinator.state(), NavState::PLANNING);
  EXPECT_TRUE(coordinator.hasActiveTask());

  CoreOutput output = coordinator.update(CoreInput{}, 2.0);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
}

// =============================================================================
// 13.11 ResetClearsTaskAndPendingAction
// =============================================================================

TEST(NavigationCoordinatorTest, ResetClearsTaskAndPendingAction)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());

  coordinator.reset();

  CoreOutput output = coordinator.update(CoreInput{}, 1.0);
  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_FALSE(coordinator.hasActiveTask());
  EXPECT_EQ(output.task_sequence, 0u);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  EXPECT_EQ(output.final_cmd.source, CommandSource::IDLE_STOP);
}

// =============================================================================
// 13.12 PlanningStateStillRejectsPlannerCmd
// =============================================================================

TEST(NavigationCoordinatorTest, PlanningStateStillRejectsPlannerCmd)
{
  NavigationCoordinator coordinator;

  coordinator.handleEvent(makeValidStartEvent());

  CoreInput input;
  input.planner_cmd.vx = 0.5;
  input.planner_cmd.vy = 0.2;
  input.planner_cmd.yaw_rate = 0.3;
  input.planner_cmd.valid = true;

  CoreOutput output = coordinator.update(input, 1.0);

  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::PLANNING_STOP);
}

// =============================================================================
// 17.1 SetRouteStartsPlanningAndIgnoresSameCycleFeedback
// =============================================================================

TEST(NavigationCoordinatorTest, SetRouteStartsPlanningAndIgnoresSameCycleFeedback)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());

  CoreInput input = makePlannerInput(PlannerState::READY, 1u, 1.0);
  CoreOutput out1 = coordinator.update(input, 1.0);

  EXPECT_EQ(out1.planner_action.type, PlannerActionType::SET_ROUTE);
  EXPECT_EQ(out1.state, NavState::PLANNING);
  EXPECT_EQ(out1.final_cmd.source, CommandSource::PLANNING_STOP);
}

// =============================================================================
// 17.2 ReadyFeedbackTransitionsToStartAlign
// =============================================================================

TEST(NavigationCoordinatorTest, ReadyFeedbackTransitionsToStartAlign)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // emit SET_ROUTE

  CoreInput input = makePlannerInput(PlannerState::READY, 1u, 1.1);
  CoreOutput output = coordinator.update(input, 1.1);

  EXPECT_EQ(output.state, NavState::START_ALIGN);
  EXPECT_EQ(output.task_sequence, 1u);
  EXPECT_TRUE(coordinator.hasActiveTask());
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::START_ALIGN);
}

// =============================================================================
// 17.3 ExecutingFeedbackTransitionsToStartAlign
// =============================================================================

TEST(NavigationCoordinatorTest, ExecutingFeedbackTransitionsToStartAlign)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput input = makePlannerInput(PlannerState::EXECUTING, 1u, 1.1);
  CoreOutput output = coordinator.update(input, 1.1);

  EXPECT_EQ(output.state, NavState::START_ALIGN);
  EXPECT_EQ(output.final_cmd.source, CommandSource::START_ALIGN);
}

// =============================================================================
// 17.4 FailedFeedbackTransitionsToFailed
// =============================================================================

TEST(NavigationCoordinatorTest, FailedFeedbackTransitionsToFailed)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput input = makePlannerInput(PlannerState::FAILED, 1u, 1.1);
  CoreOutput output = coordinator.update(input, 1.1);

  EXPECT_EQ(output.state, NavState::FAILED);
  EXPECT_EQ(output.task_sequence, 1u);
  EXPECT_TRUE(coordinator.hasActiveTask());
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::FAILED_STOP);
}

// =============================================================================
// 17.5 WaitingFeedbackKeepsPlanning
// =============================================================================

TEST(NavigationCoordinatorTest, WaitingFeedbackKeepsPlanning)
{
  {
    NavigationCoordinator coordinator;
    coordinator.handleEvent(makeValidStartEvent());
    coordinator.update(CoreInput{}, 1.0);

    CoreInput input = makePlannerInput(PlannerState::UNAVAILABLE, 1u, 1.1);
    CoreOutput output = coordinator.update(input, 1.1);

    EXPECT_EQ(output.state, NavState::PLANNING);
    EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  }
  {
    NavigationCoordinator coordinator;
    coordinator.handleEvent(makeValidStartEvent());
    coordinator.update(CoreInput{}, 1.0);

    CoreInput input = makePlannerInput(PlannerState::IDLE, 1u, 1.1);
    CoreOutput output = coordinator.update(input, 1.1);

    EXPECT_EQ(output.state, NavState::PLANNING);
    EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  }
  {
    NavigationCoordinator coordinator;
    coordinator.handleEvent(makeValidStartEvent());
    coordinator.update(CoreInput{}, 1.0);

    CoreInput input = makePlannerInput(PlannerState::PLANNING, 1u, 1.1);
    CoreOutput output = coordinator.update(input, 1.1);

    EXPECT_EQ(output.state, NavState::PLANNING);
    EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  }
}

// =============================================================================
// 17.6 InvalidFeedbackIsIgnored
// =============================================================================

TEST(NavigationCoordinatorTest, InvalidFeedbackIsIgnored)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput input{};
  input.planner.state = PlannerState::READY;
  input.planner.trajectory_id = 1u;
  input.planner.stamp_sec = 1.1;
  input.planner.valid = false;

  CoreOutput output = coordinator.update(input, 1.1);
  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.7 MismatchedTrajectoryIdIsIgnored
// =============================================================================

TEST(NavigationCoordinatorTest, MismatchedTrajectoryIdIsIgnored)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput input = makePlannerInput(PlannerState::READY, 999u, 1.1);
  CoreOutput output = coordinator.update(input, 1.1);

  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.8 ZeroTrajectoryIdIsIgnored
// =============================================================================

TEST(NavigationCoordinatorTest, ZeroTrajectoryIdIsIgnored)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput input = makePlannerInput(PlannerState::READY, 0u, 1.1);
  CoreOutput output = coordinator.update(input, 1.1);

  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.9 StaleFeedbackIsIgnored
// =============================================================================

TEST(NavigationCoordinatorTest, StaleFeedbackIsIgnored)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 10.0);  // SET_ROUTE at 10.0

  CoreInput input = makePlannerInput(PlannerState::READY, 1u, 9.9);
  CoreOutput output = coordinator.update(input, 10.1);

  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.10 FutureFeedbackIsIgnored
// =============================================================================

TEST(NavigationCoordinatorTest, FutureFeedbackIsIgnored)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 10.0);

  CoreInput input = makePlannerInput(PlannerState::READY, 1u, 10.2);
  CoreOutput output = coordinator.update(input, 10.1);

  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.11 NonFiniteFeedbackStampIsIgnored
// =============================================================================

TEST(NavigationCoordinatorTest, NonFiniteFeedbackStampIsIgnored)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput input = makePlannerInput(
      PlannerState::READY, 1u,
      std::numeric_limits<double>::quiet_NaN());
  CoreOutput output = coordinator.update(input, 1.1);

  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.12 DoesNotTimeoutBeforeSetRouteEmission
// =============================================================================

TEST(NavigationCoordinatorTest, DoesNotTimeoutBeforeSetRouteEmission)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());

  CoreOutput output = coordinator.update(CoreInput{}, 100.0);

  EXPECT_EQ(output.planner_action.type, PlannerActionType::SET_ROUTE);
  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.13 DoesNotTimeoutAtExactBoundary
// =============================================================================

TEST(NavigationCoordinatorTest, DoesNotTimeoutAtExactBoundary)
{
  NavdogConfig config;
  config.planner.planning_timeout_sec = 2.0;
  NavigationCoordinator coordinator(config);

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE at 1.0

  CoreOutput output = coordinator.update(CoreInput{}, 3.0);  // exactly 2.0
  EXPECT_EQ(output.state, NavState::PLANNING);
}

// =============================================================================
// 17.14 PlanningTimeoutTransitionsToFailed
// =============================================================================

TEST(NavigationCoordinatorTest, PlanningTimeoutTransitionsToFailed)
{
  NavdogConfig config;
  config.planner.planning_timeout_sec = 2.0;
  NavigationCoordinator coordinator(config);

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE at 1.0

  CoreOutput output = coordinator.update(CoreInput{}, 3.001);

  EXPECT_EQ(output.state, NavState::FAILED);
  EXPECT_EQ(output.final_cmd.source, CommandSource::FAILED_STOP);
  EXPECT_TRUE(coordinator.hasActiveTask());
  EXPECT_EQ(output.task_sequence, 1u);
}

// =============================================================================
// 17.15 CancelFromStartAlignReturnsIdle
// =============================================================================

TEST(NavigationCoordinatorTest, CancelFromStartAlignReturnsIdle)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);  // → START_ALIGN

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);

  CoreOutput output = coordinator.update(CoreInput{}, 1.2);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::CANCEL);
  EXPECT_EQ(output.final_cmd.source, CommandSource::CANCEL_STOP);
  EXPECT_FALSE(coordinator.hasActiveTask());
}

// =============================================================================
// 17.16 CancelFromFailedReturnsIdle
// =============================================================================

TEST(NavigationCoordinatorTest, CancelFromFailedReturnsIdle)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE

  CoreInput failed = makePlannerInput(PlannerState::FAILED, 1u, 1.1);
  coordinator.update(failed, 1.1);  // → FAILED

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);

  CoreOutput output = coordinator.update(CoreInput{}, 1.2);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::CANCEL);
  EXPECT_FALSE(coordinator.hasActiveTask());
}

// =============================================================================
// 17.17 FailedStateIgnoresLaterReadyFeedback
// =============================================================================

TEST(NavigationCoordinatorTest, FailedStateIgnoresLaterReadyFeedback)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput failed = makePlannerInput(PlannerState::FAILED, 1u, 1.1);
  coordinator.update(failed, 1.1);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.2);
  CoreOutput output = coordinator.update(ready, 1.2);

  EXPECT_EQ(output.state, NavState::FAILED);
}

// =============================================================================
// 17.18 StartAlignIgnoresLaterFailedFeedback
// =============================================================================

TEST(NavigationCoordinatorTest, StartAlignIgnoresLaterFailedFeedback)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput failed = makePlannerInput(PlannerState::FAILED, 1u, 1.2);
  CoreOutput output = coordinator.update(failed, 1.2);

  EXPECT_EQ(output.state, NavState::START_ALIGN);
}

// =============================================================================
// 17.19 FailureDropsPendingSpeedUpdate
// =============================================================================

TEST(NavigationCoordinatorTest, FailureDropsPendingSpeedUpdate)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE

  NavigationEvent speedUp{};
  speedUp.type = NavigationEventType::UPDATE_MAX_VX;
  speedUp.max_vx = 0.6;
  coordinator.handleEvent(speedUp);  // enqueue UPDATE_SPEED_LIMIT

  CoreInput failed = makePlannerInput(PlannerState::FAILED, 1u, 1.1);
  CoreOutput out1 = coordinator.update(failed, 1.1);

  EXPECT_EQ(out1.state, NavState::FAILED);
  EXPECT_EQ(out1.planner_action.type, PlannerActionType::NONE);

  CoreOutput out2 = coordinator.update(CoreInput{}, 1.2);
  EXPECT_EQ(out2.planner_action.type, PlannerActionType::NONE);
}

// =============================================================================
// 17.20 ReadyKeepsPendingSpeedUpdate
// =============================================================================

TEST(NavigationCoordinatorTest, ReadyKeepsPendingSpeedUpdate)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE

  NavigationEvent speedUp{};
  speedUp.type = NavigationEventType::UPDATE_MAX_VX;
  speedUp.max_vx = 0.6;
  coordinator.handleEvent(speedUp);  // enqueue UPDATE_SPEED_LIMIT

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  CoreOutput output = coordinator.update(ready, 1.1);

  EXPECT_EQ(output.state, NavState::START_ALIGN);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::UPDATE_SPEED_LIMIT);
}

// =============================================================================
// 17.21 ResetClearsPlanningContext
// =============================================================================

TEST(NavigationCoordinatorTest, ResetClearsPlanningContext)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE

  coordinator.reset();

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  CoreOutput output = coordinator.update(ready, 1.1);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  EXPECT_EQ(output.task_sequence, 0u);
}

// =============================================================================
// 26.1 ReadyWithLargeYawProducesAlignCommand
// =============================================================================

TEST(NavigationCoordinatorTest, ReadyWithLargeYawProducesAlignCommand)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot_in = makeRobotInput(0, 0, 90.0 * 3.14159265358979323846 / 180.0);
  CoreOutput output = coordinator.update(robot_in, 1.2);

  EXPECT_EQ(output.state, NavState::START_ALIGN);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_NE(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::START_ALIGN);
}

// =============================================================================
// 26.2 StartAlignYawRateUsesCorrectDirection
// =============================================================================

TEST(NavigationCoordinatorTest, StartAlignYawRateUsesCorrectDirection)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  // Positive error (target > robot) → negative yaw_rate
  {
    NavigationCoordinator coordinator;
    coordinator.handleEvent(makeValidStartEvent());
    coordinator.update(CoreInput{}, 1.0);

    CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
    coordinator.update(ready, 1.1);

    // route east (0°), robot at +30° → error = -30° → negative rate
    CoreInput robot_in = makeRobotInput(0, 0, 30.0 * kDeg);
    CoreOutput output = coordinator.update(robot_in, 1.2);

    EXPECT_EQ(output.state, NavState::START_ALIGN);
    EXPECT_LT(output.final_cmd.yaw_rate, 0.0);
  }
  // Negative error (target < robot) → positive yaw_rate
  {
    NavigationCoordinator coordinator;
    coordinator.handleEvent(makeValidStartEvent());
    coordinator.update(CoreInput{}, 1.0);

    CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
    coordinator.update(ready, 1.1);

    // route east (0°), robot at -30° → error = +30° → positive rate
    CoreInput robot_in = makeRobotInput(0, 0, -30.0 * kDeg);
    CoreOutput output = coordinator.update(robot_in, 1.2);

    EXPECT_EQ(output.state, NavState::START_ALIGN);
    EXPECT_GT(output.final_cmd.yaw_rate, 0.0);
  }
}

// =============================================================================
// 26.3 StartAlignYawRateIsLimited
// =============================================================================

TEST(NavigationCoordinatorTest, StartAlignYawRateIsLimited)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot_in = makeRobotInput(0, 0, 170.0 * kDeg);
  CoreOutput output = coordinator.update(robot_in, 1.2);

  EXPECT_EQ(output.state, NavState::START_ALIGN);
  EXPECT_LE(std::fabs(output.final_cmd.yaw_rate), 0.3 + 1e-9);
}

// =============================================================================
// 26.4 SmallInitialErrorTransitionsDirectlyToTracking
// =============================================================================

TEST(NavigationCoordinatorTest, SmallInitialErrorTransitionsDirectlyToTracking)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // 10° error < enter=15° → directly ALIGNED → TRACKING
  CoreInput robot_in = makeRobotInput(0, 0, 10.0 * kDeg);
  CoreOutput output = coordinator.update(robot_in, 1.2);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::TRACKING_STOP);
}

// =============================================================================
// 26.5 AlignmentTransitionsToTrackingAtExitThreshold
// =============================================================================

TEST(NavigationCoordinatorTest, AlignmentTransitionsToTrackingAtExitThreshold)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // 30° → ALIGNING
  CoreInput robot1 = makeRobotInput(0, 0, 30.0 * kDeg);
  CoreOutput out1 = coordinator.update(robot1, 1.2);
  EXPECT_EQ(out1.state, NavState::START_ALIGN);

  // 10° → still ALIGNING (above exit=5°)
  CoreInput robot2 = makeRobotInput(0, 0, 10.0 * kDeg);
  CoreOutput out2 = coordinator.update(robot2, 1.3);
  EXPECT_EQ(out2.state, NavState::START_ALIGN);

  // 4° → TRACKING (below exit=5°)
  CoreInput robot3 = makeRobotInput(0, 0, 4.0 * kDeg);
  CoreOutput out3 = coordinator.update(robot3, 1.4);
  EXPECT_EQ(out3.state, NavState::TRACKING);
  EXPECT_EQ(out3.final_cmd.source, CommandSource::TRACKING_STOP);
}

// =============================================================================
// 26.6 StartAlignTimeoutTransitionsToFailed
// =============================================================================

TEST(NavigationCoordinatorTest, StartAlignTimeoutTransitionsToFailed)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavdogConfig config;
  config.start_align.max_hold_sec = 2.0;
  NavigationCoordinator coordinator(config);

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Large error, past timeout
  CoreInput robot = makeRobotInput(0, 0, 30.0 * kDeg);
  CoreOutput output = coordinator.update(robot, 3.2);

  EXPECT_EQ(output.state, NavState::FAILED);
  EXPECT_EQ(output.final_cmd.source, CommandSource::FAILED_STOP);
  EXPECT_TRUE(coordinator.hasActiveTask());
  EXPECT_EQ(output.task_sequence, 1u);
}

// =============================================================================
// 26.7 MissingRobotWaitsThenTimesOut
// =============================================================================

TEST(NavigationCoordinatorTest, MissingRobotWaitsThenTimesOut)
{
  NavdogConfig config;
  config.start_align.max_hold_sec = 2.0;
  NavigationCoordinator coordinator(config);

  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Invalid robot → WAITING_FOR_ROBOT, still START_ALIGN
  CoreInput bad_robot{};
  CoreOutput out1 = coordinator.update(bad_robot, 2.0);
  EXPECT_EQ(out1.state, NavState::START_ALIGN);
  EXPECT_DOUBLE_EQ(out1.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(out1.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(out1.final_cmd.yaw_rate, 0.0);

  // Past timeout → FAILED
  CoreOutput out2 = coordinator.update(bad_robot, 3.2);
  EXPECT_EQ(out2.state, NavState::FAILED);
}

// =============================================================================
// 26.8 InvalidStartDirectionTransitionsToFailed
// =============================================================================

TEST(NavigationCoordinatorTest, InvalidStartDirectionTransitionsToFailed)
{
  NavigationCoordinator coordinator;

  // Task with single point at robot location, no yaw
  NavigationEvent event{};
  event.type = NavigationEventType::START_TASK;
  event.task.mode = TaskMode::NORMAL_AVOID;
  event.task.max_vx = 0.4;
  RoutePoint p{};
  p.x = 0.0;
  p.y = 0.0;
  event.task.points.push_back(p);

  coordinator.handleEvent(event);
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Valid robot at (0,0), task point at (0,0) → INVALID_TASK
  CoreInput robot_in = makeRobotInput(0, 0, 0);
  CoreOutput output = coordinator.update(robot_in, 1.2);

  EXPECT_EQ(output.state, NavState::FAILED);
  EXPECT_TRUE(coordinator.hasActiveTask());
}

// =============================================================================
// 26.9 CancelDuringStartAlignReturnsIdle
// =============================================================================

TEST(NavigationCoordinatorTest, CancelDuringStartAlignReturnsIdle)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Start rotating
  CoreInput robot = makeRobotInput(0, 0, 30.0 * kDeg);
  coordinator.update(robot, 1.2);

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);

  CoreOutput output = coordinator.update(CoreInput{}, 1.3);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_EQ(output.planner_action.type, PlannerActionType::CANCEL);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::CANCEL_STOP);
}

// =============================================================================
// 26.10 ResetDuringStartAlignClearsController
// =============================================================================

TEST(NavigationCoordinatorTest, ResetDuringStartAlignClearsController)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot = makeRobotInput(0, 0, 30.0 * kDeg);
  coordinator.update(robot, 1.2);

  coordinator.reset();

  CoreOutput output = coordinator.update(CoreInput{}, 1.3);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_FALSE(coordinator.hasActiveTask());
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
}

// =============================================================================
// 26.11 TrackingStillRejectsPlannerCmd
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingStillRejectsPlannerCmd)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Small error → directly to TRACKING
  CoreInput robot1 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot1, 1.2);

  // Now in TRACKING, send planner_cmd
  CoreInput input{};
  input.planner_cmd.vx = 0.5;
  input.planner_cmd.vy = 0.2;
  input.planner_cmd.yaw_rate = 0.2;
  input.planner_cmd.valid = true;

  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::TRACKING_STOP);
}

// =============================================================================
// 26.12 NonFiniteSetRouteTimeTransitionsToFailed
// =============================================================================

TEST(NavigationCoordinatorTest, NonFiniteSetRouteTimeTransitionsToFailed)
{
  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());

  CoreOutput output = coordinator.update(
      CoreInput{},
      std::numeric_limits<double>::quiet_NaN());

  EXPECT_EQ(output.planner_action.type, PlannerActionType::NONE);
  EXPECT_EQ(output.state, NavState::FAILED);
  EXPECT_EQ(output.final_cmd.source, CommandSource::FAILED_STOP);
  EXPECT_TRUE(coordinator.hasActiveTask());
}

// =============================================================================
// 26.13 FailedAlignmentRequiresCancelBeforeNewTask
// =============================================================================

TEST(NavigationCoordinatorTest, FailedAlignmentRequiresCancelBeforeNewTask)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Invalid direction → FAILED (robot at same point as task)
  // Use task with point at origin, robot at origin
  // Actually our makeValidStartEvent has point at (1,0), so let's timeout instead
  NavdogConfig cfg2;
  cfg2.start_align.max_hold_sec = 0.5;
  NavigationCoordinator coord2(cfg2);

  coord2.handleEvent(makeValidStartEvent());
  coord2.update(CoreInput{}, 1.0);

  CoreInput ready2 = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coord2.update(ready2, 1.1);

  // Large error + timeout
  CoreInput robot = makeRobotInput(0, 0, 30.0 * kDeg);
  coord2.update(robot, 1.7);

  // Now try new task → should be rejected
  NavigationEvent newTask = makeValidStartEvent();
  TaskHandleResult result1 = coord2.handleEvent(newTask);
  EXPECT_EQ(result1, TaskHandleResult::REJECTED_BUSY);

  // Cancel first
  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coord2.handleEvent(cancel);
  coord2.update(CoreInput{}, 1.8);

  // Now new task should work
  TaskHandleResult result2 = coord2.handleEvent(newTask);
  EXPECT_EQ(result2, TaskHandleResult::STARTED);
}

// =============================================================================
// 27.1 FirstTrackingCycleDoesNotPublishProgress
// =============================================================================

TEST(NavigationCoordinatorTest, FirstTrackingCycleDoesNotPublishProgress)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);  // SET_ROUTE

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Small error → directly ALIGNED → TRACKING (same cycle)
  CoreInput robot = makeRobotInput(0, 0, 10.0 * kDeg);
  CoreOutput output = coordinator.update(robot, 1.2);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_FALSE(output.route_progress.valid);
}

// =============================================================================
// 27.2 TrackingPublishesRouteProgressOnNextCycle
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingPublishesRouteProgressOnNextCycle)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Enter TRACKING
  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // Next cycle: robot at (3, 0) on the route
  CoreInput robot1 = makeRobotInput(3, 0, 0);
  CoreOutput output = coordinator.update(robot1, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_TRUE(output.route_progress.valid);
  EXPECT_EQ(output.route_progress.task_sequence, 1u);
  EXPECT_NEAR(output.route_progress.arc_length_m, 3.0, 1e-9);
}

// =============================================================================
// 27.3 TrackingStillOutputsSafeZero
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingStillOutputsSafeZero)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  CoreInput robot1 = makeRobotInput(3, 0, 0);
  CoreOutput output = coordinator.update(robot1, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_TRUE(output.route_progress.valid);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::TRACKING_STOP);
}

// =============================================================================
// 27.4 TrackingWaitsForRobotWithoutFailure
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingWaitsForRobotWithoutFailure)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // Invalid robot
  CoreInput bad_robot{};
  CoreOutput output = coordinator.update(bad_robot, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_FALSE(output.route_progress.valid);
  EXPECT_EQ(output.final_cmd.source, CommandSource::TRACKING_STOP);
}

// =============================================================================
// 27.5 TrackingRouteProgressNeverRegresses
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingRouteProgressNeverRegresses)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // First: robot at x=5
  CoreInput robot1 = makeRobotInput(5, 0, 0);
  CoreOutput out1 = coordinator.update(robot1, 1.3);

  EXPECT_EQ(out1.state, NavState::TRACKING);
  EXPECT_TRUE(out1.route_progress.valid);
  double arc1 = out1.route_progress.arc_length_m;

  // Second: robot jitters back to x=4
  CoreInput robot2 = makeRobotInput(4, 0, 0);
  CoreOutput out2 = coordinator.update(robot2, 1.4);

  EXPECT_EQ(out2.state, NavState::TRACKING);
  EXPECT_TRUE(out2.route_progress.valid);
  EXPECT_GE(out2.route_progress.arc_length_m, arc1);
}

// =============================================================================
// 27.6 MaxVxUpdateDoesNotResetRouteProgress
// =============================================================================

TEST(NavigationCoordinatorTest, MaxVxUpdateDoesNotResetRouteProgress)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // First: progress at x=3
  CoreInput robot1 = makeRobotInput(3, 0, 0);
  CoreOutput out1 = coordinator.update(robot1, 1.3);

  EXPECT_TRUE(out1.route_progress.valid);
  double arc1 = out1.route_progress.arc_length_m;

  // Update max_vx
  NavigationEvent speedUp{};
  speedUp.type = NavigationEventType::UPDATE_MAX_VX;
  speedUp.max_vx = 0.6;
  coordinator.handleEvent(speedUp);

  // Next cycle: robot at x=5, progress should continue
  CoreInput robot2 = makeRobotInput(5, 0, 0);
  CoreOutput out2 = coordinator.update(robot2, 1.4);

  EXPECT_EQ(out2.state, NavState::TRACKING);
  EXPECT_TRUE(out2.route_progress.valid);
  EXPECT_GT(out2.route_progress.arc_length_m, arc1);
}

// =============================================================================
// 27.7 CancelFromTrackingClearsProgress
// =============================================================================

TEST(NavigationCoordinatorTest, CancelFromTrackingClearsProgress)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  CoreInput robot1 = makeRobotInput(3, 0, 0);
  coordinator.update(robot1, 1.3);

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);

  CoreOutput output = coordinator.update(CoreInput{}, 1.4);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_FALSE(output.route_progress.valid);
  EXPECT_FALSE(coordinator.hasActiveTask());
}

// =============================================================================
// 27.8 NewTaskStartsFreshRouteProgress
// =============================================================================

TEST(NavigationCoordinatorTest, NewTaskStartsFreshRouteProgress)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  CoreInput robot1 = makeRobotInput(3, 0, 0);
  coordinator.update(robot1, 1.3);

  // Cancel task 1
  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);
  coordinator.update(CoreInput{}, 1.4);

  // Start task 2 with a different route
  NavigationEvent event2 = makeMultiPointStartEvent();
  coordinator.handleEvent(event2);
  coordinator.update(CoreInput{}, 1.5);  // SET_ROUTE for task 2

  CoreInput ready2 = makePlannerInput(PlannerState::READY, 2u, 1.6);
  coordinator.update(ready2, 1.6);

  CoreInput robot2 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot2, 1.7);

  // Next cycle in TRACKING
  CoreInput robot3 = makeRobotInput(5, 0, 0);
  CoreOutput output = coordinator.update(robot3, 1.8);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_TRUE(output.route_progress.valid);
  EXPECT_EQ(output.route_progress.task_sequence, 2u);
}

// =============================================================================
// 27.9 InvalidRouteProgressConfigTransitionsToFailed
// =============================================================================

TEST(NavigationCoordinatorTest, InvalidRouteProgressConfigTransitionsToFailed)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavdogConfig config;
  config.route_progress.max_forward_search_m = 0.0;
  NavigationCoordinator coordinator(config);

  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Enter TRACKING
  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // Next cycle: invalid config → FAILED
  CoreInput robot1 = makeRobotInput(3, 0, 0);
  CoreOutput output = coordinator.update(robot1, 1.3);

  EXPECT_EQ(output.state, NavState::FAILED);
  EXPECT_EQ(output.final_cmd.source, CommandSource::FAILED_STOP);
  EXPECT_TRUE(coordinator.hasActiveTask());
}

// =============================================================================
// 27.10 SinglePointTaskDoesNotFailInTracking
// =============================================================================

TEST(NavigationCoordinatorTest, SinglePointTaskDoesNotFailInTracking)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());  // single point at (1,0)
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Enter TRACKING (robot at origin facing the target)
  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // Next cycle: single point route should be valid
  CoreInput robot1 = makeRobotInput(0.5, 0, 0);
  CoreOutput output = coordinator.update(robot1, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_TRUE(output.route_progress.valid);
}

// =============================================================================
// 28.1 FirstTrackingCycleDoesNotPublishCorridor
// =============================================================================

TEST(NavigationCoordinatorTest, FirstTrackingCycleDoesNotPublishCorridor)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  // Small error → directly ALIGNED → TRACKING (same cycle)
  CoreInput robot = makeRobotInput(0, 0, 10.0 * kDeg);
  CoreOutput output = coordinator.update(robot, 1.2);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_FALSE(output.route_corridor.valid);
}

// =============================================================================
// 28.2 TrackingAcceptsClearScan3DAssessment
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingAcceptsClearScan3DAssessment)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // Next cycle: robot at (3,0), valid clear SCAN observation
  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_TRUE(output.route_progress.valid);
  EXPECT_TRUE(output.route_corridor.valid);
  EXPECT_FALSE(output.route_corridor.blocked);
}

// =============================================================================
// 28.3 TrackingAcceptsBlockedScan3DAssessment
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingAcceptsBlockedScan3DAssessment)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // Next cycle: blocked SCAN observation
  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeBlockedScanObservation(1u, 3.0, 1.3);
  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_TRUE(output.route_corridor.valid);
  EXPECT_TRUE(output.route_corridor.blocked);
}

// =============================================================================
// 28.4 TrackingStillOutputsZeroWhenClear
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingStillOutputsZeroWhenClear)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::TRACKING_STOP);
}

// =============================================================================
// 28.5 TrackingStillOutputsZeroWhenBlocked
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingStillOutputsZeroWhenBlocked)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeBlockedScanObservation(1u, 3.0, 1.3);
  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_TRUE(output.route_corridor.blocked);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::TRACKING_STOP);
}

// =============================================================================
// 28.6 TrackingWaitsForMissingAssessment
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingWaitsForMissingAssessment)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // route_corridor_observation.valid defaults to false
  CoreInput input = makeRobotInput(3, 0, 0);
  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_FALSE(output.route_corridor.valid);
  EXPECT_EQ(output.final_cmd.source, CommandSource::TRACKING_STOP);
}

// =============================================================================
// 28.7 TrackingRejectsMismatchedTaskAssessmentWithoutFailure
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingRejectsMismatchedTaskAssessmentWithoutFailure)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // Assessment with wrong task sequence
  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(999u, 3.0, 1.3);
  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_FALSE(output.route_corridor.valid);
  EXPECT_EQ(output.final_cmd.source, CommandSource::TRACKING_STOP);
}

// =============================================================================
// 28.8 TrackingRejectsStaleMapAssessmentWithoutFailure
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingRejectsStaleMapAssessmentWithoutFailure)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // Stale map (map_timeout_sec default = 0.30)
  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  input.route_corridor_observation.map_stamp_sec =
      1.3 - 0.31;
  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_FALSE(output.route_corridor.valid);
  EXPECT_EQ(output.final_cmd.source, CommandSource::TRACKING_STOP);
}

// =============================================================================
// 28.9 TrackingRejectsStaleProgressAssessmentWithoutFailure
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingRejectsStaleProgressAssessmentWithoutFailure)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // Robot at x=5, but observation evaluated from x=2
  // progress_lag = 5.0 - 2.0 = 3.0 > max_progress_lag_m(0.50)
  CoreInput input = makeRobotInput(5, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 2.0, 1.3);
  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_FALSE(output.route_corridor.valid);
  EXPECT_EQ(output.final_cmd.source, CommandSource::TRACKING_STOP);
}

// =============================================================================
// 28.10 TrackingRejectsOutOfMapAssessmentWithoutFailure
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingRejectsOutOfMapAssessmentWithoutFailure)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // out_of_map = true
  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  input.route_corridor_observation.out_of_map = true;
  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_FALSE(output.route_corridor.valid);
  EXPECT_EQ(output.final_cmd.source, CommandSource::TRACKING_STOP);
}

// =============================================================================
// 28.11 TrackingInvalidObservationConfigTransitionsToFailed
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingInvalidObservationConfigTransitionsToFailed)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavdogConfig config;
  config.route_corridor_observation.map_timeout_sec = 0.0;
  NavigationCoordinator coordinator(config);

  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // Next cycle: invalid observation config → FAILED
  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::FAILED);
  EXPECT_EQ(output.final_cmd.source, CommandSource::FAILED_STOP);
  EXPECT_TRUE(coordinator.hasActiveTask());
}

// =============================================================================
// 28.12 TrackingDoesNotUseObstacleSummaryForRouteBlocking
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingDoesNotUseObstacleSummaryForRouteBlocking)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // ObstacleSummary has very small front_min
  // but route_corridor_observation is CLEAR
  CoreInput input = makeRobotInput(3, 0, 0);
  input.obstacles.front_min = 0.01;
  input.obstacles.valid = true;
  input.obstacles.stamp_sec = 1.3;
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_TRUE(output.route_corridor.valid);
  EXPECT_FALSE(output.route_corridor.blocked);
}

// =============================================================================
// 28.13 CancelDoesNotReusePreviousAssessment
// =============================================================================

TEST(NavigationCoordinatorTest, CancelDoesNotReusePreviousAssessment)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  coordinator.update(input, 1.3);

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);

  CoreOutput output = coordinator.update(CoreInput{}, 1.4);

  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_FALSE(output.route_corridor.valid);
  EXPECT_FALSE(coordinator.hasActiveTask());
}

// =============================================================================
// 28.14 NewTaskDoesNotAcceptOldSequenceAssessment
// =============================================================================

TEST(NavigationCoordinatorTest, NewTaskDoesNotAcceptOldSequenceAssessment)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  CoreInput input1 = makeRobotInput(3, 0, 0);
  input1.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  coordinator.update(input1, 1.3);

  // Cancel and start new task
  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);
  coordinator.update(CoreInput{}, 1.4);

  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.5);

  CoreInput ready2 = makePlannerInput(PlannerState::READY, 2u, 1.6);
  coordinator.update(ready2, 1.6);

  // Enter TRACKING for task 2 with old sequence assessment
  CoreInput robot2 = makeRobotInput(0, 0, 10.0 * kDeg);
  robot2.route_corridor_observation =
      makeClearScanObservation(1u, 0.0, 1.7);
  CoreOutput output = coordinator.update(robot2, 1.7);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_FALSE(output.route_corridor.valid);
}

// =============================================================================
// 28.15 SinglePointTaskPublishesCorridorAssessment
// =============================================================================

TEST(NavigationCoordinatorTest, SinglePointTaskPublishesCorridorAssessment)
{
  constexpr double kDeg = 3.14159265358979323846 / 180.0;

  NavigationCoordinator coordinator;
  coordinator.handleEvent(makeValidStartEvent());  // single point at (1,0)
  coordinator.update(CoreInput{}, 1.0);

  CoreInput ready = makePlannerInput(PlannerState::READY, 1u, 1.1);
  coordinator.update(ready, 1.1);

  CoreInput robot0 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot0, 1.2);

  // Next cycle: single point route, valid clear observation
  CoreInput input = makeRobotInput(0.5, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 0.0, 1.3);
  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_TRUE(output.route_progress.valid);
  EXPECT_TRUE(output.route_corridor.valid);
  EXPECT_FALSE(output.route_corridor.blocked);
}

// =============================================================================
// 29.1 TrackingInitializesRouteFollowMode
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingInitializesRouteFollowMode)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_TRUE(output.navigation_mode.initialized);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
}

// =============================================================================
// 29.2 TrackingClearKeepsRouteFollowMode
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingClearKeepsRouteFollowMode)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeTrackingInput(3, 0, 0, 1u, 3.0, 1.3);
  coordinator.update(input, 1.3);

  CoreOutput output = coordinator.update(input, 1.4);
  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_FALSE(output.navigation_mode.transitioned);
}

// =============================================================================
// 29.3 TrackingFarBlockedDoesNotEnterAvoid
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingFarBlockedDoesNotEnterAvoid)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 1.3, 2.0);
  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_TRUE(output.navigation_mode.route_blocked);
  EXPECT_FALSE(output.navigation_mode.route_blocked_near);
}

// =============================================================================
// 29.4 TrackingNearBlockedRequiresConfirmation
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingNearBlockedRequiresConfirmation)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput clear_input = makeTrackingInput(3, 0, 0, 1u, 3.0, 1.3);
  coordinator.update(clear_input, 1.3);

  CoreInput blocked_input = makeRobotInput(3, 0, 0);
  blocked_input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 1.4, 1.0);
  CoreOutput output = coordinator.update(blocked_input, 1.4);

  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_EQ(output.navigation_mode.reason,
            NavigationModeReason::BLOCK_CONFIRMING);
}

// =============================================================================
// 29.5 TrackingImmediateBlockedEntersLocalAvoid
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingImmediateBlockedEntersLocalAvoid)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput clear_input = makeTrackingInput(3, 0, 0, 1u, 3.0, 1.3);
  coordinator.update(clear_input, 1.3);

  CoreInput blocked_input = makeRobotInput(3, 0, 0);
  blocked_input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 1.4, 0.5);
  CoreOutput output = coordinator.update(blocked_input, 1.4);

  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.navigation_mode.reason,
            NavigationModeReason::BLOCK_IMMEDIATE);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
}

// =============================================================================
// 29.6 TrackingClearAfterAvoidEntersRejoin
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingClearAfterAvoidEntersRejoin)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeRobotInput(3, 0, 0);

  // Init + immediate block -> LOCAL_AVOID.
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  coordinator.update(input, 1.3);
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 1.4, 0.5);
  coordinator.update(input, 1.4);

  // CLEAR: min_hold met at t=2.0, clear_confirm at t=2.4.
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 2.0);
  coordinator.update(input, 2.0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 2.4);
  CoreOutput output = coordinator.update(input, 2.4);

  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::ROUTE_REJOIN);
}

// =============================================================================
// 29.7 TrackingAlignedRejoinReturnsRouteFollow
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingAlignedRejoinReturnsRouteFollow)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  coordinator.update(input, 1.3);
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 1.4, 0.5);
  coordinator.update(input, 1.4);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 2.0);
  coordinator.update(input, 2.0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 2.4);
  coordinator.update(input, 2.4);

  // Start rejoin confirmation (aligned at t=2.4).
  coordinator.update(input, 2.4);

  // Confirm at t=2.7 (0.3s after timer start).
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 2.7);
  CoreOutput output = coordinator.update(input, 2.7);

  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_EQ(output.navigation_mode.reason,
            NavigationModeReason::REJOIN_COMPLETE);
}

// =============================================================================
// 29.8 TrackingRejoinBlockedReturnsLocalAvoid
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingRejoinBlockedReturnsLocalAvoid)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  coordinator.update(input, 1.3);
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 1.4, 0.5);
  coordinator.update(input, 1.4);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 2.0);
  coordinator.update(input, 2.0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 2.4);
  coordinator.update(input, 2.4);

  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 2.5, 0.5);
  CoreOutput output = coordinator.update(input, 2.5);

  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.navigation_mode.reason,
            NavigationModeReason::REJOIN_BLOCKED);
}

// =============================================================================
// 29.9 TrackingRouteOnlyNeverEntersLocalAvoid
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingRouteOnlyNeverEntersLocalAvoid)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator, TaskMode::ROUTE_ONLY);

  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  coordinator.update(input, 1.3);

  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 1.4, 0.5);
  CoreOutput output = coordinator.update(input, 1.4);

  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_EQ(output.navigation_mode.reason,
            NavigationModeReason::ROUTE_ONLY_BLOCKED);
}

// =============================================================================
// 29.10 TrackingChargingAllowsLocalAvoid
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingChargingAllowsLocalAvoid)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator, TaskMode::CHARGING);

  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  coordinator.update(input, 1.3);

  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 1.4, 0.5);
  CoreOutput output = coordinator.update(input, 1.4);

  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::LOCAL_AVOID);
}

// =============================================================================
// 29.11 TrackingMissingCorridorKeepsCurrentMode
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingMissingCorridorKeepsCurrentMode)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeTrackingInput(3, 0, 0, 1u, 3.0, 1.3);
  coordinator.update(input, 1.3);

  // No corridor observation.
  CoreInput no_corridor = makeRobotInput(3, 0, 0);
  CoreOutput output = coordinator.update(no_corridor, 1.4);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::ROUTE_FOLLOW);
}

// =============================================================================
// 29.12 TrackingStaleCorridorKeepsCurrentMode
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingStaleCorridorKeepsCurrentMode)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeTrackingInput(3, 0, 0, 1u, 3.0, 1.3);
  coordinator.update(input, 1.3);

  CoreInput stale_input = makeRobotInput(3, 0, 0);
  stale_input.route_corridor_observation =
      makeStaleScanObservation(1u, 3.0, 1.4);
  CoreOutput output = coordinator.update(stale_input, 1.4);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::ROUTE_FOLLOW);
}

// =============================================================================
// 29.13 TrackingModeStillOutputsSafeZero
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingModeStillOutputsSafeZero)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 1.3, 0.5);
  CoreOutput output = coordinator.update(input, 1.3);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source,
            CommandSource::TRACKING_STOP);
}

// =============================================================================
// 29.14 TrackingModeDoesNotPassPlannerCmd
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingModeDoesNotPassPlannerCmd)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  coordinator.update(input, 1.3);

  input.planner_cmd.vx = 0.5;
  input.planner_cmd.vy = 0.2;
  input.planner_cmd.yaw_rate = 0.3;
  input.planner_cmd.valid = true;
  input.planner_cmd.source = CommandSource::PLANNER;
  CoreOutput output = coordinator.update(input, 1.4);

  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
}

// =============================================================================
// 29.15 MaxVxUpdateDoesNotResetNavigationMode
// =============================================================================

TEST(NavigationCoordinatorTest, MaxVxUpdateDoesNotResetNavigationMode)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeTrackingInput(3, 0, 0, 1u, 3.0, 1.3);
  coordinator.update(input, 1.3);

  NavigationEvent speedUp{};
  speedUp.type = NavigationEventType::UPDATE_MAX_VX;
  speedUp.max_vx = 0.6;
  coordinator.handleEvent(speedUp);

  CoreOutput output = coordinator.update(input, 1.4);
  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_FALSE(output.navigation_mode.transitioned);
}

// =============================================================================
// 29.16 CancelClearsNavigationMode
// =============================================================================

TEST(NavigationCoordinatorTest, CancelClearsNavigationMode)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 1.3, 0.5);
  coordinator.update(input, 1.3);

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);

  CoreOutput output = coordinator.update(CoreInput{}, 1.4);
  EXPECT_EQ(output.state, NavState::IDLE);
  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::NONE);
}

// =============================================================================
// 29.17 FailedStateClearsNavigationMode
// =============================================================================

TEST(NavigationCoordinatorTest, FailedStateClearsNavigationMode)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeTrackingInput(3, 0, 0, 1u, 3.0, 1.3);
  CoreOutput out1 = coordinator.update(input, 1.3);
  EXPECT_EQ(out1.navigation_mode.mode,
            NavigationMode::ROUTE_FOLLOW);

  CoreOutput out2 = coordinator.update(input,
      std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(out2.state, NavState::FAILED);
  EXPECT_EQ(out2.navigation_mode.mode,
            NavigationMode::NONE);
}

// =============================================================================
// 29.18 NewTaskDoesNotReuseOldNavigationMode
// =============================================================================

TEST(NavigationCoordinatorTest, NewTaskDoesNotReuseOldNavigationMode)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  coordinator.update(input, 1.3);
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 1.4, 0.5);
  coordinator.update(input, 1.4);

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);
  coordinator.update(CoreInput{}, 1.5);

  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 1.6);
  CoreInput ready2 = makePlannerInput(PlannerState::READY, 2u, 1.7);
  coordinator.update(ready2, 1.7);

  constexpr double kDeg = 3.14159265358979323846 / 180.0;
  CoreInput robot2 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot2, 1.8);

  CoreInput input2 = makeTrackingInput(3, 0, 0, 2u, 3.0, 1.9);
  CoreOutput output = coordinator.update(input2, 1.9);

  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::ROUTE_FOLLOW);
}

// =============================================================================
// 29.19 NewTaskDoesNotReuseRejoinAnchor
// =============================================================================

TEST(NavigationCoordinatorTest, NewTaskDoesNotReuseRejoinAnchor)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  coordinator.update(input, 1.3);
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 1.4, 0.5);
  coordinator.update(input, 1.4);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 2.0);
  coordinator.update(input, 2.0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 2.4);
  coordinator.update(input, 2.4);
  EXPECT_TRUE(coordinator.update(input, 2.4)
      .navigation_mode.has_rejoin_anchor);

  NavigationEvent cancel{};
  cancel.type = NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);
  coordinator.update(CoreInput{}, 2.5);

  coordinator.handleEvent(makeMultiPointStartEvent());
  coordinator.update(CoreInput{}, 2.6);
  CoreInput ready2 = makePlannerInput(PlannerState::READY, 2u, 2.7);
  coordinator.update(ready2, 2.7);

  constexpr double kDeg = 3.14159265358979323846 / 180.0;
  CoreInput robot2 = makeRobotInput(0, 0, 10.0 * kDeg);
  coordinator.update(robot2, 2.8);

  CoreInput input2 = makeTrackingInput(3, 0, 0, 2u, 3.0, 2.9);
  CoreOutput output = coordinator.update(input2, 2.9);

  EXPECT_FALSE(output.navigation_mode.has_rejoin_anchor);
}

// =============================================================================
// 29.20 TrackingUnavailableCorridorClearsBlockedFlags
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingUnavailableCorridorClearsBlockedFlags)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeRobotInput(3, 0, 0);
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 1.3, 1.0);
  coordinator.update(input, 1.3);

  // Next frame: no corridor observation.
  CoreInput no_corridor = makeRobotInput(3, 0, 0);
  CoreOutput output = coordinator.update(no_corridor, 1.4);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_FALSE(output.navigation_mode.corridor_available);
  EXPECT_FALSE(output.navigation_mode.route_blocked);
  EXPECT_FALSE(output.navigation_mode.route_blocked_near);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source,
            CommandSource::TRACKING_STOP);
}

// =============================================================================
// 29.21 TrackingRejoinBlockClearBlockRequiresFreshConfirmation
// =============================================================================

TEST(NavigationCoordinatorTest, TrackingRejoinBlockClearBlockRequiresFreshConfirmation)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeRobotInput(3, 0, 0);

  // Init + immediate block → LOCAL_AVOID.
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 1.3);
  coordinator.update(input, 1.3);
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 1.4, 0.5);
  coordinator.update(input, 1.4);

  // CLEAR → ROUTE_REJOIN.
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 2.0);
  coordinator.update(input, 2.0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 2.4);
  coordinator.update(input, 2.4);
  // Extra update to start rejoin timer.
  coordinator.update(input, 2.4);

  // ROUTE_REJOIN: near BLOCKED at 1.0m — start blocked timer.
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 2.5, 1.0);
  coordinator.update(input, 2.5);

  // CLEAR — resets blocked timer.
  input.route_corridor_observation =
      makeClearScanObservation(1u, 3.0, 2.6);
  coordinator.update(input, 2.6);

  // BLOCKED again at 1.0m — timer should start fresh.
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 2.7, 1.0);
  CoreOutput output = coordinator.update(input, 2.7);
  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::ROUTE_REJOIN);
  EXPECT_EQ(output.navigation_mode.reason,
            NavigationModeReason::BLOCK_CONFIRMING);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source,
            CommandSource::TRACKING_STOP);

  // Continue BLOCKED — only 0.1s since fresh start, not enough.
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 2.8, 1.0);
  output = coordinator.update(input, 2.8);
  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::ROUTE_REJOIN);

  // 0.2s continuous — should enter LOCAL_AVOID.
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 3.0, 2.9, 1.0);
  output = coordinator.update(input, 2.9);
  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.navigation_mode.reason,
            NavigationModeReason::REJOIN_BLOCKED);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_EQ(output.final_cmd.source,
            CommandSource::TRACKING_STOP);
}

// =============================================================================
// TrackingRouteFollowOutputsNonZeroVelocity
// =============================================================================

TEST(NavigationCoordinatorTest,
     TrackingRouteFollowOutputsNonZeroVelocity)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  CoreInput input = makeTrackingInput(0.5, 0.0, 0.0, 1u, 0.5, 2.0);
  CoreOutput output = coordinator.update(input, 2.0);

  EXPECT_EQ(output.state, NavState::TRACKING);
  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_GT(output.final_cmd.vx, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::PLANNER);
}

// =============================================================================
// LocalAvoidWithoutTrajectoryStops
// =============================================================================

TEST(NavigationCoordinatorTest,
     LocalAvoidWithoutTrajectoryStops)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  FakeLocalPlannerAdapter adapter;
  coordinator.setLocalPlannerAdapter(&adapter);

  // Trigger LOCAL_AVOID with immediate obstacle.
  CoreInput input = makeTrackingInput(0.0, 0.0, 0.0, 1u, 0.0, 2.0);
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 0.0, 2.0, 0.5);

  CoreOutput output = coordinator.update(input, 2.0);

  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.final_cmd.vx, 0.0);
  EXPECT_EQ(output.final_cmd.source,
            CommandSource::TRACKING_STOP);
}

// =============================================================================
// LocalAvoidAcceptsMatchingTrajectory
// =============================================================================

TEST(NavigationCoordinatorTest,
     LocalAvoidAcceptsMatchingTrajectory)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  FakeLocalPlannerAdapter adapter;
  LocalTrajectory trajectory = makeTestLocalTrajectory(
      1u, 1u, NavigationMode::LOCAL_AVOID, 2.0, 2.0);
  adapter.setTrajectory(trajectory);
  coordinator.setLocalPlannerAdapter(&adapter);

  CoreInput input = makeTrackingInput(0.0, 0.0, 0.0, 1u, 0.0, 2.0);
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 0.0, 2.0, 0.5);

  CoreOutput output = coordinator.update(input, 2.0);

  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_GT(output.final_cmd.vx, 0.0);
  EXPECT_EQ(output.final_cmd.source, CommandSource::PLANNER);
}

// =============================================================================
// ExpiredTrajectoryStops
// =============================================================================

TEST(NavigationCoordinatorTest, ExpiredTrajectoryStops)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  FakeLocalPlannerAdapter adapter;
  LocalTrajectory trajectory = makeTestLocalTrajectory(
      1u, 1u, NavigationMode::LOCAL_AVOID, 1.0, 1.0);
  adapter.setTrajectory(trajectory);
  coordinator.setLocalPlannerAdapter(&adapter);

  CoreInput input = makeTrackingInput(0.0, 0.0, 0.0, 1u, 0.0, 2.0);
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 0.0, 2.0, 0.5);

  // elapsed = 2.0 - 1.0 = 1.0s, duration = 1.0s -> expired.
  CoreOutput output = coordinator.update(input, 2.0);

  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::LOCAL_AVOID);
  EXPECT_EQ(output.final_cmd.vx, 0.0);
  EXPECT_EQ(output.final_cmd.source,
            CommandSource::TRACKING_STOP);
}

// =============================================================================
// ReturningToRouteFollowClearsOldTrajectory
// =============================================================================

TEST(NavigationCoordinatorTest,
     ReturningToRouteFollowClearsOldTrajectory)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  FakeLocalPlannerAdapter adapter;
  LocalTrajectory trajectory = makeTestLocalTrajectory(
      1u, 1u, NavigationMode::LOCAL_AVOID, 2.0, 2.0);
  adapter.setTrajectory(trajectory);
  coordinator.setLocalPlannerAdapter(&adapter);

  // Enter LOCAL_AVOID.
  CoreInput input = makeTrackingInput(0.0, 0.0, 0.0, 1u, 0.0, 2.0);
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 0.0, 2.0, 0.5);
  coordinator.update(input, 2.0);

  // Return to ROUTE_FOLLOW with CLEAR corridor.
  input = makeTrackingInput(0.2, 0.0, 0.0, 1u, 0.2, 3.0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 0.2, 3.0);
  CoreOutput output = coordinator.update(input, 3.0);

  EXPECT_EQ(output.navigation_mode.mode,
            NavigationMode::ROUTE_FOLLOW);
  EXPECT_EQ(output.final_cmd.source, CommandSource::PLANNER);
  EXPECT_GT(output.final_cmd.vx, 0.0);
}

// =============================================================================
// TrajectoryCollisionTriggersReplan
// =============================================================================

TEST(NavigationCoordinatorTest,
     TrajectoryCollisionTriggersReplan)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  FakeLocalPlannerAdapter adapter;
  LocalTrajectory trajectory = makeTestLocalTrajectory(
      1u, 1u, NavigationMode::LOCAL_AVOID, 2.0, 2.0);
  adapter.setTrajectory(trajectory);
  coordinator.setLocalPlannerAdapter(&adapter);

  // Enter LOCAL_AVOID and consume the first plan request.
  CoreInput input = makeTrackingInput(0.0, 0.0, 0.0, 1u, 0.0, 2.0);
  input.route_corridor_observation =
      makeBlockedScanObservationAt(1u, 0.0, 2.0, 0.5);
  coordinator.update(input, 2.0);

  const std::uint64_t count_after_first = adapter.requestCount();

  // Mark trajectory as colliding; next update should request replan.
  adapter.setCollisionFlag(true);
  coordinator.update(input, 2.05);

  EXPECT_GT(adapter.requestCount(), count_after_first);
}

// =============================================================================
// GoalFinishOutputsZeroVelocity
// =============================================================================

TEST(NavigationCoordinatorTest, GoalFinishOutputsZeroVelocity)
{
  NavigationCoordinator coordinator;
  setupToTracking(coordinator);

  // Near goal.
  CoreInput input = makeTrackingInput(9.95, 0.0, 0.0, 1u, 9.95, 2.0);
  input.route_corridor_observation =
      makeClearScanObservation(1u, 9.95, 2.0);
  CoreOutput output = coordinator.update(input, 2.0);

  EXPECT_EQ(output.state, NavState::SUCCEEDED);
  EXPECT_EQ(output.final_cmd.vx, 0.0);
  EXPECT_EQ(output.final_cmd.vy, 0.0);
  EXPECT_EQ(output.final_cmd.yaw_rate, 0.0);
}

}  // namespace
}  // namespace navdog

// =============================================================================
// main
// =============================================================================

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
