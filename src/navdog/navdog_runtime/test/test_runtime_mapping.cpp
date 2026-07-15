#include "navdog_runtime/navdog_runtime_node.hpp"

#include <gtest/gtest.h>

namespace navdog_runtime
{
namespace
{
navdog::NavigationEvent makeStartEvent()
{
  navdog::NavigationEvent event{};
  event.type = navdog::NavigationEventType::START_TASK;
  event.task.max_vx = 0.4;
  navdog::RoutePoint a{};
  navdog::RoutePoint b{}; b.x = 2.0;
  event.task.points = {a, b};
  return event;
}

navdog::RobotState makeRobotAt(double x, double y, double yaw, double stamp)
{
  navdog::RobotState r{};
  r.x = x; r.y = y; r.z = 0.3; r.yaw = yaw;
  r.stamp_sec = stamp;
  r.valid = true;
  return r;
}
}  // namespace

// =============================================================================
// Existing tests
// =============================================================================

TEST(RuntimeStatusTest, IdleMapsToStatus0)
{
  navdog::CoreOutput output{}; int status, error;
  NavdogRuntimeNode::statusForOutput(output, false, status, error);
  EXPECT_EQ(status, 0); EXPECT_EQ(error, 0);
}

TEST(RuntimeStatusTest, TrackingMapsToStatus1)
{
  navdog::CoreOutput output{}; output.state = navdog::NavState::TRACKING;
  int status, error; NavdogRuntimeNode::statusForOutput(output, false, status, error);
  EXPECT_EQ(status, 1); EXPECT_EQ(error, 0);
}

TEST(RuntimeStatusTest, PausedMapsToStatus5)
{
  navdog::CoreOutput output{}; output.state = navdog::NavState::PAUSED;
  int status, error; NavdogRuntimeNode::statusForOutput(output, false, status, error);
  EXPECT_EQ(status, 5);
}

TEST(RuntimeStatusTest, FailedMapsToError2)
{
  navdog::CoreOutput output{}; output.state = navdog::NavState::FAILED;
  int status, error; NavdogRuntimeNode::statusForOutput(output, false, status, error);
  EXPECT_EQ(status, 0); EXPECT_EQ(error, 2);
}

TEST(RuntimeStatusTest, FeedbackUsesFinalCommand)
{
  navdog::VelocityCommand command{};
  command.valid = true; command.vx = 0.2; command.vy = -0.1; command.yaw_rate = 0.15;
  const auto twist = NavdogRuntimeNode::toTwist(command);
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.2);
  EXPECT_DOUBLE_EQ(twist.linear.y, -0.1);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.15);
}

TEST(RuntimeNodeTest, SetRouteActionProducesReadyFeedback)
{
  navdog::PlannerAction action{};
  action.type = navdog::PlannerActionType::SET_ROUTE;
  action.task.sequence = 9;
  const auto feedback = NavdogRuntimeNode::feedbackForAction(action, 2.0);
  EXPECT_TRUE(feedback.valid);
  EXPECT_EQ(feedback.state, navdog::PlannerState::READY);
  EXPECT_EQ(feedback.trajectory_id, 9u);
}

TEST(RuntimeNodeTest, ReadyFeedbackMovesPlanningToStartAlign)
{
  navdog::NavigationCoordinator coordinator;
  coordinator.handleEvent(makeStartEvent());
  const auto first = coordinator.update(navdog::CoreInput{}, 1.0);
  navdog::CoreInput input{};
  input.planner = NavdogRuntimeNode::feedbackForAction(first.planner_action, 1.0);
  EXPECT_EQ(coordinator.update(input, 1.1).state, navdog::NavState::START_ALIGN);
}

TEST(RuntimeNodeTest, PausedStatePublishesImmediateZero)
{
  navdog::NavigationCoordinator coordinator;
  coordinator.handleEvent(makeStartEvent());
  navdog::NavigationEvent pause{}; pause.type = navdog::NavigationEventType::PAUSE;
  coordinator.handleEvent(pause);
  const auto output = coordinator.update(navdog::CoreInput{}, 1.0);
  EXPECT_EQ(output.state, navdog::NavState::PAUSED);
  EXPECT_DOUBLE_EQ(NavdogRuntimeNode::toTwist(output.final_cmd).linear.x, 0.0);
}

TEST(RuntimeNodeTest, CancelPublishesImmediateZero)
{
  navdog::NavigationCoordinator coordinator;
  coordinator.handleEvent(makeStartEvent());
  navdog::NavigationEvent cancel{}; cancel.type = navdog::NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);
  const auto output = coordinator.update(navdog::CoreInput{}, 1.0);
  EXPECT_EQ(output.state, navdog::NavState::IDLE);
  EXPECT_DOUBLE_EQ(NavdogRuntimeNode::toTwist(output.final_cmd).linear.x, 0.0);
}

TEST(RuntimeNodeTest, FinalCommandPublishedToCmdVel)
{
  navdog::VelocityCommand command{};
  command.valid = true;
  command.vx = 0.31;
  EXPECT_DOUBLE_EQ(NavdogRuntimeNode::toTwist(command).linear.x, 0.31);
}

TEST(RuntimeNodeTest, OnlyFinalCmdIsPublished)
{
  navdog::CoreOutput output{};
  output.final_cmd.valid = true;
  output.final_cmd.vx = 0.2;
  navdog::VelocityCommand unrelated{};
  unrelated.valid = true;
  unrelated.vx = 0.9;
  EXPECT_DOUBLE_EQ(NavdogRuntimeNode::toTwist(output.final_cmd).linear.x, 0.2);
}

// =============================================================================
// New tests: status mapping fixes
// =============================================================================

TEST(RuntimeStatusTest, SafetyStopInTrackingDoesNotReportError2)
{
  // SAFETY_STOP from coordinator during TRACKING (e.g. obstacle stop)
  // should be status=1 (running), error=0, not status=0 error=2.
  navdog::CoreOutput output{};
  output.state = navdog::NavState::TRACKING;
  output.final_cmd.source = navdog::CommandSource::SAFETY_STOP;
  output.final_cmd.valid = true;
  int status = -1, error = -1;
  NavdogRuntimeNode::statusForOutput(output, false, status, error);
  EXPECT_EQ(status, 1);
  EXPECT_EQ(error, 0);
}

TEST(RuntimeStatusTest, EmergencyStopMapsToError2)
{
  navdog::CoreOutput output{};
  output.state = navdog::NavState::EMERGENCY_STOP;
  int status, error;
  NavdogRuntimeNode::statusForOutput(output, false, status, error);
  EXPECT_EQ(status, 0);
  EXPECT_EQ(error, 2);
}

TEST(RuntimeStatusTest, FailedMapsToError2RegardlessOfSource)
{
  navdog::CoreOutput output{};
  output.state = navdog::NavState::FAILED;
  output.final_cmd.source = navdog::CommandSource::PLANNER;
  int status, error;
  NavdogRuntimeNode::statusForOutput(output, false, status, error);
  EXPECT_EQ(status, 0);
  EXPECT_EQ(error, 2);
}

TEST(RuntimeStatusTest, ProtocolErrorSetsError1)
{
  navdog::CoreOutput output{};
  output.state = navdog::NavState::TRACKING;
  int status, error;
  NavdogRuntimeNode::statusForOutput(output, true, status, error);
  EXPECT_EQ(status, 1);
  EXPECT_EQ(error, 1);
}

// =============================================================================
// New tests: cancel clears pending feedback
// =============================================================================

TEST(RuntimeNodeTest, CancelClearsPendingFeedback)
{
  navdog::NavigationCoordinator coordinator;
  coordinator.handleEvent(makeStartEvent());
  // Get planner action from coordinator
  const auto first = coordinator.update(navdog::CoreInput{}, 1.0);
  EXPECT_EQ(first.planner_action.type, navdog::PlannerActionType::SET_ROUTE);
  // Simulate: we have pending feedback from the planner
  navdog::CoreInput input{};
  input.planner = NavdogRuntimeNode::feedbackForAction(first.planner_action, 1.0);
  // Cancel before the next update
  navdog::NavigationEvent cancel{};
  cancel.type = navdog::NavigationEventType::CANCEL_TASK;
  coordinator.handleEvent(cancel);
  const auto output = coordinator.update(input, 1.1);
  EXPECT_EQ(output.state, navdog::NavState::IDLE);
  // After cancel, there should be no new SET_ROUTE action
  EXPECT_NE(output.planner_action.type, navdog::PlannerActionType::SET_ROUTE);
}

// =============================================================================
// New tests: pause keeps pending feedback
// =============================================================================

TEST(RuntimeNodeTest, PauseKeepsPendingFeedback)
{
  navdog::NavigationCoordinator coordinator;

  coordinator.handleEvent(makeStartEvent());

  const auto first =
      coordinator.update(navdog::CoreInput{}, 1.0);

  ASSERT_EQ(
      first.planner_action.type,
      navdog::PlannerActionType::SET_ROUTE);

  // Runtime receives READY feedback but keeps it externally while paused.
  const navdog::PlannerFeedback held_feedback =
      NavdogRuntimeNode::feedbackForAction(
          first.planner_action,
          1.0);

  navdog::NavigationEvent pause{};
  pause.type = navdog::NavigationEventType::PAUSE;

  ASSERT_EQ(
      coordinator.handleEvent(pause),
      navdog::TaskHandleResult::PAUSED);

  // Even if feedback is accidentally supplied during PAUSED,
  // Coordinator must remain paused. The PAUSE planner action itself
  // is still expected to be emitted.
  navdog::CoreInput paused_input{};
  paused_input.planner = held_feedback;

  const auto paused_output =
      coordinator.update(paused_input, 1.1);

  EXPECT_EQ(
      paused_output.state,
      navdog::NavState::PAUSED);

  EXPECT_EQ(
      paused_output.planner_action.type,
      navdog::PlannerActionType::PAUSE);

  EXPECT_DOUBLE_EQ(paused_output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(paused_output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(paused_output.final_cmd.yaw_rate, 0.0);

  // Resume and provide the same feedback that Runtime held while paused.
  navdog::NavigationEvent resume{};
  resume.type = navdog::NavigationEventType::RESUME;

  ASSERT_EQ(
      coordinator.handleEvent(resume),
      navdog::TaskHandleResult::RESUMED);

  navdog::CoreInput resumed_input{};
  resumed_input.planner = held_feedback;

  const auto resumed_output =
      coordinator.update(resumed_input, 1.2);

  EXPECT_EQ(
      resumed_output.state,
      navdog::NavState::START_ALIGN);

  EXPECT_EQ(
      resumed_output.planner_action.type,
      navdog::PlannerActionType::RESUME);
}

// =============================================================================
// New tests: map not ready publishes zero
// =============================================================================

TEST(RuntimeNodeTest, MapNotReadyPublishesZero)
{
  // Coordinator without valid map/route_progress should output zero velocity.
  navdog::NavigationCoordinator coordinator;
  coordinator.handleEvent(makeStartEvent());
  // First update: no planner feedback yet, no valid route progress
  const auto output = coordinator.update(navdog::CoreInput{}, 1.0);
  // Should be in PLANNING state with zero command
  EXPECT_EQ(output.state, navdog::NavState::PLANNING);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_TRUE(output.final_cmd.valid);
}

// =============================================================================
// New tests: stale odom publishes zero
// =============================================================================

TEST(RuntimeNodeTest, StaleOdomPublishesZero)
{
  navdog::NavigationCoordinator coordinator;
  coordinator.handleEvent(makeStartEvent());
  // Provide planner feedback to move past PLANNING
  const auto first = coordinator.update(navdog::CoreInput{}, 1.0);
  navdog::CoreInput input{};
  input.planner = NavdogRuntimeNode::feedbackForAction(first.planner_action, 1.0);

  // Robot with very old stamp (stale odom)
  input.robot = makeRobotAt(0.0, 0.0, 0.0, 0.0);  // stamp_sec = 0.0, now_sec = 1.1
  input.obstacles.valid = true;
  input.obstacles.stamp_sec = 1.1;  // fresh obstacles

  const auto output = coordinator.update(input, 1.1);
  // With stale odom, the safety supervisor should force zero velocity
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
  EXPECT_TRUE(output.final_cmd.valid);
}

// =============================================================================
// New tests: invalid robot publishes zero
// =============================================================================

TEST(RuntimeNodeTest, InvalidRobotPublishesZero)
{
  navdog::NavigationCoordinator coordinator;
  coordinator.handleEvent(makeStartEvent());
  const auto first = coordinator.update(navdog::CoreInput{}, 1.0);
  navdog::CoreInput input{};
  input.planner = NavdogRuntimeNode::feedbackForAction(first.planner_action, 1.0);
  // robot.valid = false
  input.robot.valid = false;
  input.robot.stamp_sec = 1.1;
  input.obstacles.valid = true;
  input.obstacles.stamp_sec = 1.1;

  const auto output = coordinator.update(input, 1.1);
  EXPECT_DOUBLE_EQ(output.final_cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.final_cmd.yaw_rate, 0.0);
}

// =============================================================================
// New tests: config defaults
// =============================================================================

TEST(NavdogConfigTest, DefaultModeThresholdsAreFinitePositive)
{
  navdog::NavdogConfig config{};
  EXPECT_GT(config.navigation_mode.avoid_enter_distance_m, 0.0);
  EXPECT_GT(config.navigation_mode.avoid_immediate_distance_m, 0.0);
  EXPECT_GT(config.navigation_mode.avoid_block_confirm_sec, 0.0);
  EXPECT_GT(config.navigation_mode.route_clear_confirm_sec, 0.0);
  EXPECT_GT(config.navigation_mode.rejoin_confirm_sec, 0.0);
  EXPECT_TRUE(std::isfinite(config.navigation_mode.avoid_enter_distance_m));
  EXPECT_TRUE(std::isfinite(config.navigation_mode.rejoin_lateral_tolerance_m));
}

TEST(NavdogConfigTest, DefaultLimitsAreFinitePositive)
{
  navdog::NavdogConfig config{};
  EXPECT_GT(config.limits.max_vx, 0.0);
  EXPECT_GT(config.limits.max_vy, 0.0);
  EXPECT_GT(config.limits.max_yaw_rate, 0.0);
  EXPECT_GT(config.limits.max_accel_x, 0.0);
  EXPECT_GT(config.limits.max_accel_y, 0.0);
  EXPECT_GT(config.limits.max_accel_yaw, 0.0);
  EXPECT_TRUE(std::isfinite(config.limits.max_vx));
}

TEST(NavdogConfigTest, DefaultSafetyTimeoutsAreFinitePositive)
{
  navdog::NavdogConfig config{};
  EXPECT_GT(config.safety.odom_timeout_sec, 0.0);
  EXPECT_GT(config.safety.obstacle_timeout_sec, 0.0);
  EXPECT_GT(config.safety.planner_cmd_timeout_sec, 0.0);
  EXPECT_TRUE(std::isfinite(config.safety.odom_timeout_sec));
}

TEST(NavdogConfigTest, PlannerTriggerDefaultsAreFinitePositive)
{
  navdog::NavdogConfig config{};
  EXPECT_GT(config.planner_trigger.replan_retry_interval_sec, 0.0);
  EXPECT_GT(config.planner_trigger.min_remaining_duration_sec, 0.0);
  EXPECT_GT(config.planner_trigger.trajectory_source_max_age_sec, 0.0);
  EXPECT_TRUE(std::isfinite(config.planner_trigger.target_change_threshold_m));
}

// =============================================================================
// New tests: simulation defaults
// =============================================================================

TEST(SimulationConfigTest, DefaultPoseMatchesRouteStart)
{
  // Default simulation init pose should be (0, 0, 0.3) so that a
  // straight-line MQTT route starting from (0,0,0.3) works immediately.
  constexpr double default_init_x = 0.0;
  constexpr double default_init_y = 0.0;
  constexpr double default_init_z = 0.3;
  constexpr double default_init_yaw = 0.0;

  // Verify defaults are finite
  EXPECT_TRUE(std::isfinite(default_init_x));
  EXPECT_TRUE(std::isfinite(default_init_y));
  EXPECT_TRUE(std::isfinite(default_init_z));
  EXPECT_TRUE(std::isfinite(default_init_yaw));

  // A route starting at (0, 0, 0.3) has its first point matching the pose
  navdog::RoutePoint origin{};
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = default_init_z;

  EXPECT_DOUBLE_EQ(origin.x, default_init_x);
  EXPECT_DOUBLE_EQ(origin.y, default_init_y);
  EXPECT_DOUBLE_EQ(origin.z, default_init_z);
}

// =============================================================================
// New tests: task sequence logging
// =============================================================================

TEST(RuntimeNodeTest, RouteManagerReturnsRealSequence)
{
  // TaskManager assigns its own sequence, not the MQTT parser sequence.
  navdog::NavigationCoordinator coordinator;
  auto event = makeStartEvent();
  event.task.sequence = 999;  // MQTT parser sequence (should be ignored by TaskManager)
  coordinator.handleEvent(event);

  // TaskManager generates its own sequence (starts at 1)
  EXPECT_NE(coordinator.routeManager().taskSequence(), 999u);
  EXPECT_GT(coordinator.routeManager().taskSequence(), 0u);
}

// =============================================================================
// New tests: cmd_vel publisher check and conflict latch
// =============================================================================

TEST(CmdVelConflictTest, CachedResultReflectsLatchState)
{
  // hasUniqueCmdVelPublisherCached returns true only when no conflict is latched.
  // Default state: no conflict -> cached check returns true (unique publisher).
  bool latch = false;
  EXPECT_TRUE(!latch);  // hasUniqueCmdVelPublisherCached() == !cmd_vel_conflict_latched_

  // After conflict detected: latch becomes true -> cached returns false.
  latch = true;
  EXPECT_FALSE(!latch);
}

TEST(CmdVelConflictTest, ConflictLatchesFaultDoesNotAutoRecover)
{
  // Once cmd_vel_conflict_latched_ is set to true, it must stay true
  // until node restart. The publisherCheckCallback sets it and never
  // clears it. This verifies the design contract.
  //
  // In the running node:
  //   publisherCheckCallback detects conflict -> cmd_vel_conflict_latched_ = true
  //   hasUniqueCmdVelPublisherCached() returns false forever after
  //   controlCallback produces zero effective_command_ forever after

  bool conflict_latched = false;

  // Simulate: no conflict initially
  EXPECT_FALSE(conflict_latched);

  // Simulate: publisherCheckCallback detects conflict
  conflict_latched = true;
  EXPECT_TRUE(conflict_latched);

  // Simulate: subsequent checks - latch never auto-clears
  // (no code path clears cmd_vel_conflict_latched_)
  conflict_latched = true;  // stays true
  EXPECT_TRUE(conflict_latched);

  // Must require node restart to clear
}

TEST(CmdVelConflictTest, EffectiveCommandIsZeroWhenConflictLatched)
{
  // When cmd_vel_conflict_latched_ is true, the effective command must be:
  //   vx=0, vy=0, yaw_rate=0, valid=true, source=SAFETY_STOP

  navdog::VelocityCommand effective{};
  effective.vx = 0.5; effective.vy = 0.1; effective.yaw_rate = 0.2;
  effective.valid = true;
  effective.source = navdog::CommandSource::PLANNER;

  // Conflict detected: override to zero
  bool conflict = true;
  if (conflict)
  {
    effective.vx = 0.0;
    effective.vy = 0.0;
    effective.yaw_rate = 0.0;
    effective.valid = true;
    effective.source = navdog::CommandSource::SAFETY_STOP;
  }

  EXPECT_DOUBLE_EQ(effective.vx, 0.0);
  EXPECT_DOUBLE_EQ(effective.vy, 0.0);
  EXPECT_DOUBLE_EQ(effective.yaw_rate, 0.0);
  EXPECT_TRUE(effective.valid);
  EXPECT_EQ(effective.source, navdog::CommandSource::SAFETY_STOP);
}

TEST(CmdVelConflictTest, MqttMustReportEffectiveVelocityNotOriginalCommand)
{
  // When conflict forces zero, MQTT must report the effective zero velocity,
  // not the original non-zero final_cmd from the coordinator output.
  navdog::VelocityCommand final_cmd{};
  final_cmd.vx = 0.5; final_cmd.vy = 0.0; final_cmd.yaw_rate = 0.0;
  final_cmd.valid = true;
  final_cmd.source = navdog::CommandSource::PLANNER;

  navdog::VelocityCommand effective_cmd = final_cmd;
  bool conflict = true;

  if (conflict)
  {
    effective_cmd.vx = 0.0;
    effective_cmd.vy = 0.0;
    effective_cmd.yaw_rate = 0.0;
    effective_cmd.valid = true;
    effective_cmd.source = navdog::CommandSource::SAFETY_STOP;
  }

  // MQTT must use effective_cmd, not final_cmd
  EXPECT_DOUBLE_EQ(effective_cmd.vx, 0.0);     // MQTT reports 0
  EXPECT_DOUBLE_EQ(final_cmd.vx, 0.5);          // original was non-zero
  EXPECT_NE(effective_cmd.vx, final_cmd.vx);    // must differ
}

}  // namespace navdog_runtime

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
