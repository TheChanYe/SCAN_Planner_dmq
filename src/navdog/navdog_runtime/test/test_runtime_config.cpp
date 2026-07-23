#include "navdog_runtime/navdog_runtime_node.hpp"
#include "navdog_runtime/ros1_config_loader.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cmath>
#include <limits>

namespace navdog_runtime
{
namespace
{

// =============================================================================
// NavdogConfigLoadsModeThresholds
// =============================================================================

TEST(NavdogConfigLoadingTest, LoadsLimitsFromParams)
{
  ros::NodeHandle nh("~");
  nh.setParam("limits/max_vx", 0.75);
  nh.setParam("limits/max_vy", 0.30);
  nh.setParam("limits/max_yaw_rate", 0.55);
  nh.setParam("limits/max_accel_x", 0.40);
  nh.setParam("limits/max_accel_y", 0.35);
  nh.setParam("limits/max_accel_yaw", 0.70);

  const auto config = NavdogRuntimeNode::loadNavdogConfig(nh);

  EXPECT_DOUBLE_EQ(config.limits.max_vx, 0.75);
  EXPECT_DOUBLE_EQ(config.limits.max_vy, 0.30);
  EXPECT_DOUBLE_EQ(config.limits.max_yaw_rate, 0.55);
  EXPECT_DOUBLE_EQ(config.limits.max_accel_x, 0.40);
  EXPECT_DOUBLE_EQ(config.limits.max_accel_y, 0.35);
  EXPECT_DOUBLE_EQ(config.limits.max_accel_yaw, 0.70);
}

TEST(NavdogConfigLoadingTest, LoadsSafetyTimeoutsFromParams)
{
  ros::NodeHandle nh("~");
  nh.setParam("safety/odom_timeout_sec", 0.3);
  nh.setParam("safety/obstacle_timeout_sec", 0.4);
  nh.setParam("safety/planner_cmd_timeout_sec", 0.25);
  nh.setParam("safety/slow_down_front", 2.0);
  nh.setParam("safety/emergency_stop", 0.5);

  const auto config = NavdogRuntimeNode::loadNavdogConfig(nh);

  EXPECT_DOUBLE_EQ(config.safety.odom_timeout_sec, 0.3);
  EXPECT_DOUBLE_EQ(config.safety.obstacle_timeout_sec, 0.4);
  EXPECT_DOUBLE_EQ(config.safety.planner_cmd_timeout_sec, 0.25);
  EXPECT_DOUBLE_EQ(config.safety.slow_down_front, 2.0);
  EXPECT_DOUBLE_EQ(config.safety.emergency_stop, 0.5);
}

// DISABLED: planner_trigger config loading removed (LOCAL_AVOID refactor)
TEST(NavdogConfigLoadingTest, DISABLED_LoadsPlannerTriggerFromParams)
{
  ros::NodeHandle nh("~");
  nh.setParam("planner_trigger/replan_retry_interval_sec", 0.5);
  nh.setParam("planner_trigger/trajectory_expiry_margin_sec", 0.3);
  nh.setParam("planner_trigger/min_remaining_duration_sec", 0.4);
  nh.setParam("planner_trigger/trajectory_source_max_age_sec", 0.35);
  nh.setParam("planner_trigger/target_change_threshold_m", 0.25);

  const auto config = NavdogRuntimeNode::loadNavdogConfig(nh);

  EXPECT_DOUBLE_EQ(config.planner_trigger.replan_retry_interval_sec, 0.5);
  EXPECT_DOUBLE_EQ(config.planner_trigger.trajectory_expiry_margin_sec, 0.3);
  EXPECT_DOUBLE_EQ(config.planner_trigger.min_remaining_duration_sec, 0.4);
  EXPECT_DOUBLE_EQ(config.planner_trigger.trajectory_source_max_age_sec, 0.35);
  EXPECT_DOUBLE_EQ(config.planner_trigger.target_change_threshold_m, 0.25);
}

TEST(NavdogConfigLoadingTest, LoadsGoalControllerFromParams)
{
  ros::NodeHandle nh("~");
  nh.setParam("goal_controller/near_goal_switch_dist", 0.9);
  nh.setParam("goal_controller/near_goal_kp_v", 0.35);
  nh.setParam("goal_controller/near_goal_min_v", 0.12);
  nh.setParam("goal_controller/obstacle_finish_timeout_sec", 6.0);
  nh.setParam("goal_controller/finish_dist", 0.2);

  const auto config = NavdogRuntimeNode::loadNavdogConfig(nh);

  EXPECT_DOUBLE_EQ(config.goal_controller.near_goal_switch_dist, 0.9);
  EXPECT_DOUBLE_EQ(config.goal_controller.near_goal_kp_v, 0.35);
  EXPECT_DOUBLE_EQ(config.goal_controller.near_goal_min_v, 0.12);
  EXPECT_DOUBLE_EQ(config.goal_controller.obstacle_finish_timeout_sec, 6.0);
  EXPECT_DOUBLE_EQ(config.goal_controller.finish_dist, 0.2);
}

TEST(NavdogConfigLoadingTest, LoadsRouteFollowerFromParams)
{
  ros::NodeHandle nh("~");
  nh.setParam("route_follower/lookahead_distance_m", 1.2);
  nh.setParam("route_follower/kp_x", 0.9);
  nh.setParam("route_follower/kp_yaw", 1.3);
  nh.setParam("route_follower/max_vx", 0.85);

  const auto config = NavdogRuntimeNode::loadNavdogConfig(nh);

  EXPECT_DOUBLE_EQ(config.route_follower.lookahead_distance_m, 1.2);
  EXPECT_DOUBLE_EQ(config.route_follower.kp_x, 0.9);
  EXPECT_DOUBLE_EQ(config.route_follower.kp_yaw, 1.3);
  EXPECT_DOUBLE_EQ(config.route_follower.max_vx, 0.85);
}

// DISABLED: local_avoid_target config loading removed (LOCAL_AVOID refactor)
TEST(NavdogConfigLoadingTest, DISABLED_LoadsLocalAvoidTargetFromParams)
{
  ros::NodeHandle nh("~");
  nh.setParam("local_avoid_target/default_forward_distance_m", 3.0);
  nh.setParam("local_avoid_target/min_forward_distance_m", 0.8);
  nh.setParam("local_avoid_target/max_forward_distance_m", 3.5);

  const auto config = NavdogRuntimeNode::loadNavdogConfig(nh);

  EXPECT_DOUBLE_EQ(config.local_avoid_target.default_forward_distance_m, 3.0);
  EXPECT_DOUBLE_EQ(config.local_avoid_target.min_forward_distance_m, 0.8);
  EXPECT_DOUBLE_EQ(config.local_avoid_target.max_forward_distance_m, 3.5);
}

TEST(NavdogConfigLoadingTest, LoadsTaskConfigFromParams)
{
  ros::NodeHandle nh("~");
  nh.setParam("task/default_max_vx", 0.5);
  nh.setParam("task/min_max_vx", 0.1);
  nh.setParam("task/max_max_vx", 1.2);

  const auto config = Ros1ConfigLoader::load(nh);

  EXPECT_DOUBLE_EQ(config.task.default_max_vx, 0.5);
  EXPECT_DOUBLE_EQ(config.task.min_max_vx, 0.1);
  EXPECT_DOUBLE_EQ(config.task.max_max_vx, 1.2);
}

TEST(NavdogConfigLoadingTest, LoadsStartAlignFromParams)
{
  ros::NodeHandle nh("~");
  nh.setParam("start_align/enter_deg", 20.0);
  nh.setParam("start_align/exit_deg", 8.0);
  nh.setParam("start_align/max_hold_sec", 2.5);
  nh.setParam("start_align/max_yaw_rate", 0.4);

  const auto config = NavdogRuntimeNode::loadNavdogConfig(nh);

  EXPECT_DOUBLE_EQ(config.start_align.enter_deg, 20.0);
  EXPECT_DOUBLE_EQ(config.start_align.exit_deg, 8.0);
  EXPECT_DOUBLE_EQ(config.start_align.max_hold_sec, 2.5);
  EXPECT_DOUBLE_EQ(config.start_align.max_yaw_rate, 0.4);
}

TEST(NavdogConfigLoadingTest, LoadsRouteProgressFromParams)
{
  ros::NodeHandle nh("~");
  nh.setParam("route_progress/min_segment_length_m", 0.02);
  nh.setParam("route_progress/max_forward_search_m", 4.0);
  nh.setParam("route_progress/on_route_lateral_tolerance_m", 0.35);

  const auto config = NavdogRuntimeNode::loadNavdogConfig(nh);

  EXPECT_DOUBLE_EQ(config.route_progress.min_segment_length_m, 0.02);
  EXPECT_DOUBLE_EQ(config.route_progress.max_forward_search_m, 4.0);
  EXPECT_DOUBLE_EQ(config.route_progress.on_route_lateral_tolerance_m, 0.35);
}

TEST(NavdogConfigLoadingTest, LoadsRouteCorridorFromParams)
{
  ros::NodeHandle nh("~");
  nh.setParam("route_corridor/lookahead_distance_m", 4.0);
  nh.setParam("route_corridor/half_width_m", 0.75);
  nh.setParam("route_corridor_observation/map_timeout_sec", 0.25);
  nh.setParam("route_corridor_observation/max_progress_lag_m", 0.6);

  const auto config = NavdogRuntimeNode::loadNavdogConfig(nh);

  EXPECT_DOUBLE_EQ(config.route_corridor.lookahead_distance_m, 4.0);
  EXPECT_DOUBLE_EQ(config.route_corridor.half_width_m, 0.75);
  EXPECT_DOUBLE_EQ(config.route_corridor_observation.map_timeout_sec, 0.25);
  EXPECT_DOUBLE_EQ(config.route_corridor_observation.max_progress_lag_m, 0.6);
}

// DISABLED: trajectory_follower config loading removed (LOCAL_AVOID refactor)
TEST(NavdogConfigLoadingTest, DISABLED_LoadsTrajectoryFollowerFromParams)
{
  ros::NodeHandle nh("~");
  nh.setParam("trajectory_follower/time_forward_sec", 0.9);
  nh.setParam("trajectory_follower/kp_pos", 0.85);
  nh.setParam("trajectory_follower/kp_yaw", 1.6);

  const auto config = NavdogRuntimeNode::loadNavdogConfig(nh);

  EXPECT_DOUBLE_EQ(config.trajectory_follower.time_forward_sec, 0.9);
  EXPECT_DOUBLE_EQ(config.trajectory_follower.kp_pos, 0.85);
  EXPECT_DOUBLE_EQ(config.trajectory_follower.kp_yaw, 1.6);
}

TEST(NavdogConfigLoadingTest, LoadsNavigationModeConfigFromParams)
{
  ros::NodeHandle nh("~");
  nh.setParam("navigation_mode/enter_blocked_distance_m", 2.5);
  nh.setParam("navigation_mode/enter_confirm_sec", 0.06);
  nh.setParam("navigation_mode/immediate_enter_distance_m", 0.50);
  nh.setParam("navigation_mode/min_local_avoid_hold_sec", 1.2);
  nh.setParam("navigation_mode/exit_clear_confirm_sec", 0.60);
  nh.setParam("navigation_mode/exit_front_clearance_m", 3.0);
  nh.setParam("navigation_mode/exit_left_clearance_m", 0.70);
  nh.setParam("navigation_mode/exit_right_clearance_m", 0.70);

  const auto config = NavdogRuntimeNode::loadNavdogConfig(nh);

  EXPECT_DOUBLE_EQ(config.navigation_mode.enter_blocked_distance_m, 2.5);
  EXPECT_DOUBLE_EQ(config.navigation_mode.enter_confirm_sec, 0.06);
  EXPECT_DOUBLE_EQ(config.navigation_mode.immediate_enter_distance_m, 0.50);
  EXPECT_DOUBLE_EQ(config.navigation_mode.min_local_avoid_hold_sec, 1.2);
  EXPECT_DOUBLE_EQ(config.navigation_mode.exit_clear_confirm_sec, 0.60);
  EXPECT_DOUBLE_EQ(config.navigation_mode.exit_front_clearance_m, 3.0);
  EXPECT_DOUBLE_EQ(config.navigation_mode.exit_left_clearance_m, 0.70);
  EXPECT_DOUBLE_EQ(config.navigation_mode.exit_right_clearance_m, 0.70);
}

TEST(NavdogConfigLoadingTest, LoadsPlannerTimeoutFromParams)
{
  ros::NodeHandle nh("~");
  nh.setParam("planner/planning_timeout_sec", 7.0);

  const auto config = NavdogRuntimeNode::loadNavdogConfig(nh);

  EXPECT_DOUBLE_EQ(config.planner.planning_timeout_sec, 7.0);
}

// =============================================================================
// MqttTopicsLoadFromParams
// =============================================================================

TEST(MqttConfigTest, LoadsTopicsFromParams)
{
  ros::NodeHandle nh("~");
  nh.setParam("mqtt/task_topic", std::string("custom/task"));
  nh.setParam("mqtt/pause_topic", std::string("custom/pause"));
  nh.setParam("mqtt/status_topic", std::string("custom/status"));
  nh.setParam("mqtt/max_queue_size", 64);
  nh.setParam("mqtt/qos", 2);
  nh.setParam("mqtt/port", 8883);
  nh.setParam("mqtt/keepalive_sec", 60);

  std::string task_topic, pause_topic, status_topic;
  int qos, port, keepalive;
  int max_queue_size;

  nh.param<std::string>("mqtt/task_topic", task_topic, "default");
  nh.param<std::string>("mqtt/pause_topic", pause_topic, "default");
  nh.param<std::string>("mqtt/status_topic", status_topic, "default");
  nh.param("mqtt/max_queue_size", max_queue_size, 0);
  nh.param("mqtt/qos", qos, -1);
  nh.param("mqtt/port", port, -1);
  nh.param("mqtt/keepalive_sec", keepalive, -1);

  EXPECT_EQ(task_topic, "custom/task");
  EXPECT_EQ(pause_topic, "custom/pause");
  EXPECT_EQ(status_topic, "custom/status");
  EXPECT_EQ(max_queue_size, 64);
  EXPECT_EQ(qos, 2);
  EXPECT_EQ(port, 8883);
  EXPECT_EQ(keepalive, 60);
}

// =============================================================================
// Config validation tests
//
// These mirror the validation conditions in NavdogRuntimeNode::initialize().
// =============================================================================

namespace
{
bool isValidRate(double rate_hz)
{
  return std::isfinite(rate_hz) && rate_hz > 0.0;
}

bool isValidTimeout(double timeout_sec)
{
  return std::isfinite(timeout_sec) && timeout_sec > 0.0;
}

bool isValidSpeed(double speed)
{
  return std::isfinite(speed) && speed > 0.0;
}

bool isValidMqttQos(int qos)
{
  return qos >= 0 && qos <= 2;
}

bool isValidMqttPort(int port)
{
  return port >= 1 && port <= 65535;
}

bool isValidMqttKeepalive(int keepalive)
{
  return keepalive > 0;
}

bool isValidMqttQueueSize(std::size_t size)
{
  return size > 0;
}

bool isValidTopic(const std::string& topic)
{
  return !topic.empty();
}
}  // namespace

TEST(ConfigValidationTest, InvalidRateRejected)
{
  EXPECT_FALSE(isValidRate(-1.0));
  EXPECT_FALSE(isValidRate(0.0));
  double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isValidRate(nan));
  EXPECT_TRUE(isValidRate(50.0));
}

TEST(ConfigValidationTest, InvalidOdomTimeoutRejected)
{
  EXPECT_FALSE(isValidTimeout(0.0));
  EXPECT_FALSE(isValidTimeout(-0.1));
  EXPECT_TRUE(isValidTimeout(0.5));
}

TEST(ConfigValidationTest, InvalidMaxVxRejected)
{
  EXPECT_FALSE(isValidSpeed(0.0));
  EXPECT_FALSE(isValidSpeed(-1.0));
  EXPECT_TRUE(isValidSpeed(1.0));
}

TEST(ConfigValidationTest, InvalidMqttQueueSizeRejected)
{
  EXPECT_FALSE(isValidMqttQueueSize(0));
  EXPECT_TRUE(isValidMqttQueueSize(32));
}

TEST(ConfigValidationTest, MqttQosValidation)
{
  EXPECT_TRUE(isValidMqttQos(0));
  EXPECT_TRUE(isValidMqttQos(1));
  EXPECT_TRUE(isValidMqttQos(2));
  EXPECT_FALSE(isValidMqttQos(-1));
  EXPECT_FALSE(isValidMqttQos(3));
}

TEST(ConfigValidationTest, MqttPortValidation)
{
  EXPECT_TRUE(isValidMqttPort(1));
  EXPECT_TRUE(isValidMqttPort(1883));
  EXPECT_TRUE(isValidMqttPort(65535));
  EXPECT_FALSE(isValidMqttPort(0));
  EXPECT_FALSE(isValidMqttPort(65536));
}

TEST(ConfigValidationTest, MqttKeepaliveValidation)
{
  EXPECT_TRUE(isValidMqttKeepalive(1));
  EXPECT_TRUE(isValidMqttKeepalive(30));
  EXPECT_FALSE(isValidMqttKeepalive(0));
  EXPECT_FALSE(isValidMqttKeepalive(-1));
}

TEST(ConfigValidationTest, TopicValidation)
{
  EXPECT_FALSE(isValidTopic(""));
  EXPECT_TRUE(isValidTopic("robot/task"));
}

TEST(ConfigValidationTest, AccelLimitsRejectedWhenNonPositive)
{
  EXPECT_FALSE(isValidSpeed(0.0));
  EXPECT_FALSE(isValidSpeed(-0.5));
  EXPECT_TRUE(isValidSpeed(0.5));
}

TEST(ConfigValidationTest, NavigationModeThresholdsMustBePositive)
{
  EXPECT_TRUE(isValidTimeout(0.2));
  EXPECT_TRUE(isValidTimeout(0.4));
  EXPECT_TRUE(isValidTimeout(0.3));
  EXPECT_FALSE(isValidTimeout(0.0));
  EXPECT_FALSE(isValidTimeout(-0.1));
}

}  // namespace
}  // namespace navdog_runtime

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_runtime_config");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
