#include "navdog_runtime/ros1_config_loader.hpp"

namespace navdog_runtime
{

ApplicationConfig Ros1ConfigLoader::load(ros::NodeHandle& nh)
{
  ApplicationConfig app{};
  auto& c = app.core;
#define LOAD(name, field) nh.param(name, field, field)
  LOAD("control_rate_hz", c.runtime.control_rate_hz);
  LOAD("status_rate_hz", c.runtime.status_rate_hz);
  LOAD("task/default_max_vx", app.task.default_max_vx);
  LOAD("task/min_max_vx", app.task.min_max_vx);
  LOAD("task/max_max_vx", app.task.max_max_vx);
  LOAD("start_align/enter_deg", c.start_align.enter_deg);
  LOAD("start_align/exit_deg", c.start_align.exit_deg);
  LOAD("start_align/max_hold_sec", c.start_align.max_hold_sec);
  LOAD("start_align/kp_yaw", c.start_align.kp_yaw);
  LOAD("start_align/max_yaw_rate", c.start_align.max_yaw_rate);
  LOAD("start_align/target_min_dist_m", c.start_align.target_min_dist_m);
  LOAD("route_progress/min_segment_length_m", c.route_progress.min_segment_length_m);
  LOAD("route_progress/max_forward_search_m", c.route_progress.max_forward_search_m);
  LOAD("route_progress/on_route_lateral_tolerance_m", c.route_progress.on_route_lateral_tolerance_m);
  LOAD("route_corridor/lookahead_distance_m", c.route_corridor.lookahead_distance_m);
  LOAD("route_corridor_observation/map_timeout_sec", c.route_corridor_observation.map_timeout_sec);
  LOAD("route_corridor_observation/max_progress_lag_m", c.route_corridor_observation.max_progress_lag_m);
  LOAD("planner/planning_timeout_sec", c.planner.planning_timeout_sec);
  LOAD("safety/slow_down_front", c.safety.slow_down_front);
  LOAD("safety/emergency_stop", c.safety.emergency_stop);
  LOAD("safety/odom_timeout_sec", c.safety.odom_timeout_sec);
  LOAD("safety/obstacle_timeout_sec", c.safety.obstacle_timeout_sec);
  LOAD("safety/planner_cmd_timeout_sec", c.safety.planner_cmd_timeout_sec);
  LOAD("safety/future_tolerance_sec", c.safety.future_tolerance_sec);
  LOAD("limits/max_vx", c.limits.max_vx);
  LOAD("limits/max_vy", c.limits.max_vy);
  LOAD("limits/max_yaw_rate", c.limits.max_yaw_rate);
  LOAD("limits/max_accel_x", c.limits.max_accel_x);
  LOAD("limits/max_accel_y", c.limits.max_accel_y);
  LOAD("limits/max_accel_yaw", c.limits.max_accel_yaw);
  LOAD("route_follower/lookahead_distance_m", c.route_follower.lookahead_distance_m);
  LOAD("route_follower/kp_x", c.route_follower.kp_x);
  LOAD("route_follower/kp_y", c.route_follower.kp_y);
  LOAD("route_follower/kp_yaw", c.route_follower.kp_yaw);
  LOAD("route_follower/heading_turn_only_threshold_rad", c.route_follower.heading_turn_only_threshold_rad);
  LOAD("route_follower/max_vx", c.route_follower.max_vx);
  LOAD("trajectory_follower/time_forward_sec", c.trajectory_follower.time_forward_sec);
  LOAD("trajectory_follower/kp_pos", c.trajectory_follower.kp_pos);
  LOAD("trajectory_follower/kp_yaw", c.trajectory_follower.kp_yaw);
  LOAD("local_avoid_target/default_forward_distance_m", c.local_avoid_target.default_forward_distance_m);
  LOAD("local_avoid_target/min_forward_distance_m", c.local_avoid_target.min_forward_distance_m);
  LOAD("local_avoid_target/max_forward_distance_m", c.local_avoid_target.max_forward_distance_m);
  LOAD("local_avoid_target/route_yaw_tolerance_rad", c.local_avoid_target.route_yaw_tolerance_rad);
  LOAD("goal_controller/near_goal_switch_dist", c.goal_controller.near_goal_switch_dist);
  LOAD("goal_controller/near_goal_kp_v", c.goal_controller.near_goal_kp_v);
  LOAD("goal_controller/near_goal_min_v", c.goal_controller.near_goal_min_v);
  LOAD("goal_controller/near_goal_max_v", c.goal_controller.near_goal_max_v);
  LOAD("goal_controller/near_goal_turn_only_deg", c.goal_controller.near_goal_turn_only_deg);
  LOAD("goal_controller/near_goal_kp_w", c.goal_controller.near_goal_kp_w);
  LOAD("goal_controller/near_goal_max_w", c.goal_controller.near_goal_max_w);
  LOAD("goal_controller/obstacle_finish_timeout_sec", c.goal_controller.obstacle_finish_timeout_sec);
  LOAD("goal_controller/finish_dist", c.goal_controller.finish_dist);
  LOAD("goal_controller/finish_yaw_tolerance_deg", c.goal_controller.finish_yaw_tolerance_deg);
  LOAD("planner_trigger/replan_retry_interval_sec", c.planner_trigger.replan_retry_interval_sec);
  LOAD("planner_trigger/trajectory_expiry_margin_sec", c.planner_trigger.trajectory_expiry_margin_sec);
  LOAD("planner_trigger/min_remaining_duration_sec", c.planner_trigger.min_remaining_duration_sec);
  LOAD("planner_trigger/trajectory_source_max_age_sec", c.planner_trigger.trajectory_source_max_age_sec);
  LOAD("planner_trigger/trajectory_future_tolerance_sec", c.planner_trigger.trajectory_future_tolerance_sec);
  LOAD("planner_trigger/target_change_threshold_m", c.planner_trigger.target_change_threshold_m);

  app.scan_adapter.planner_trigger = c.planner_trigger;
  app.runtime_io.control_rate_hz = c.runtime.control_rate_hz;
  app.runtime_io.status_rate_hz = c.runtime.status_rate_hz;
  LOAD("odom_topic", app.runtime_io.odom_topic);
  LOAD("final_cmd_topic", app.runtime_io.final_cmd_topic);
  LOAD("odom_twist_in_world_frame", app.runtime_io.odom_twist_in_world_frame);
  LOAD("final_output/command_timeout_sec", app.final_output.command_timeout_sec);
  LOAD("final_output/publish_rate_hz", app.final_output.publish_rate_hz);

  LOAD("mqtt/enabled", app.mqtt.enabled);
  LOAD("mqtt/host", app.mqtt.host);
  LOAD("mqtt/port", app.mqtt.port);
  LOAD("mqtt/keepalive_sec", app.mqtt.keepalive_sec);
  LOAD("mqtt/client_id", app.mqtt.client_id);
  LOAD("mqtt/qos", app.mqtt.qos);
  LOAD("mqtt/task_topic", app.mqtt.task_topic);
  LOAD("mqtt/pause_topic", app.mqtt.pause_topic);
  LOAD("mqtt/status_topic", app.mqtt.status_topic);
  int queue_size = static_cast<int>(app.mqtt.max_queue_size);
  LOAD("mqtt/max_queue_size", queue_size);
  if (queue_size > 0) app.mqtt.max_queue_size = static_cast<std::size_t>(queue_size);
  LOAD("default_route_z", app.mqtt.default_route_z);
  LOAD("default_max_vx", app.mqtt.default_max_vx);
#undef LOAD

  c.goal_controller.near_goal_turn_only_rad =
      c.goal_controller.near_goal_turn_only_deg * 3.14159265358979323846 / 180.0;
  c.goal_controller.finish_yaw_tolerance_rad =
      c.goal_controller.finish_yaw_tolerance_deg * 3.14159265358979323846 / 180.0;
  return app;
}

}  // namespace navdog_runtime
