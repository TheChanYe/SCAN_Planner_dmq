#include "navdog_runtime/ros1_config_loader.hpp"

namespace navdog_runtime
{

// load：从ROS参数服务器按层级路径加载navdog_runtime全部配置项（使用LOAD宏封装
// nh.param(name, field, field)，未设置时保留传入field的默认值）。
// 步骤：1.依次加载运行频率、任务速度范围、起点对齐、路线进度、路径走廊、
//      安全、导航模式切换、速度/加速度限幅、路线跟随、目标控制器等各子模块参数；
//   2.加载runtime IO/最终输出相关参数；3.加载MQTT桥接层参数（队列长度先用int
//      中转再转换为size_t，避免param宏对size_t支持不佳）；
//   4.将near_goal_turn_only_deg/finish_yaw_tolerance_deg两个度数配置预先转换为
//      弧度缓存字段，避免下游每次使用时重复转换。
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
  LOAD("route_corridor/half_width_m", c.route_corridor.half_width_m);
  LOAD("route_corridor_observation/map_timeout_sec", c.route_corridor_observation.map_timeout_sec);
  LOAD("route_corridor_observation/max_progress_lag_m", c.route_corridor_observation.max_progress_lag_m);
  LOAD("planner/planning_timeout_sec", c.planner.planning_timeout_sec);
  LOAD("safety/slow_down_front", c.safety.slow_down_front);
  LOAD("safety/emergency_stop", c.safety.emergency_stop);
  LOAD("safety/odom_timeout_sec", c.safety.odom_timeout_sec);
  LOAD("safety/obstacle_timeout_sec", c.safety.obstacle_timeout_sec);
  LOAD("safety/planner_cmd_timeout_sec", c.safety.planner_cmd_timeout_sec);
  LOAD("safety/future_tolerance_sec", c.safety.future_tolerance_sec);
  LOAD("navigation_mode/enter_blocked_distance_m", c.navigation_mode.enter_blocked_distance_m);
  LOAD("navigation_mode/enter_confirm_sec", c.navigation_mode.enter_confirm_sec);
  LOAD("navigation_mode/immediate_enter_distance_m", c.navigation_mode.immediate_enter_distance_m);
  LOAD("navigation_mode/min_local_avoid_hold_sec", c.navigation_mode.min_local_avoid_hold_sec);
  LOAD("navigation_mode/exit_clear_confirm_sec", c.navigation_mode.exit_clear_confirm_sec);
  LOAD("navigation_mode/exit_front_clearance_m", c.navigation_mode.exit_front_clearance_m);
  LOAD("navigation_mode/exit_left_clearance_m", c.navigation_mode.exit_left_clearance_m);
  LOAD("navigation_mode/exit_right_clearance_m", c.navigation_mode.exit_right_clearance_m);
  LOAD("stair_up/enabled", c.stair_up.enabled);
  LOAD("stair_up/lookahead_distance_m", c.stair_up.lookahead_distance_m);
  LOAD("stair_up/sample_step_m", c.stair_up.sample_step_m);
  LOAD("stair_up/trigger_rise_m", c.stair_up.trigger_rise_m);
  LOAD("stair_up/flat_tolerance_m", c.stair_up.flat_tolerance_m);
  LOAD("stair_up/exit_progress_margin_m", c.stair_up.exit_progress_margin_m);
  LOAD("stair_up/exit_confirm_sec", c.stair_up.exit_confirm_sec);
  LOAD("limits/max_vx", c.limits.max_vx);
  LOAD("limits/max_vy", c.limits.max_vy);
  LOAD("limits/max_yaw_rate", c.limits.max_yaw_rate);
  LOAD("limits/max_accel_x", c.limits.max_accel_x);
  LOAD("limits/max_accel_y", c.limits.max_accel_y);
  LOAD("limits/max_accel_yaw", c.limits.max_accel_yaw);
  LOAD("route_follower/lookahead_distance_m", c.route_follower.lookahead_distance_m);
  LOAD("route_follower/max_lookahead_distance_m", c.route_follower.max_lookahead_distance_m);
  LOAD("route_follower/lookahead_time_sec", c.route_follower.lookahead_time_sec);
  LOAD("route_follower/kp_x", c.route_follower.kp_x);
  LOAD("route_follower/kp_y", c.route_follower.kp_y);
  LOAD("route_follower/kp_yaw", c.route_follower.kp_yaw);
  LOAD("route_follower/heading_slowdown_start_rad", c.route_follower.heading_slowdown_start_rad);
  LOAD("route_follower/heading_turn_only_threshold_rad", c.route_follower.heading_turn_only_threshold_rad);
  LOAD("route_follower/max_vx", c.route_follower.max_vx);
  LOAD("goal_controller/near_goal_switch_dist", c.goal_controller.near_goal_switch_dist);
  LOAD("goal_controller/near_goal_kp_v", c.goal_controller.near_goal_kp_v);
  LOAD("goal_controller/near_goal_min_v", c.goal_controller.near_goal_min_v);
  LOAD("goal_controller/near_goal_max_v", c.goal_controller.near_goal_max_v);
  LOAD("goal_controller/near_goal_turn_only_deg", c.goal_controller.near_goal_turn_only_deg);
  LOAD("goal_controller/near_goal_kp_w", c.goal_controller.near_goal_kp_w);
  LOAD("goal_controller/near_goal_max_w", c.goal_controller.near_goal_max_w);
  LOAD("goal_controller/obstacle_finish_timeout_sec", c.goal_controller.obstacle_finish_timeout_sec);
  LOAD("goal_controller/finish_dist", c.goal_controller.finish_dist);
  LOAD("goal_controller/goal_align_reacquire_dist", c.goal_controller.goal_align_reacquire_dist);
  LOAD("goal_controller/goal_align_timeout_sec", c.goal_controller.goal_align_timeout_sec);
  LOAD("goal_controller/goal_align_min_yaw_rate", c.goal_controller.goal_align_min_yaw_rate);
  LOAD("goal_controller/finish_yaw_tolerance_deg", c.goal_controller.finish_yaw_tolerance_deg);
  app.runtime_io.control_rate_hz = c.runtime.control_rate_hz;
  app.runtime_io.status_rate_hz = c.runtime.status_rate_hz;
  LOAD("odom_topic", app.runtime_io.odom_topic);
  LOAD("final_cmd_topic", app.runtime_io.final_cmd_topic);
  LOAD("final_cmd_feedback_topic", app.runtime_io.final_cmd_feedback_topic);
  LOAD("external_stop_topic", app.runtime_io.external_stop_topic);
  LOAD("max_vx_limit_topic", app.runtime_io.max_vx_limit_topic);
  LOAD("protocol_status_topic", app.runtime_io.protocol_status_topic);
  LOAD("protocol_error_topic", app.runtime_io.protocol_error_topic);
  LOAD("odom_twist_in_world_frame", app.runtime_io.odom_twist_in_world_frame);
  LOAD("publish_mqtt_status", app.runtime_io.publish_mqtt_status);
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
  LOAD("mqtt/obstacle_topic", app.mqtt.obstacle_topic);
  LOAD("mqtt/status_topic", app.mqtt.status_topic);
  LOAD("mqtt/voice_topic", app.mqtt.voice_topic);
  int queue_size = static_cast<int>(app.mqtt.max_queue_size);
  LOAD("mqtt/max_queue_size", queue_size);
  if (queue_size > 0) app.mqtt.max_queue_size = static_cast<std::size_t>(queue_size);
  LOAD("default_route_z", app.mqtt.default_route_z);
  LOAD("default_max_vx", app.mqtt.default_max_vx);
  LOAD("dynamic_obstacle/enabled", app.dynamic_obstacle.enabled);
  LOAD("dynamic_obstacle/stop_distance_m", app.dynamic_obstacle.stop_distance_m);
  LOAD("dynamic_obstacle/hold_sec", app.dynamic_obstacle.hold_sec);
  LOAD("dynamic_obstacle/timeout_sec", app.dynamic_obstacle.timeout_sec);
  LOAD("turn_voice/enabled", app.turn_voice.enabled);
  LOAD("turn_voice/min_yaw_rate", app.turn_voice.min_yaw_rate);
  LOAD("turn_voice/max_linear_speed", app.turn_voice.max_linear_speed);
  LOAD("turn_voice/cooldown_sec", app.turn_voice.cooldown_sec);
  LOAD("turn_voice/message", app.turn_voice.message);
#undef LOAD

  c.goal_controller.near_goal_turn_only_rad =
      c.goal_controller.near_goal_turn_only_deg * 3.14159265358979323846 / 180.0;
  c.goal_controller.finish_yaw_tolerance_rad =
      c.goal_controller.finish_yaw_tolerance_deg * 3.14159265358979323846 / 180.0;
  return app;
}

}  // namespace navdog_runtime
