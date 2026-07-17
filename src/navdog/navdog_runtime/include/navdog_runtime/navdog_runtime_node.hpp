#pragma once

#include "navdog_runtime/application_config.hpp"

#include <navdog_core/navigation_coordinator.hpp>
#include <navdog_protocol/mqtt_bridge.hpp>
#include <navdog_scan_adapter/occupancy_query_adapter.hpp>
#include <navdog_scan_adapter/scan_grid_map_query.hpp>
#include <navdog_scan_adapter/scan_local_planner_adapter.hpp>
#include <navdog_scan_adapter/scan_obstacle_summary_evaluator_3d.hpp>
#include <navdog_scan_adapter/scan_route_corridor_evaluator_3d.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <plan_manage_dmq/planner_manager.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include <memory>
#include <mutex>

namespace navdog_runtime
{

class NavdogRuntimeNode
{
public:
  NavdogRuntimeNode(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~NavdogRuntimeNode();
  bool initialize();

  static void statusForOutput(const navdog::CoreOutput& output,
      bool protocol_error, int& status, int& error);
  static geometry_msgs::Twist toTwist(const navdog::VelocityCommand& command);
  static navdog::PlannerFeedback feedbackForAction(
      const navdog::PlannerAction& action, double now_sec);
  static navdog::NavdogConfig loadNavdogConfig(ros::NodeHandle& nh);

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& message);
  void controlCallback(const ros::TimerEvent&);
  void processEvents();
  void processPlannerAction(const navdog::PlannerAction& action, double now_sec);
  void publishRoute();
  void publishLocalTrajectory(const navdog::CoreOutput& output);
  void publishOutput(const navdog::CoreOutput& output, double now_sec);
  void publishMqttStatus(const navdog::CoreOutput& output);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ApplicationConfig application_config_{};
  std::unique_ptr<navdog::NavigationCoordinator> coordinator_;
  std::shared_ptr<scan_planner::SCANPlannerManager> planner_manager_;
  std::shared_ptr<navdog_scan_adapter::ScanGridMapQuery> grid_query_;
  std::shared_ptr<navdog_scan_adapter::OccupancyQueryAdapter> occupancy_query_;
  std::unique_ptr<navdog_scan_adapter::ScanLocalPlannerAdapter> local_planner_adapter_;
  std::unique_ptr<navdog_scan_adapter::ScanRouteCorridorEvaluator3D> corridor_evaluator_;
  std::unique_ptr<navdog_scan_adapter::ScanObstacleSummaryEvaluator3D> obstacle_evaluator_;
  std::unique_ptr<navdog_protocol::MqttBridge> mqtt_;

  ros::Subscriber odom_subscriber_;
  ros::Publisher route_publisher_;
  ros::Publisher local_trajectory_publisher_;
  ros::Publisher state_publisher_;
  ros::Publisher mode_publisher_;
  ros::Publisher final_cmd_publisher_;
  ros::Timer control_timer_;

  mutable std::mutex odom_mutex_;
  navdog::RobotState robot_{};
  navdog::RouteProgress last_route_progress_{};
  navdog::PlannerFeedback pending_planner_feedback_{};
  ros::Time last_status_publish_{};
  std::uint64_t published_local_plan_sequence_{0};
  bool local_trajectory_visible_{false};
};

}  // namespace navdog_runtime
