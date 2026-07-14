#pragma once

#include "navdog_runtime/mqtt_bridge.hpp"

#include <navdog_core/navigation_coordinator.hpp>
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
#include <std_msgs/Empty.h>
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

  static void statusForOutput(
      const navdog::CoreOutput& output,
      bool protocol_error,
      int& status,
      int& error);
  static geometry_msgs::Twist toTwist(const navdog::VelocityCommand& command);
  static navdog::PlannerFeedback feedbackForAction(
      const navdog::PlannerAction& action, double now_sec);
  static navdog::NavdogConfig loadNavdogConfig(ros::NodeHandle& nh);

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& message);
  void controlCallback(const ros::TimerEvent&);
  void publisherCheckCallback(const ros::TimerEvent&);
  void processEvents();
  void processPlannerAction(const navdog::PlannerAction& action, double now_sec);
  void publishDebug(const navdog::CoreOutput& output, double now_sec,
                    const navdog::VelocityCommand& effective_cmd);
  void publishRoute(const navdog::NavigationTask& task);
  bool publishNativeScanTakeoverPath(
      const navdog::NavigationTask& task,
      const navdog::RobotState& robot,
      const navdog::RouteProgress& progress);
  void setControlOwner(std::uint8_t owner);
  void resetNativeScanTakeover(bool publish_reset);
  void releaseNativeScanToRoute(const navdog::NavigationTask& task,
                                const navdog::RobotState& robot,
                                const navdog::CoreOutput& output);
  void updateControlOwner(const navdog::CoreOutput& output,
                          const navdog::RobotState& robot);
  void publishMqttStatus(const navdog::CoreOutput& output);
  bool hasUniqueCmdVelPublisher();
  bool hasUniqueCmdVelPublisherCached() const;
  void publishZeroFiveTimes();

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  navdog::NavigationCoordinator coordinator_;
  std::shared_ptr<scan_planner::SCANPlannerManager> planner_manager_;
  std::shared_ptr<navdog_scan_adapter::ScanGridMapQuery> grid_query_;
  std::shared_ptr<navdog_scan_adapter::OccupancyQueryAdapter> occupancy_query_;
  std::unique_ptr<navdog_scan_adapter::ScanLocalPlannerAdapter> local_planner_adapter_;
  std::unique_ptr<navdog_scan_adapter::ScanRouteCorridorEvaluator3D> corridor_evaluator_;
  std::unique_ptr<navdog_scan_adapter::ScanObstacleSummaryEvaluator3D> obstacle_evaluator_;
  std::unique_ptr<MqttBridge> mqtt_;

  ros::Subscriber odom_subscriber_;
  ros::Publisher cmd_vel_publisher_;
  ros::Publisher route_publisher_;
  ros::Publisher state_publisher_;
  ros::Publisher mode_publisher_;
  ros::Publisher final_cmd_publisher_;
  ros::Publisher control_owner_publisher_;
  ros::Publisher native_scan_path_publisher_;
  ros::Publisher native_scan_reset_publisher_;
  ros::Timer control_timer_;
  ros::Timer publisher_check_timer_;

  mutable std::mutex odom_mutex_;
  navdog::RobotState robot_{};
  navdog::RouteProgress last_route_progress_{};
  navdog::PlannerFeedback pending_planner_feedback_{};
  navdog::CoreOutput last_output_{};
  ros::Time last_status_publish_{};
  navdog::NavState last_logged_state_{navdog::NavState::IDLE};
  navdog::NavigationMode last_logged_mode_{navdog::NavigationMode::NONE};

  navdog::VelocityCommand effective_command_{};
  bool cmd_vel_conflict_{false};
  bool cmd_vel_conflict_latched_{false};

  bool native_scan_takeover_{true};
  bool scan_takeover_active_{false};
  std::uint64_t scan_takeover_task_sequence_{0};
  std::uint8_t control_owner_{0};

  std::string odom_topic_;
  std::string cmd_vel_topic_;
  bool odom_twist_in_world_frame_{true};
  double control_rate_hz_{50.0};
  double status_rate_hz_{10.0};
  bool initialized_{false};
};

}  // namespace navdog_runtime
