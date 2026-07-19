# ARCHITECTURE BOUNDARIES

Dependency direction is `navdog_task ← navdog_core ← navdog_runtime`. Protocol direction is `Mosquitto/JSON → navdog_protocol → navdog_runtime → NavigationEvent → navdog_core`. SCAN direction is `plan_env/scan_planner_dmq → navdog_scan_adapter → navdog_runtime → CoreInput → navdog_core`.

Task and Core are standard C++ only. Protocol may use Mosquitto/JsonCpp but no ROS or navigation decision. The adapter may use ROS/GridMap but only produces observations. Runtime is the composition root; it may not duplicate `NavigationModeManager` conditions. SCAN plans/tracks only and never parses MQTT or changes Navdog state.

`/navdog/route_cmd` is Route-only, `/navdog/scan_cmd` is SCAN-only, and `cmd_vel_owner_mux` is the only `/cmd_vel` publisher.
