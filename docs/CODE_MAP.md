# CODE MAP

`navdog_task` owns task data, session sequence and legal task transitions. `navdog_protocol` translates Mosquitto/JSON into `NavigationEvent`. `navdog_core` is the pure-C++ coordinator: route progress, start alignment, Route/SCAN mode and final Route safety gate. `navdog_scan_adapter` converts GridMap observations to core structures. `navdog_runtime` is the ROS composition root. `cmd_vel_owner_mux` is the sole `/cmd_vel` publisher.

Native SCAN execution lives in `plan_manage_dmq`: `SCANReplanFSM` receives the remaining reference path, `SCANPlannerManager` creates/replans B-splines, and `closed_loop_controller` publishes only `/navdog/scan_cmd`.

Important entries: `TaskManager::handleEvent`, `MqttBridge::onMessage`, `NavigationCoordinator::update`, `NavigationModeManager::update`, `NavdogRuntimeNode::controlCallback`, and `cmd_vel_owner_mux::timerCallback`.
