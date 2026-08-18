# NAVIGATION WORKFLOW

`MQTT task → MqttBridge → NavigationEvent → Runtime → NavigationCoordinator → PLANNING → START_ALIGN → TRACKING/ROUTE_FOLLOW → LOCAL_AVOID → ROUTE_FOLLOW → GoalController → SUCCEEDED`.

Each 50 Hz Runtime cycle consumes queued events, snapshots odometry, evaluates SCAN-derived obstacle/corridor inputs, calls `NavigationCoordinator::update`, reacts to its structured mode transition (reset then delayed remaining-path publication), processes planner actions, and publishes Route command/state/mode/status. Runtime never re-evaluates Route/SCAN thresholds.

`LOCAL_AVOID` feeds Native SCAN `/native_scan/raw_cmd` into Core as
`planner_cmd`. The same final `SafetySupervisor` used by Route limits this
actual SCAN command, Runtime publishes it on `/navdog/scan_cmd`, and the mux
selects it only after ownership changes and a fresh takeover command arrives.
For planner commands the supervisor preserves the collision-checked
`vx/vy/yaw` direction while enforcing freshness, map validity, task/device
limits and acceleration bounds. Runtime's coarse sector summary does not
repeat Native SCAN's trajectory collision decision. The Native controller freezes B-spline time
when odometry falls outside its trajectory-anchor tolerance; a larger tracking
loss invalidates the old trajectory and recovers through the existing
emergency-stop/fresh-plan path.
