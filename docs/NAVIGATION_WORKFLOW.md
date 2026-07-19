# NAVIGATION WORKFLOW

`MQTT task → MqttBridge → NavigationEvent → Runtime → NavigationCoordinator → PLANNING → START_ALIGN → TRACKING/ROUTE_FOLLOW → LOCAL_AVOID → ROUTE_FOLLOW → GoalController → SUCCEEDED`.

Each 50 Hz Runtime cycle consumes queued events, snapshots odometry, evaluates SCAN-derived obstacle/corridor inputs, calls `NavigationCoordinator::update`, reacts to its structured mode transition (reset then delayed remaining-path publication), processes planner actions, and publishes Route command/state/mode/status. Runtime never re-evaluates Route/SCAN thresholds.

`LOCAL_AVOID` makes Core return zero Route velocity. Native SCAN then provides `/navdog/scan_cmd`; the mux selects it only after ownership changes and a fresh SCAN command arrives.
