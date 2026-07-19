#!/usr/bin/env bash
# Static dependency guard only. It deliberately does not replace a catkin build.
set -euo pipefail

check_absent() {
  local label="$1" directory="$2" expression="$3"
  if rg -n --glob '*.{hpp,h,cpp,cc,cxx,CMakeLists.txt,package.xml}' "$expression" "$directory" >/dev/null; then
    echo "[FAIL] $label"
    rg -n --glob '*.{hpp,h,cpp,cc,cxx,CMakeLists.txt,package.xml}' "$expression" "$directory"
    return 1
  fi
  echo "[PASS] $label"
}

status=0
check_absent "navdog_task/core has no ROS dependency" src/navdog/navdog_task '(^|[^[:alnum:]_])(ros/|ROS_(INFO|WARN|ERROR)|geometry_msgs|nav_msgs|std_msgs)' || status=1
check_absent "navdog_task has no MQTT or SCAN dependency" src/navdog/navdog_task '(mosquitto|json/json|plan_env|scan_planner)' || status=1
check_absent "navdog_core has no ROS dependency" src/navdog/navdog_core '(^|[^[:alnum:]_])(ros/|ROS_(INFO|WARN|ERROR)|geometry_msgs|nav_msgs|std_msgs)' || status=1
check_absent "navdog_core has no MQTT or SCAN dependency" src/navdog/navdog_core '(mosquitto|json/json|plan_env|scan_planner)' || status=1
check_absent "navdog_protocol has no ROS dependency" src/navdog/navdog_protocol '(#include[[:space:]]*[<\"](ros/|geometry_msgs|nav_msgs|std_msgs|navdog_core/|navdog_runtime/)|ROS_(INFO|WARN|ERROR))' || status=1
check_absent "SCAN packages have no MQTT dependency" src/planner/plan_manage_dmq '(mosquitto|MqttBridge|MqttCodec)' || status=1
check_absent "navdog_scan_adapter has no MQTT dependency" src/navdog/navdog_scan_adapter '(mosquitto|MqttBridge|MqttCodec)' || status=1
exit "$status"
