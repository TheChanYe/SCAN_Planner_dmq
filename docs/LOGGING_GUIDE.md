# LOGGING GUIDE

ROS launch sets `ROSCONSOLE_FORMAT` to `[${severity}] [${time}] [${node}] ${message}`. MQTT uses `[YYYY-MM-DD HH:MM:SS.mmm][LEVEL][MQTT]` on stderr with atomic lines.

Runtime is the only source of `NAV_STATE` and `NAV_MODE`; the mux records only `CMD_OWNER`. MQTT owns connection, acceptance and rejection events. Native SCAN owns FSM/planning/collision events. Core has no direct I/O.

INFO is transition/event based. Repeated stale, invalid, waiting or map-not-ready conditions must use ROS throttle macros. Logs carry sequence and enum names where applicable; rejected MQTT payloads record only topic, byte count and error category.
