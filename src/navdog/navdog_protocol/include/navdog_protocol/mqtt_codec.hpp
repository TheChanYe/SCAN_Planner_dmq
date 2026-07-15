#pragma once

#include <navdog_task/task_types.hpp>

#include <cstdint>
#include <string>

namespace navdog_protocol
{

class MqttCodec
{
public:
  static bool parseTaskMessage(const std::string& payload,
      double default_route_z, double default_max_vx, std::uint64_t sequence,
      navdog_task::NavigationEvent& event, bool& charging_reserve);
  static bool parsePauseMessage(const std::string& payload,
      navdog_task::NavigationEvent& event);
  static std::string encodeStatus(int status, int error);
};

}  // namespace navdog_protocol
