#include "navdog_protocol/mqtt_codec.hpp"

#include <json/json.h>
#include <cmath>
#include <sstream>

namespace navdog_protocol
{
namespace
{
bool finiteJson(const Json::Value& value, double& output)
{
  if (!value.isNumeric()) return false;
  output = value.asDouble();
  return std::isfinite(output);
}
}

bool MqttCodec::parseTaskMessage(const std::string& payload,
    double default_route_z, double default_max_vx, std::uint64_t sequence,
    navdog_task::NavigationEvent& event, bool& charging_reserve)
{
  event = navdog_task::NavigationEvent{};
  charging_reserve = false;
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string errors;
  std::istringstream stream(payload);
  if (!Json::parseFromStream(builder, stream, &root, &errors) ||
      !root.isObject() || !root["ctrl"].isInt()) return false;
  const int ctrl = root["ctrl"].asInt();
  if (ctrl == 0 || ctrl == 3)
  {
    event.type = navdog_task::NavigationEventType::CANCEL_TASK;
    charging_reserve = ctrl == 3;
    return true;
  }
  if (ctrl != 1 || sequence == 0) return false;
  const Json::Value& data = root["navigation_data"];
  const Json::Value& points = data["points"];
  if (!data.isObject() || !points.isArray() || points.size() < 2) return false;
  event.type = navdog_task::NavigationEventType::START_TASK;
  event.task.sequence = sequence;
  event.task.mode = navdog_task::TaskMode::NORMAL_AVOID;
  event.task.max_vx = default_max_vx;
  if (data.isMember("max_vx") && !finiteJson(data["max_vx"], event.task.max_vx))
    return false;
  if (!(event.task.max_vx > 0.0)) return false;
  for (const auto& item : points)
  {
    navdog_task::RoutePoint point{};
    if (!item.isObject() || !finiteJson(item["x"], point.x) ||
        !finiteJson(item["y"], point.y)) return false;
    point.z = default_route_z;
    if (item.isMember("z") && !finiteJson(item["z"], point.z)) return false;
    if (item.isMember("yaw"))
    {
      if (!finiteJson(item["yaw"], point.yaw)) return false;
      point.has_yaw = true;
    }
    event.task.points.push_back(point);
  }
  return true;
}

bool MqttCodec::parsePauseMessage(const std::string& payload,
    navdog_task::NavigationEvent& event)
{
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string errors;
  std::istringstream stream(payload);
  if (!Json::parseFromStream(builder, stream, &root, &errors) ||
      !root.isObject() || !root["action"].isInt()) return false;
  const int action = root["action"].asInt();
  if (action != 1 && action != 2) return false;
  event = navdog_task::NavigationEvent{};
  event.type = action == 1 ? navdog_task::NavigationEventType::PAUSE
                           : navdog_task::NavigationEventType::RESUME;
  return true;
}

std::string MqttCodec::encodeStatus(int status, int error)
{
  Json::Value root;
  root["status"] = status;
  root["error"] = error;
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  return Json::writeString(writer, root);
}

}  // namespace navdog_protocol
