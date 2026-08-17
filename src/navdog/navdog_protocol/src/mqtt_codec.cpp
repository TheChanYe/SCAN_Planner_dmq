#include "navdog_protocol/mqtt_codec.hpp"

#include <json/json.h>
#include <cmath>
#include <sstream>

/*
  * 该文件实现了MqttCodec类，用于解析和编码MQTT消息。
  * 它提供了从JSON格式的MQTT消息中提取导航任务和暂停/恢复事件的功能，
  * 并将状态和错误信息编码为JSON格式的字符串。
  **/
namespace navdog_protocol
{
namespace
{
/**
 * @brief finiteJson
 * 检查JSON值是否为数值类型且为有限数，并转换为double输出。
 * 输入：value - 待检查的JSON节点。输出：output - 转换后的数值（仅当返回true时有效）。
 * 返回：非数值类型或NaN/Inf均返回false。
 */
bool finiteJson(const Json::Value& value, double& output)
{
  if (!value.isNumeric()) return false;
  output = value.asDouble();
  return std::isfinite(output);
}
}
/**
 * @brief parseTaskMessage
 * 解析任务下发JSON报文为NavigationEvent。
 * 步骤：
 *   1.解析JSON失败或非object或ctrl非int直接返回false；
 *   2.ctrl==0或ctrl==3表示取消任务（3为充电保留语义），直接构造CANCEL_TASK事件返回true；
 *   3.非ctrl==1或sequence为0（未分配内部序号）均视为无效；
 *   4.取navigation_data.points数组，非object/非array/空数组均无效；
 *   5.构造START_TASK事件，填入内部sequence、模式固定为NORMAL_AVOID；
 *   6.解析max_vx（缺失时用default_max_vx），必须为正数；
 *   7.遍历每个路点：x/y必须存在且为有限数，z缺失时用default_route_z，
 *      yaw可选但存在时必须为有限数并标记has_yaw；
 *   8.全部通过则返回true，任一环节失败则返回false。
 * 输入：payload - JSON文本；default_route_z/default_max_vx - 默认值；sequence - 内部序号。
 * 输出：event - 解析结果；charging_reserve - 是否为充电保留。
 */
bool MqttCodec::parseTaskMessage(const std::string& payload,
    double default_route_z, double default_max_vx, std::uint64_t sequence,
    navdog_task::NavigationEvent& event, bool& charging_reserve,
    NavigationMessageMeta* meta)
{
  event = navdog_task::NavigationEvent{};
  charging_reserve = false;
  if (meta) *meta = NavigationMessageMeta{};
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string errors;
  std::istringstream stream(payload);
  /*
  * 解析JSON负载，检查控制字段，提取导航数据和路点信息，并验证最大速度和路点的有效性。
  * 如果解析成功，将结果存储在NavigationEvent对象中，并返回true；否则
  * 返回false。
  */
  if (!Json::parseFromStream(builder, stream, &root, &errors) ||
      !root.isObject() || !root["ctrl"].isInt()) return false;
  const int ctrl = root["ctrl"].asInt();
  /* 处理取消任务和充电保留标志*/
  if (ctrl == 0 || ctrl == 3)
  {
    event.type = navdog_task::NavigationEventType::CANCEL_TASK;
    charging_reserve = ctrl == 3;
    return true;
  }
  if ((ctrl != 1 && ctrl != 2) || sequence == 0) return false;
  const Json::Value& data = root["navigation_data"];
  const Json::Value& points = data["points"];
  // 检查数据是否为对象，points是否为数组且不为空
  // 如果不是，则返回false
  // 如果是，则将数据存储在NavigationEvent对象中
  if (!data.isObject() || !points.isArray() || points.empty()) return false;
  event.type = navdog_task::NavigationEventType::START_TASK;
  event.task.sequence = sequence;
  event.task.mode = ctrl == 2 ? navdog_task::TaskMode::ROUTE_ONLY
                              : navdog_task::TaskMode::NORMAL_AVOID;
  event.task.max_vx = default_max_vx;
  if (meta)
  {
    const Json::Value& id = data["id"];
    if (id.isUInt() && id.asUInt() <= 65535u)
    {
      meta->id = static_cast<std::uint16_t>(id.asUInt());
      meta->has_id = true;
    }
    const Json::Value& map_name = data["map_name"];
    if (map_name.isString())
    {
      meta->map_name = map_name.asString();
      meta->has_map_name = true;
    }
  }
  if (data.isMember("max_vx") && !finiteJson(data["max_vx"], event.task.max_vx))
    return false;
  if (meta && data.isMember("max_vx"))
  {
    meta->max_vx = event.task.max_vx;
    meta->has_max_vx = true;
  }
  if (!(event.task.max_vx > 0.0)) return false; // 最大速度必须大于0
  for (const auto& item : points) // 遍历每个路点，检查其是否为对象，x/y/z/yaw是否为有限数
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
/**
 * @brief parsePauseMessage
 * 解析暂停消息，提取导航事件。
 * 具体实现包括解析JSON负载，检查控制字段，并返回解析是否成功。
 * 
 */
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

bool MqttCodec::parseObstacleMessage(const std::string& payload,
    ExternalObstacleInfo& obstacle)
{
  obstacle = ExternalObstacleInfo{};
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string errors;
  std::istringstream stream(payload);
  if (!Json::parseFromStream(builder, stream, &root, &errors) ||
      !root.isObject() || !root["status"].isInt() || !root["error"].isInt())
    return false;

  const int status = root["status"].asInt();
  const int error = root["error"].asInt();
  if (status < 0 || status > 255 || error < 0 || error > 255)
    return false;

  double distance = std::numeric_limits<double>::infinity();
  if (root.isMember("distance") && !finiteJson(root["distance"], distance))
    return false;

  obstacle.status = static_cast<std::uint8_t>(status);
  obstacle.error = static_cast<std::uint8_t>(error);
  obstacle.distance = distance;
  obstacle.valid = true;
  return true;
}
/**
 * @brief encodeStatus
 * 编码状态消息为JSON字符串。
 * @param status 状态码
 * @param error 错误码
 * @return JSON字符串
 */
std::string MqttCodec::encodeStatus(int status, int error,
    double vx, double vy, double yaw_rate)
{
  Json::Value root;
  root["status"] = status;
  root["error"] = error;
  Json::Value velocity;
  velocity["vx"] = std::isfinite(vx) ? vx : 0.0;
  velocity["vy"] = std::isfinite(vy) ? vy : 0.0;
  velocity["yaw_rate"] = std::isfinite(yaw_rate) ? yaw_rate : 0.0;
  root["velocity"] = velocity;
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  return Json::writeString(writer, root);
}

std::string MqttCodec::encodeVoiceMessage(const std::string& message)
{
  Json::Value root;
  root["message"] = message;
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  return Json::writeString(writer, root);
}

}  // namespace navdog_protocol
