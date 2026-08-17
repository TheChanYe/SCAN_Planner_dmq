#include <gtest/gtest.h>
#include "navdog_protocol/mqtt_codec.hpp"

TEST(MqttCodec, KeepsTaskControlProtocolCompatible)
{
  navdog_task::NavigationEvent event{};
  bool charging = false;
  ASSERT_TRUE(navdog_protocol::MqttCodec::parseTaskMessage(
      R"({"ctrl":1,"navigation_data":{"max_vx":0.3,"points":[{"x":0,"y":0},{"x":1,"y":0}]}})",
      0.3, 0.4, 7, event, charging));
  EXPECT_EQ(navdog_task::NavigationEventType::START_TASK, event.type);
  EXPECT_EQ(2u, event.task.points.size());
  EXPECT_EQ(navdog_task::TaskMode::NORMAL_AVOID, event.task.mode);
  EXPECT_FALSE(charging);
  ASSERT_TRUE(navdog_protocol::MqttCodec::parseTaskMessage(
      R"({"ctrl":3})", 0.3, 0.4, 8, event, charging));
  EXPECT_TRUE(charging);
}

TEST(MqttCodec, RejectsMalformedAndEncodesStatus)
{
  navdog_task::NavigationEvent event{};
  bool charging = false;
  EXPECT_FALSE(navdog_protocol::MqttCodec::parseTaskMessage(
      "not-json", 0.3, 0.4, 1, event, charging));
  EXPECT_EQ(R"({"error":2,"status":5,"velocity":{"vx":0.1,"vy":-0.2,"yaw_rate":0.3}})",
            navdog_protocol::MqttCodec::encodeStatus(5, 2, 0.1, -0.2, 0.3));
}

TEST(MqttCodec, Ctrl2UsesRouteOnlyAndParsesMeta)
{
  navdog_task::NavigationEvent event{};
  navdog_protocol::NavigationMessageMeta meta{};
  bool charging = false;
  ASSERT_TRUE(navdog_protocol::MqttCodec::parseTaskMessage(
      R"({"ctrl":2,"navigation_data":{"id":123,"map_name":"map1","points":[{"x":0,"y":0}]}})",
      0.3, 0.4, 11, event, charging, &meta));
  EXPECT_EQ(navdog_task::NavigationEventType::START_TASK, event.type);
  EXPECT_EQ(navdog_task::TaskMode::ROUTE_ONLY, event.task.mode);
  EXPECT_TRUE(meta.has_id);
  EXPECT_EQ(123, meta.id);
  EXPECT_TRUE(meta.has_map_name);
  EXPECT_EQ("map1", meta.map_name);
}

TEST(MqttCodec, ParsesExternalObstacleInfo)
{
  navdog_protocol::ExternalObstacleInfo obstacle{};
  ASSERT_TRUE(navdog_protocol::MqttCodec::parseObstacleMessage(
      R"({"distance":0.8,"status":2,"error":0})", obstacle));
  EXPECT_TRUE(obstacle.valid);
  EXPECT_EQ(2, obstacle.status);
  EXPECT_EQ(0, obstacle.error);
  EXPECT_DOUBLE_EQ(0.8, obstacle.distance);
}

TEST(MqttCodec, AcceptsSingleDestinationPoint)
{
  navdog_task::NavigationEvent event{};
  bool charging = false;
  ASSERT_TRUE(navdog_protocol::MqttCodec::parseTaskMessage(
      R"({"ctrl":1,"navigation_data":{"max_vx":0.3,"points":[{"x":1.2,"y":-0.4}]}})",
      0.3, 0.4, 9, event, charging));
  EXPECT_EQ(navdog_task::NavigationEventType::START_TASK, event.type);
  ASSERT_EQ(1u, event.task.points.size());
  EXPECT_DOUBLE_EQ(1.2, event.task.points.front().x);
  EXPECT_DOUBLE_EQ(-0.4, event.task.points.front().y);
  EXPECT_DOUBLE_EQ(0.3, event.task.points.front().z);
}

TEST(MqttCodec, RejectsTaskWithoutDestination)
{
  navdog_task::NavigationEvent event{};
  bool charging = false;
  EXPECT_FALSE(navdog_protocol::MqttCodec::parseTaskMessage(
      R"({"ctrl":1,"navigation_data":{"points":[]}})",
      0.3, 0.4, 10, event, charging));
}

TEST(MqttCodec, PauseResumeAreCompatible)
{
  navdog_task::NavigationEvent event{};
  ASSERT_TRUE(navdog_protocol::MqttCodec::parsePauseMessage(
      R"({"action":1})", event));
  EXPECT_EQ(navdog_task::NavigationEventType::PAUSE, event.type);
  ASSERT_TRUE(navdog_protocol::MqttCodec::parsePauseMessage(
      R"({"action":2})", event));
  EXPECT_EQ(navdog_task::NavigationEventType::RESUME, event.type);
}
