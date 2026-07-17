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
  EXPECT_EQ(R"({"error":2,"status":5})",
            navdog_protocol::MqttCodec::encodeStatus(5, 2));
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
