#include "navdog_runtime/mqtt_bridge.hpp"

#include <gtest/gtest.h>
#include <limits>

namespace navdog_runtime
{
namespace
{
bool parseTask(const std::string& json, navdog::NavigationEvent& event,
               bool& charging)
{
  return MqttBridge::parseTaskMessage(json, 0.3, 0.4, 7, event, charging);
}

TEST(MqttBridgeTest, ValidCtrl1BuildsTask)
{
  navdog::NavigationEvent event{}; bool charging = false;
  ASSERT_TRUE(parseTask(R"({"ctrl":1,"navigation_data":{"max_vx":0.3,"points":[{"x":0,"y":0},{"x":1,"y":0}]}})", event, charging));
  EXPECT_EQ(event.type, navdog::NavigationEventType::START_TASK);
  EXPECT_EQ(event.task.sequence, 7u);
  EXPECT_EQ(event.task.points.size(), 2u);
}

TEST(MqttBridgeTest, MissingPointsRejected)
{
  navdog::NavigationEvent event{}; bool charging = false;
  EXPECT_FALSE(parseTask(R"({"ctrl":1,"navigation_data":{}})", event, charging));
}

TEST(MqttBridgeTest, SinglePointRejected)
{
  navdog::NavigationEvent event{}; bool charging = false;
  EXPECT_FALSE(parseTask(R"({"ctrl":1,"navigation_data":{"points":[{"x":0,"y":0}]}})", event, charging));
}

TEST(MqttBridgeTest, NaNPointRejected)
{
  navdog::NavigationEvent event{}; bool charging = false;
  EXPECT_FALSE(parseTask(R"({"ctrl":1,"navigation_data":{"points":[{"x":"NaN","y":0},{"x":1,"y":0}]}})", event, charging));
}

TEST(MqttBridgeTest, OptionalYawSupported)
{
  navdog::NavigationEvent event{}; bool charging = false;
  ASSERT_TRUE(parseTask(R"({"ctrl":1,"navigation_data":{"points":[{"x":0,"y":0},{"x":1,"y":0,"yaw":1.2}]}})", event, charging));
  EXPECT_FALSE(event.task.points[0].has_yaw);
  EXPECT_TRUE(event.task.points[1].has_yaw);
}

TEST(MqttBridgeTest, MissingZUsesDefault)
{
  navdog::NavigationEvent event{}; bool charging = false;
  ASSERT_TRUE(parseTask(R"({"ctrl":1,"navigation_data":{"points":[{"x":0,"y":0},{"x":1,"y":0}]}})", event, charging));
  EXPECT_DOUBLE_EQ(event.task.points[0].z, 0.3);
}

TEST(MqttBridgeTest, Ctrl0BuildsCancelEvent)
{
  navdog::NavigationEvent event{}; bool charging = false;
  ASSERT_TRUE(parseTask(R"({"ctrl":0})", event, charging));
  EXPECT_EQ(event.type, navdog::NavigationEventType::CANCEL_TASK);
}

TEST(MqttBridgeTest, Ctrl3DoesNotStartNavigation)
{
  navdog::NavigationEvent event{}; bool charging = false;
  ASSERT_TRUE(parseTask(R"({"ctrl":3})", event, charging));
  EXPECT_EQ(event.type, navdog::NavigationEventType::CANCEL_TASK);
  EXPECT_TRUE(charging);
}

TEST(MqttBridgeTest, PauseActionBuildsPause)
{
  navdog::NavigationEvent event{};
  ASSERT_TRUE(MqttBridge::parsePauseMessage(R"({"action":1})", event));
  EXPECT_EQ(event.type, navdog::NavigationEventType::PAUSE);
}

TEST(MqttBridgeTest, ResumeActionBuildsResume)
{
  navdog::NavigationEvent event{};
  ASSERT_TRUE(MqttBridge::parsePauseMessage(R"({"action":2})", event));
  EXPECT_EQ(event.type, navdog::NavigationEventType::RESUME);
}
}
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
