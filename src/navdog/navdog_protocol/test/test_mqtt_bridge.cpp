#include <gtest/gtest.h>
#define private public
#include "navdog_protocol/mqtt_bridge.hpp"
#undef private

namespace
{
navdog_task::NavigationEvent startEvent(double max_vx,
    navdog_task::TaskMode mode = navdog_task::TaskMode::NORMAL_AVOID)
{
  navdog_task::NavigationEvent event{};
  event.type = navdog_task::NavigationEventType::START_TASK;
  event.task.mode = mode;
  event.task.max_vx = max_vx;
  navdog_task::RoutePoint point{};
  event.task.points.push_back(point);
  return event;
}

navdog_protocol::NavigationMessageMeta metaWithMaxVx(double max_vx)
{
  navdog_protocol::NavigationMessageMeta meta{};
  meta.has_max_vx = true;
  meta.max_vx = max_vx;
  return meta;
}
}  // namespace

TEST(MqttBridge, DisabledBridgeHasNoTransportSideEffects)
{
  navdog_protocol::MqttBridgeConfig config;
  config.enabled = false;
  navdog_protocol::MqttBridge bridge(config);
  EXPECT_TRUE(bridge.start());
  navdog_task::NavigationEvent event{};
  EXPECT_FALSE(bridge.popEvent(event));
}

TEST(MqttBridge, FirstRouteLocksSequenceAndRequestedMaxVx)
{
  navdog_protocol::MqttBridgeConfig config;
  config.enabled = false;
  navdog_protocol::MqttBridge bridge(config);
  auto event = startEvent(0.4);
  std::uint64_t active_sequence = 0;

  EXPECT_TRUE(bridge.enqueueTask(event, false, metaWithMaxVx(0.4),
      active_sequence));

  EXPECT_EQ(1u, active_sequence);
  EXPECT_TRUE(bridge.route_locked_);
  EXPECT_TRUE(bridge.active_requested_max_vx_valid_);
  EXPECT_DOUBLE_EQ(0.4, bridge.active_requested_max_vx_);
  navdog_task::NavigationEvent queued{};
  ASSERT_TRUE(bridge.popEvent(queued));
  EXPECT_EQ(navdog_task::NavigationEventType::START_TASK, queued.type);
  EXPECT_EQ(1u, queued.task.sequence);
}

TEST(MqttBridge, LockedRouteWithSameSpeedIsIgnored)
{
  navdog_protocol::MqttBridgeConfig config;
  config.enabled = false;
  navdog_protocol::MqttBridge bridge(config);
  std::uint64_t active_sequence = 0;
  auto first = startEvent(0.4);
  ASSERT_TRUE(bridge.enqueueTask(first, false, metaWithMaxVx(0.4),
      active_sequence));
  navdog_task::NavigationEvent queued{};
  ASSERT_TRUE(bridge.popEvent(queued));

  auto repeat = startEvent(0.4);
  EXPECT_FALSE(bridge.enqueueTask(repeat, false, metaWithMaxVx(0.4),
      active_sequence));
  EXPECT_EQ(1u, active_sequence);
  EXPECT_FALSE(bridge.popEvent(queued));
}

TEST(MqttBridge, LockedRouteWithChangedSpeedQueuesUpdateOnly)
{
  navdog_protocol::MqttBridgeConfig config;
  config.enabled = false;
  navdog_protocol::MqttBridge bridge(config);
  std::uint64_t active_sequence = 0;
  auto first = startEvent(0.4);
  ASSERT_TRUE(bridge.enqueueTask(first, false, metaWithMaxVx(0.4),
      active_sequence));
  navdog_task::NavigationEvent queued{};
  ASSERT_TRUE(bridge.popEvent(queued));

  auto repeat = startEvent(0.2);
  EXPECT_TRUE(bridge.enqueueTask(repeat, false, metaWithMaxVx(0.2),
      active_sequence));
  EXPECT_EQ(1u, active_sequence);
  ASSERT_TRUE(bridge.popEvent(queued));
  EXPECT_EQ(navdog_task::NavigationEventType::UPDATE_MAX_VX, queued.type);
  EXPECT_DOUBLE_EQ(0.2, queued.max_vx);
  EXPECT_EQ(0u, queued.task.sequence);
  EXPECT_TRUE(bridge.active_requested_max_vx_valid_);
  EXPECT_DOUBLE_EQ(0.2, bridge.active_requested_max_vx_);
}

TEST(MqttBridge, LockedRouteCtrl2DoesNotChangeModeButUpdatesSpeed)
{
  navdog_protocol::MqttBridgeConfig config;
  config.enabled = false;
  navdog_protocol::MqttBridge bridge(config);
  std::uint64_t active_sequence = 0;
  auto first = startEvent(0.4, navdog_task::TaskMode::NORMAL_AVOID);
  ASSERT_TRUE(bridge.enqueueTask(first, false, metaWithMaxVx(0.4),
      active_sequence));
  navdog_task::NavigationEvent queued{};
  ASSERT_TRUE(bridge.popEvent(queued));

  auto repeat = startEvent(0.15, navdog_task::TaskMode::ROUTE_ONLY);
  EXPECT_TRUE(bridge.enqueueTask(repeat, false, metaWithMaxVx(0.15),
      active_sequence));
  ASSERT_TRUE(bridge.popEvent(queued));
  EXPECT_EQ(navdog_task::NavigationEventType::UPDATE_MAX_VX, queued.type);
  EXPECT_DOUBLE_EQ(0.15, queued.max_vx);
}

TEST(MqttBridge, LockedRouteWithoutExplicitSpeedDoesNotUseDefault)
{
  navdog_protocol::MqttBridgeConfig config;
  config.enabled = false;
  navdog_protocol::MqttBridge bridge(config);
  std::uint64_t active_sequence = 0;
  auto first = startEvent(0.4);
  ASSERT_TRUE(bridge.enqueueTask(first, false, metaWithMaxVx(0.4),
      active_sequence));
  navdog_task::NavigationEvent queued{};
  ASSERT_TRUE(bridge.popEvent(queued));

  navdog_protocol::NavigationMessageMeta no_speed_meta{};
  auto repeat = startEvent(config.default_max_vx);
  EXPECT_FALSE(bridge.enqueueTask(repeat, false, no_speed_meta,
      active_sequence));
  EXPECT_DOUBLE_EQ(0.4, bridge.active_requested_max_vx_);
  EXPECT_FALSE(bridge.popEvent(queued));
}

TEST(MqttBridge, CancelClearsRouteLockAndSpeedCache)
{
  navdog_protocol::MqttBridgeConfig config;
  config.enabled = false;
  navdog_protocol::MqttBridge bridge(config);
  std::uint64_t active_sequence = 0;
  auto first = startEvent(0.4);
  ASSERT_TRUE(bridge.enqueueTask(first, false, metaWithMaxVx(0.4),
      active_sequence));

  navdog_task::NavigationEvent cancel{};
  cancel.type = navdog_task::NavigationEventType::CANCEL_TASK;
  EXPECT_TRUE(bridge.enqueueTask(cancel, false,
      navdog_protocol::NavigationMessageMeta{}, active_sequence));

  EXPECT_FALSE(bridge.route_locked_);
  EXPECT_FALSE(bridge.active_requested_max_vx_valid_);
  auto next = startEvent(0.3);
  EXPECT_TRUE(bridge.enqueueTask(next, false, metaWithMaxVx(0.3),
      active_sequence));
  EXPECT_EQ(2u, active_sequence);
}
