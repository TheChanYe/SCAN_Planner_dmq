#include <gtest/gtest.h>
#include "navdog_protocol/mqtt_bridge.hpp"

TEST(MqttBridge, DisabledBridgeHasNoTransportSideEffects)
{
  navdog_protocol::MqttBridgeConfig config;
  config.enabled = false;
  navdog_protocol::MqttBridge bridge(config);
  EXPECT_TRUE(bridge.start());
  navdog_task::NavigationEvent event{};
  EXPECT_FALSE(bridge.popEvent(event));
}
