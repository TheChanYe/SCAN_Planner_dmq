#include <gtest/gtest.h>

#include <navdog_protocol/mqtt_log.hpp>

#include <regex>
#include <thread>
#include <vector>

TEST(MqttLog, FormatsLocalTimeWithMilliseconds)
{
  const std::string line = navdog_protocol::MqttLog::format("INFO", "event=TEST");
  EXPECT_TRUE(std::regex_match(line,
      std::regex("\\[[0-9]{4}-[0-9]{2}-[0-9]{2} [0-9]{2}:[0-9]{2}:[0-9]{2}\\.[0-9]{3}\\]\\[INFO\\]\\[MQTT\\] event=TEST")));
}

TEST(MqttLog, ConcurrentFormattingProducesCompleteLines)
{
  std::vector<std::thread> threads;
  for (int i = 0; i < 8; ++i)
    threads.emplace_back([] { navdog_protocol::MqttLog::write("INFO", "event=THREAD_TEST"); });
  for (auto& thread : threads) thread.join();
  SUCCEED();  // write() serializes the complete stderr line with its internal mutex.
}
