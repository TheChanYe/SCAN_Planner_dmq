#include "navdog_protocol/mqtt_log.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <mutex>
#include <sstream>

namespace navdog_protocol
{
namespace
{
std::mutex& logMutex()
{
  static std::mutex mutex;
  return mutex;
}
}  // namespace

std::string MqttLog::format(const char* level, const std::string& message)
{
  const auto now = std::chrono::system_clock::now();
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch());
  const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&seconds, &local_time);

  std::ostringstream stream;
  stream << '[' << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S")
         << '.' << std::setfill('0') << std::setw(3)
         << (millis.count() % 1000) << "][" << (level ? level : "INFO")
         << "][MQTT] " << message;
  return stream.str();
}

void MqttLog::write(const char* level, const std::string& message)
{
  const std::string line = format(level, message);
  std::lock_guard<std::mutex> lock(logMutex());
  std::fprintf(stderr, "%s\n", line.c_str());
  std::fflush(stderr);
}

}  // namespace navdog_protocol
