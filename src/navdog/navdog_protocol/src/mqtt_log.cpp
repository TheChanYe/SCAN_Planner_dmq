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
// logMutex：返回进程内共享的静态互斥锁，保证来自Mosquitto网络线程与ROS控制线程的日志
// 不会交错输出。
std::mutex& logMutex()
{
  static std::mutex mutex;
  return mutex;
}
}  // namespace

// format：将日志级别与正文拼接成带本地毫秒时间戳的单行文本（格式：
// [YYYY-MM-DD HH:MM:SS.mmm][LEVEL][MQTT] message），不写日志，仅负责格式化，
// 供 write() 和测试代码共用。
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

// write：格式化后在锁保护下原子地写入stderr并立即flush，避免多线程交替输出导致的日志行混乱。
void MqttLog::write(const char* level, const std::string& message)
{
  const std::string line = format(level, message);
  std::lock_guard<std::mutex> lock(logMutex());
  std::fprintf(stderr, "%s\n", line.c_str());
  std::fflush(stderr);
}

}  // namespace navdog_protocol
