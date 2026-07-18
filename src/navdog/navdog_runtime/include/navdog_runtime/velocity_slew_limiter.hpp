#pragma once

#include <algorithm>
#include <cmath>

namespace navdog_runtime
{

/// Pure C++ velocity change rate limiter.
/// No ROS, no navigation state, no message types — just math.
class VelocitySlewLimiter
{
public:
  struct Config
  {
    double accel_x;
    double decel_x;
    double accel_y;
    double decel_y;
    double accel_yaw;
    double decel_yaw;
    Config()
        : accel_x(0.50), decel_x(0.80),
          accel_y(0.25), decel_y(0.50),
          accel_yaw(0.80), decel_yaw(1.20)
    {}
  };

  VelocitySlewLimiter() {}
  explicit VelocitySlewLimiter(const Config& config)
      : config_(config)
  {}

  void setConfig(const Config& config) { config_ = config; }

  /// Reset internal state to zero (used on hard-stop or mode change).
  void reset()
  {
    vx_ = 0.0;
    vy_ = 0.0;
    yaw_ = 0.0;
  }

  /// Set the current output directly (used when switching owner so the
  /// limiter starts from the actual last-published velocity).
  void setCurrent(double vx, double vy, double yaw)
  {
    vx_ = vx;
    vy_ = vy;
    yaw_ = yaw;
  }

  /// Advance one control cycle.
  /// Returns the rate-limited velocity that can be published this frame.
  void update(double target_vx, double target_vy, double target_yaw,
              double dt, double& out_vx, double& out_vy, double& out_yaw)
  {
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.2)
    {
      // Abnormal dt — keep previous output, do NOT jump to target.
      out_vx = vx_;
      out_vy = vy_;
      out_yaw = yaw_;
      return;
    }

    out_vx = limitAxis(vx_, target_vx, dt, config_.accel_x, config_.decel_x);
    out_vy = limitAxis(vy_, target_vy, dt, config_.accel_y, config_.decel_y);
    out_yaw = limitAxis(yaw_, target_yaw, dt, config_.accel_yaw, config_.decel_yaw);

    vx_ = out_vx;
    vy_ = out_vy;
    yaw_ = out_yaw;
  }

private:
  static double clampLocal(double value, double lo, double hi)
  {
    return (value < lo) ? lo : (value > hi) ? hi : value;
  }

  static double limitAxis(double current, double target, double dt,
                           double accel_limit, double decel_limit)
  {
    if (!std::isfinite(current) || !std::isfinite(target) ||
        !std::isfinite(dt) || dt <= 0.0)
    {
      return current;
    }

    const double accel = std::max(0.0, accel_limit);
    const double decel = std::max(0.0, decel_limit);

    // Direction reversal: this cycle only decelerates toward zero.
    if (current * target < 0.0)
    {
      const double max_change = decel * dt;
      if (std::abs(current) <= max_change)
        return 0.0;
      return current - std::copysign(max_change, current);
    }

    const bool increasing = std::abs(target) > std::abs(current);
    const double rate = increasing ? accel : decel;
    const double max_change = rate * dt;
    const double delta = target - current;

    if (std::abs(delta) <= max_change)
      return target;

    return current + std::copysign(max_change, delta);
  }

  Config config_{};
  double vx_{0.0};
  double vy_{0.0};
  double yaw_{0.0};
};

}  // namespace navdog_runtime
