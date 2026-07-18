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
    double accel_x{0.50};
    double decel_x{0.80};
    double accel_y{0.25};
    double decel_y{0.50};
    double accel_yaw{0.80};
    double decel_yaw{1.20};
  };

  VelocitySlewLimiter() {}
  explicit VelocitySlewLimiter(const Config& config) : config_(config) {}

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
    if (dt <= 0.0 || dt > 0.2)
    {
      // Abnormal dt — pass through unchanged.
      out_vx = vx_ = target_vx;
      out_vy = vy_ = target_vy;
      out_yaw = yaw_ = target_yaw;
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
  static double limitAxis(double current, double target, double dt,
                           double accel_limit, double decel_limit)
  {
    const double delta = target - current;

    if (std::abs(delta) < 1e-6)
      return target;

    // Determine rate limit based on whether speed magnitude is increasing
    // or decreasing relative to the current direction.
    const bool same_direction = (current * delta) >= 0.0;
    const double rate = same_direction
        ? (std::abs(target) > std::abs(current) ? accel_limit : decel_limit)
        : decel_limit;  // Direction reversal: decelerate toward zero first.

    const double max_change = rate * dt;
    const double clamped_delta = std::max(-max_change, std::min(delta, max_change));
    return current + clamped_delta;
  }

  Config config_{};
  double vx_{0.0};
  double vy_{0.0};
  double yaw_{0.0};
};

}  // namespace navdog_runtime
