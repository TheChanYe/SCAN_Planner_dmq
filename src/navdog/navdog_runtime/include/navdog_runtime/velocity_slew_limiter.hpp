#pragma once

#include <algorithm>
#include <cmath>

namespace navdog_runtime
{

/// 纯 C++ 速度变化率限制器，对vx/vy/yaw_rate分别按加速/减速上限逐步递近目标速度，
/// 避免输出速度突变导致机体冲击。
/// 不依赖ROS、导航状态、消息类型 —— 纯数学计算。
class VelocitySlewLimiter
{
public:
  // Config：各轴分别的加/减速度上限（m/s^2 或 rad/s^2）。
  struct Config
  {
    double accel_x;     // x方向加速上限
    double decel_x;     // x方向减速上限
    double accel_y;     // y方向加速上限
    double decel_y;     // y方向减速上限
    double accel_yaw;   // 角速度加速上限
    double decel_yaw;   // 角速度减速上限
    Config()
        : accel_x(0.50), decel_x(0.80),
          accel_y(0.25), decel_y(0.50),
          accel_yaw(0.80), decel_yaw(1.20)
    {}
  };

  // 默认构造函数：使用默认配置。
  VelocitySlewLimiter() {}
  // 构造函数：使用指定配置。
  explicit VelocitySlewLimiter(const Config& config)
      : config_(config)
  {}

  // 运行时更新配置。
  void setConfig(const Config& config) { config_ = config; }

  /// 将内部状态（vx/vy/yaw）重置为0（用于紧急停止或模式切换时）。
  void reset()
  {
    vx_ = 0.0;
    vy_ = 0.0;
    yaw_ = 0.0;
  }

  /// 直接设置当前输出值（用于切换指令所有者时，使限制器从实际上一次已发布的速度
  /// 开始推进，避免交接冲击）。
  void setCurrent(double vx, double vy, double yaw)
  {
    vx_ = vx;
    vy_ = vy;
    yaw_ = yaw;
  }

  /// 推进一个控制周期。
  /// 输入：target_vx/vy/yaw - 本周期的目标速度；dt - 控制间隔（秒）。
  /// 输出：out_vx/vy/yaw - 本帧实际可发布的限速后速度。
  /// 若dt异常（非有限数、非正数或过大）则保持上一帧输出不变，不直接跳变到目标。
  /// 否则对三个轴分别调用limitAxis限速，并更新内部状态为本帧输出。
  void update(double target_vx, double target_vy, double target_yaw,
              double dt, double& out_vx, double& out_vy, double& out_yaw)
  {
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.2)
    {
      // dt异常 —— 保持上一次输出，不跳变到目标。
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
  // clampLocal：将value限制在[lo, hi]区间内。
  static double clampLocal(double value, double lo, double hi)
  {
    return (value < lo) ? lo : (value > hi) ? hi : value;
  }

  // limitAxis：对单个轴按加/减速上限逐步逆近目标值。
  // 步骤：1.任一输入非法则直接返回当前值（不变）；
  // 2.方向反转（current与target异号）时，本周期只允许按减速上限向零靠拢，
  //    若已接近零则直接归零，否则按最大变化量向零方向移动（避免直接穿过零点反向过头）；
  // 3.同向情况下根据是否增大选择加速/减速率，若误差小于本周期最大可变化量则直接
  //    达到目标，否则按最大变化量向目标方向逐步推进。
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

    // 方向反转：本周期仅减速向零靠拢。
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

  Config config_{};       // 限速配置
  double vx_{0.0};        // 当前x方向速度
  double vy_{0.0};        // 当前y方向速度
  double yaw_{0.0};       // 当前角速度
};

}  // namespace navdog_runtime
