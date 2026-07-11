#include "navdog_core/navigation_coordinator.hpp"

namespace navdog
{

NavigationCoordinator::NavigationCoordinator(const NavdogConfig& config)
    : config_(config),
      state_(NavState::IDLE),
      active_task_sequence_(0)
{
}

void NavigationCoordinator::reset()
{
  state_ = NavState::IDLE;
  active_task_sequence_ = 0;
}

CoreOutput NavigationCoordinator::update(const CoreInput& input, double now_sec)
{
  (void)input;

  CoreOutput output{};
  output.state = state_;
  output.task_sequence = active_task_sequence_;
  output.planner_action.type = PlannerActionType::NONE;

  output.final_cmd.vx = 0.0;
  output.final_cmd.vy = 0.0;
  output.final_cmd.yaw_rate = 0.0;
  output.final_cmd.valid = true;
  output.final_cmd.source = CommandSource::IDLE_STOP;
  output.final_cmd.stamp_sec = now_sec;

  return output;
}

NavState NavigationCoordinator::state() const noexcept
{
  return state_;
}

const NavdogConfig& NavigationCoordinator::config() const noexcept
{
  return config_;
}

}  // namespace navdog
