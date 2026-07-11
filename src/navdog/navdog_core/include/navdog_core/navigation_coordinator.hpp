#pragma once

#include "navdog_core/types.hpp"
#include "navdog_core/config.hpp"

#include <cstdint>

namespace navdog
{

class NavigationCoordinator
{
public:
  explicit NavigationCoordinator(
      const NavdogConfig& config = NavdogConfig{});

  void reset();

  CoreOutput update(
      const CoreInput& input,
      double now_sec);

  NavState state() const noexcept;

  const NavdogConfig& config() const noexcept;

private:
  NavdogConfig config_{};
  NavState state_{NavState::IDLE};
  std::uint64_t active_task_sequence_{0};
};

}  // namespace navdog
