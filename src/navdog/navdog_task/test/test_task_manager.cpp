#include <gtest/gtest.h>
#include "navdog_task/task_manager.hpp"

namespace
{
navdog_task::NavigationEvent startEvent()
{
  navdog_task::NavigationEvent event{};
  event.type = navdog_task::NavigationEventType::START_TASK;
  event.task.max_vx = 0.4;
  event.task.points.push_back(navdog_task::RoutePoint{});
  return event;
}
}

TEST(TaskManager, TransfersRouteOnceAndKeepsOnlySession)
{
  navdog_task::TaskManager manager;
  auto transition = manager.handleEvent(startEvent());
  EXPECT_EQ(navdog_task::TaskHandleResult::STARTED, transition.result);
  ASSERT_EQ(1u, transition.accepted_route.size());
  EXPECT_TRUE(manager.session().active);
  EXPECT_EQ(1u, manager.session().sequence);
  EXPECT_TRUE(manager.handleEvent({}).accepted_route.empty());
}

TEST(TaskManager, RejectsEmptyRouteAndSequenceIsMonotonic)
{
  navdog_task::TaskManager manager;
  navdog_task::NavigationEvent invalid = startEvent();
  invalid.task.points.clear();
  EXPECT_EQ(navdog_task::TaskHandleResult::REJECTED_INVALID_TASK,
            manager.handleEvent(invalid).result);
  EXPECT_EQ(1u, manager.handleEvent(startEvent()).session.sequence);
  navdog_task::NavigationEvent cancel{};
  cancel.type = navdog_task::NavigationEventType::CANCEL_TASK;
  EXPECT_EQ(navdog_task::TaskHandleResult::CANCELLED,
            manager.handleEvent(cancel).result);
  EXPECT_EQ(2u, manager.handleEvent(startEvent()).session.sequence);
}

TEST(TaskManager, PauseResumeCancelAndSpeedDoNotCarryRoute)
{
  navdog_task::TaskManager manager;
  manager.handleEvent(startEvent());
  navdog_task::NavigationEvent event{};
  event.type = navdog_task::NavigationEventType::PAUSE;
  EXPECT_TRUE(manager.handleEvent(event).session.paused);
  event.type = navdog_task::NavigationEventType::RESUME;
  EXPECT_FALSE(manager.handleEvent(event).session.paused);
  event.type = navdog_task::NavigationEventType::UPDATE_MAX_VX;
  event.max_vx = 0.6;
  const auto speed = manager.handleEvent(event);
  EXPECT_EQ(navdog_task::TaskHandleResult::MAX_VX_UPDATED, speed.result);
  EXPECT_TRUE(speed.accepted_route.empty());
  event.type = navdog_task::NavigationEventType::CANCEL_TASK;
  const auto cancel = manager.handleEvent(event);
  EXPECT_FALSE(cancel.session.active);
  EXPECT_TRUE(cancel.route_changed);
}

TEST(TaskManager, ResetDoesNotReuseSequence)
{
  navdog_task::TaskManager manager;
  EXPECT_EQ(1u, manager.handleEvent(startEvent()).session.sequence);
  manager.reset();
  EXPECT_EQ(2u, manager.handleEvent(startEvent()).session.sequence);
}
