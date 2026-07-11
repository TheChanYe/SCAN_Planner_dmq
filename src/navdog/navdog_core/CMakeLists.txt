cmake_minimum_required(VERSION 3.1)
project(navdog_core)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navdog_core
)

add_library(navdog_core
  src/navigation_coordinator.cpp
  src/task_manager.cpp
  src/start_align_controller.cpp
  src/route_progress_tracker.cpp
  src/route_corridor_observation_gate.cpp
)

target_include_directories(navdog_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)



target_compile_features(navdog_core PUBLIC cxx_std_14)

target_compile_options(navdog_core
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_navigation_coordinator
    test/test_navigation_coordinator.cpp
  )

  if(TARGET test_navigation_coordinator)
    target_link_libraries(test_navigation_coordinator
      navdog_core
    )
  endif()

  catkin_add_gtest(test_task_manager
    test/test_task_manager.cpp
  )

  if(TARGET test_task_manager)
    target_link_libraries(test_task_manager
      navdog_core
    )
  endif()

  catkin_add_gtest(test_start_align_controller
    test/test_start_align_controller.cpp
  )

  if(TARGET test_start_align_controller)
    target_link_libraries(test_start_align_controller
      navdog_core
    )
  endif()

  catkin_add_gtest(test_route_progress_tracker
    test/test_route_progress_tracker.cpp
  )

  if(TARGET test_route_progress_tracker)
    target_link_libraries(test_route_progress_tracker
      navdog_core
    )
  endif()

  catkin_add_gtest(test_route_corridor_observation_gate
    test/test_route_corridor_observation_gate.cpp
  )

  if(TARGET test_route_corridor_observation_gate)
    target_link_libraries(test_route_corridor_observation_gate
      navdog_core
    )
  endif()
endif()

install(
  TARGETS navdog_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/navdog_core/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
