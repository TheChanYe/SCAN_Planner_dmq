cmake_minimum_required(VERSION 3.1)
project(navdog_runtime)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp nav_msgs geometry_msgs std_msgs tf
  navdog_task navdog_protocol navdog_core navdog_scan_adapter
  scan_planner_dmq plan_env
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navdog_runtime
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs std_msgs tf
    navdog_task navdog_protocol navdog_core navdog_scan_adapter
    scan_planner_dmq plan_env
)

include_directories(include SYSTEM
  ${catkin_INCLUDE_DIRS})

add_library(navdog_runtime
  src/navdog_runtime_node.cpp
  src/ros1_config_loader.cpp
  src/final_command_publisher.cpp
)
target_link_libraries(navdog_runtime
  navdog_protocol navdog_core navdog_scan_adapter scan_planner_manager_dmq
  ${catkin_LIBRARIES})
add_dependencies(navdog_runtime ${catkin_EXPORTED_TARGETS})

add_executable(navdog_runtime_node src/main.cpp)
target_link_libraries(navdog_runtime_node navdog_runtime)

add_executable(cmd_vel_owner_mux src/cmd_vel_owner_mux.cpp)
target_link_libraries(cmd_vel_owner_mux navdog_runtime ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_final_command_publisher
    test/test_final_command_publisher.cpp)
  if(TARGET test_final_command_publisher)
    target_link_libraries(test_final_command_publisher navdog_runtime)
  endif()
  catkin_add_gtest(test_runtime_mapping test/test_runtime_mapping.cpp)
  if(TARGET test_runtime_mapping)
    target_link_libraries(test_runtime_mapping navdog_runtime)
  endif()
  catkin_add_gtest(test_runtime_config test/test_runtime_config.cpp)
  if(TARGET test_runtime_config)
    target_link_libraries(test_runtime_config navdog_runtime)
  endif()
endif()

install(TARGETS navdog_runtime navdog_runtime_node cmd_vel_owner_mux
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/navdog_runtime/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
