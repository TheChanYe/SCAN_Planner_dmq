cmake_minimum_required(VERSION 2.8.3)
project(scan_planner_dmq)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  plan_env
  path_searching
  bspline_opt
  traj_utils
  scan_planner
  cv_bridge
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scan_planner_manager_dmq
  CATKIN_DEPENDS
    plan_env
    path_searching
    bspline_opt
    traj_utils
    scan_planner
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_library(scan_planner_manager_dmq
  src/planner_manager.cpp
)

target_link_libraries(scan_planner_manager_dmq
  ${catkin_LIBRARIES}
)

add_dependencies(
  scan_planner_manager_dmq
  ${catkin_EXPORTED_TARGETS}
)

add_executable(scan_planner_dmq_node
  src/scan_planner_node.cpp
  src/scan_replan_fsm.cpp
)

target_link_libraries(scan_planner_dmq_node
  scan_planner_manager_dmq
  ${catkin_LIBRARIES}
)

add_dependencies(
  scan_planner_dmq_node
  ${catkin_EXPORTED_TARGETS}
)

add_executable(open_loop_controller_dmq src/open_loop_controller.cpp)
target_link_libraries(open_loop_controller_dmq ${catkin_LIBRARIES})
add_dependencies(open_loop_controller_dmq ${catkin_EXPORTED_TARGETS})

add_executable(closed_loop_controller_dmq src/closed_loop_controller.cpp)
target_link_libraries(closed_loop_controller_dmq ${catkin_LIBRARIES})
add_dependencies(closed_loop_controller_dmq ${catkin_EXPORTED_TARGETS})

add_executable(go2_kinematic_sim_dmq src/go2_kinematic_sim.cpp)
target_link_libraries(go2_kinematic_sim_dmq ${catkin_LIBRARIES})
add_dependencies(go2_kinematic_sim_dmq ${catkin_EXPORTED_TARGETS})

add_executable(go2_gait_publisher_dmq src/go2_gait_publisher.cpp)
target_link_libraries(go2_gait_publisher_dmq ${catkin_LIBRARIES})
add_dependencies(go2_gait_publisher_dmq ${catkin_EXPORTED_TARGETS})

install(
  TARGETS scan_planner_manager_dmq
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
