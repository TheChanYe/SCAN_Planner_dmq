cmake_minimum_required(VERSION 3.1)
project(dmq_dog)

add_compile_options(-std=c++14)

find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dmq_dog
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    tf
    tf2_ros
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${MOSQUITTO_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

add_library(dmq_dog
  src/dmq_mqtt_client.cpp
)
target_link_libraries(dmq_dog
  ${catkin_LIBRARIES}
  ${MOSQUITTO_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)
add_dependencies(dmq_dog ${catkin_EXPORTED_TARGETS})

add_executable(dmq_bridge_node
  src/dmq_bridge_node.cpp
)
target_link_libraries(dmq_bridge_node
  dmq_dog
  ${catkin_LIBRARIES}
)
add_dependencies(dmq_bridge_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_driver_motion_mode test/test_driver_motion_mode.cpp)
  if(TARGET test_driver_motion_mode)
    target_link_libraries(test_driver_motion_mode ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS dmq_dog dmq_bridge_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/dmq_dog/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
