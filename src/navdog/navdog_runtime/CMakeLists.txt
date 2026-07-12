cmake_minimum_required(VERSION 3.1)
project(navdog_runtime)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp nav_msgs geometry_msgs std_msgs tf
  navdog_core navdog_scan_adapter scan_planner_dmq plan_env
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navdog_runtime
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs std_msgs tf
    navdog_core navdog_scan_adapter scan_planner_dmq plan_env
)

include_directories(include SYSTEM
  ${catkin_INCLUDE_DIRS} ${MOSQUITTO_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})
link_directories(${MOSQUITTO_LIBRARY_DIRS} ${JSONCPP_LIBRARY_DIRS})

add_library(navdog_runtime
  src/mqtt_bridge.cpp
  src/navdog_runtime_node.cpp
)
target_link_libraries(navdog_runtime
  navdog_core navdog_scan_adapter scan_planner_manager_dmq
  ${catkin_LIBRARIES} ${MOSQUITTO_LIBRARIES} ${JSONCPP_LIBRARIES})
add_dependencies(navdog_runtime ${catkin_EXPORTED_TARGETS})

add_executable(navdog_runtime_node src/main.cpp)
target_link_libraries(navdog_runtime_node navdog_runtime)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_mqtt_bridge test/test_mqtt_bridge.cpp)
  if(TARGET test_mqtt_bridge)
    target_link_libraries(test_mqtt_bridge navdog_runtime)
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

install(TARGETS navdog_runtime navdog_runtime_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/navdog_runtime/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
