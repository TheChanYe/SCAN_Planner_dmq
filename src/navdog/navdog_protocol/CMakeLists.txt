cmake_minimum_required(VERSION 3.1)
project(navdog_protocol)

find_package(catkin REQUIRED COMPONENTS navdog_task)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

catkin_package(INCLUDE_DIRS include LIBRARIES navdog_protocol
  CATKIN_DEPENDS navdog_task)

add_library(navdog_protocol src/mqtt_codec.cpp src/mqtt_bridge.cpp)
target_include_directories(navdog_protocol PUBLIC include ${catkin_INCLUDE_DIRS}
  ${MOSQUITTO_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})
link_directories(${MOSQUITTO_LIBRARY_DIRS} ${JSONCPP_LIBRARY_DIRS})
target_link_libraries(navdog_protocol navdog_task ${catkin_LIBRARIES}
  ${MOSQUITTO_LIBRARIES} ${JSONCPP_LIBRARIES})
target_compile_features(navdog_protocol PUBLIC cxx_std_14)
target_compile_options(navdog_protocol PRIVATE -Wall -Wextra -Wpedantic)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_mqtt_codec test/test_mqtt_codec.cpp)
  if(TARGET test_mqtt_codec)
    target_link_libraries(test_mqtt_codec navdog_protocol)
  endif()
  catkin_add_gtest(test_mqtt_bridge test/test_mqtt_bridge.cpp)
  if(TARGET test_mqtt_bridge)
    target_link_libraries(test_mqtt_bridge navdog_protocol)
  endif()
endif()

install(TARGETS navdog_protocol
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/navdog_protocol/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
