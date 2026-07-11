cmake_minimum_required(VERSION 3.1)
project(navdog_scan_adapter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  navdog_core
  plan_env
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navdog_scan_adapter
  CATKIN_DEPENDS roscpp navdog_core plan_env
  DEPENDS EIGEN3
)

add_library(navdog_scan_adapter
  src/scan_grid_map_query.cpp
  src/scan_route_corridor_evaluator_3d.cpp
)

target_include_directories(navdog_scan_adapter
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(navdog_scan_adapter PUBLIC cxx_std_14)

target_compile_options(navdog_scan_adapter
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_scan_route_corridor_evaluator_3d
    test/test_scan_route_corridor_evaluator_3d.cpp
  )

  if(TARGET test_scan_route_corridor_evaluator_3d)
    target_link_libraries(test_scan_route_corridor_evaluator_3d
      navdog_scan_adapter
    )
  endif()
endif()

install(
  TARGETS navdog_scan_adapter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/navdog_scan_adapter/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
