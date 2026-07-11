cmake_minimum_required(VERSION 3.1)
project(navdog_scan_adapter)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  navdog_core
  plan_env
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navdog_scan_adapter
  CATKIN_DEPENDS roscpp navdog_core plan_env
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(navdog_scan_adapter
  src/scan_grid_map_query.cpp
  src/scan_route_corridor_evaluator_3d.cpp
)

target_link_libraries(navdog_scan_adapter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(navdog_scan_adapter
  ${catkin_EXPORTED_TARGETS}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    test_scan_route_corridor_evaluator_3d
    test/test_scan_route_corridor_evaluator_3d.cpp
  )

  if(TARGET test_scan_route_corridor_evaluator_3d)
    target_link_libraries(
      test_scan_route_corridor_evaluator_3d
      navdog_scan_adapter
      ${catkin_LIBRARIES}
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
