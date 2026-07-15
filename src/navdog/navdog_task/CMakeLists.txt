cmake_minimum_required(VERSION 3.1)
project(navdog_task)

find_package(catkin REQUIRED)

catkin_package(INCLUDE_DIRS include LIBRARIES navdog_task)

add_library(navdog_task src/task_manager.cpp)
target_include_directories(navdog_task PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(navdog_task PUBLIC cxx_std_14)
target_compile_options(navdog_task PRIVATE -Wall -Wextra -Wpedantic)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_task_manager test/test_task_manager.cpp)
  if(TARGET test_task_manager)
    target_link_libraries(test_task_manager navdog_task)
  endif()
endif()

install(TARGETS navdog_task
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/navdog_task/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
