cmake_minimum_required(VERSION 3.0.2)
project(local_mapping)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES local_mapping_health
  CATKIN_DEPENDS roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(local_mapping_health src/health_monitor.cpp)
target_link_libraries(local_mapping_health ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(health_monitor_test test/health_monitor_test.cpp)
  if(TARGET health_monitor_test)
    target_link_libraries(
      health_monitor_test local_mapping_health ${catkin_LIBRARIES})
  endif()
endif()
