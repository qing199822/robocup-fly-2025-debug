cmake_minimum_required(VERSION 3.0.2)
project(local_mapping)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS darknet_ros_msgs roscpp)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES local_mapping_health local_mapping_semantic
  CATKIN_DEPENDS darknet_ros_msgs roscpp
  DEPENDS OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(local_mapping_health src/health_monitor.cpp)
target_link_libraries(local_mapping_health ${catkin_LIBRARIES})

add_library(local_mapping_semantic src/semantic_depth_filter.cpp)
add_dependencies(local_mapping_semantic ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  local_mapping_semantic ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(health_monitor_test test/health_monitor_test.cpp)
  if(TARGET health_monitor_test)
    target_link_libraries(
      health_monitor_test local_mapping_health ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(
    semantic_depth_filter_test test/semantic_depth_filter_test.cpp)
  if(TARGET semantic_depth_filter_test)
    add_dependencies(semantic_depth_filter_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(
      semantic_depth_filter_test local_mapping_semantic ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES})
  endif()
endif()
