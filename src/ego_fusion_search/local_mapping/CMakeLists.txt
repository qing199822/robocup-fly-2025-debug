cmake_minimum_required(VERSION 3.0.2)
project(local_mapping)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  darknet_ros_msgs
  geometry_msgs
  message_filters
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  search_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES local_mapping_core
  CATKIN_DEPENDS
    cv_bridge
    darknet_ros_msgs
    geometry_msgs
    message_filters
    nav_msgs
    pcl_conversions
    pcl_ros
    roscpp
    search_msgs
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
  DEPENDS OpenCV PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(local_mapping_core
  src/health_monitor.cpp
  src/integration_rate_limiter.cpp
  src/semantic_depth_filter.cpp
  src/time_rollback_guard.cpp
  src/voxel_map.cpp
  src/frontier_selector.cpp
)
add_dependencies(local_mapping_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  local_mapping_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(local_mapping_node src/local_mapping_node.cpp)
add_dependencies(local_mapping_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  local_mapping_node
  local_mapping_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  foreach(test_target
      health_monitor_test
      integration_rate_limiter_test
      time_rollback_guard_test
      semantic_depth_filter_test
      voxel_map_test
      frontier_selector_test)
    catkin_add_gtest(${test_target} test/${test_target}.cpp)
    if(TARGET ${test_target})
      add_dependencies(${test_target} ${catkin_EXPORTED_TARGETS})
      target_link_libraries(
        ${test_target}
        local_mapping_core
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
      )
    endif()
  endforeach()
  add_rostest(test/local_mapping_node.test)
  add_rostest(test/local_mapping_clock_rollback.test)
endif()

install(TARGETS local_mapping_core local_mapping_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
