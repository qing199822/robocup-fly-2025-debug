cmake_minimum_required(VERSION 3.0.2)
project(search_coordinator)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  search_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES search_coordinator_core
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp search_msgs std_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(search_coordinator_core src/coordinator.cpp)
target_link_libraries(search_coordinator_core ${catkin_LIBRARIES})

add_executable(search_coordinator_node src/search_coordinator_node.cpp)
add_dependencies(search_coordinator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  search_coordinator_node search_coordinator_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(coordinator_test test/coordinator_test.cpp)
  if(TARGET coordinator_test)
    target_link_libraries(
      coordinator_test search_coordinator_core ${catkin_LIBRARIES})
  endif()
  add_rostest(test/search_coordinator_node.test)
endif()

install(TARGETS search_coordinator_core search_coordinator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
