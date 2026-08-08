cmake_minimum_required(VERSION 3.0.2)
project(ego_adapter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  search_msgs
  std_msgs
  tf2
  topic_tools
  traj_utils
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ego_command_policy
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    quadrotor_msgs
    roscpp
    search_msgs
    std_msgs
    tf2
    topic_tools
    traj_utils
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(ego_command_policy src/command_policy.cpp)
add_dependencies(ego_command_policy ${catkin_EXPORTED_TARGETS})
target_link_libraries(ego_command_policy ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(command_policy_test test/command_policy_test.cpp)
  if(TARGET command_policy_test)
    add_dependencies(command_policy_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(
      command_policy_test ego_command_policy ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS ego_command_policy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
