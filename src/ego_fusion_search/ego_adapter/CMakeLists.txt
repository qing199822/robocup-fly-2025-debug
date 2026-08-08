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

add_library(ego_command_policy
  src/bspline_sampler.cpp
  src/command_policy.cpp
)
add_dependencies(ego_command_policy ${catkin_EXPORTED_TARGETS})
target_link_libraries(ego_command_policy ${catkin_LIBRARIES})

add_executable(ego_adapter_node src/ego_adapter_node.cpp)
add_dependencies(ego_adapter_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ego_adapter_node ego_command_policy ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/ego_camera_bootstrap.py
  scripts/ego_external_guard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  foreach(test_target command_policy_test bspline_sampler_test)
    catkin_add_gtest(${test_target} test/${test_target}.cpp)
    if(TARGET ${test_target})
      add_dependencies(${test_target} ${catkin_EXPORTED_TARGETS})
      target_link_libraries(
        ${test_target} ego_command_policy ${catkin_LIBRARIES})
    endif()
  endforeach()
  add_rostest(test/ego_adapter_node.test)
endif()

install(TARGETS ego_command_policy ego_adapter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
