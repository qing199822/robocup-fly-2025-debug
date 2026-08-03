cmake_minimum_required(VERSION 3.0.2)
project(safety_filter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES safety_policy
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(safety_policy src/safety_policy.cpp)
target_link_libraries(safety_policy ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(safety_policy_test test/safety_policy_test.cpp)
  if(TARGET safety_policy_test)
    target_link_libraries(safety_policy_test safety_policy ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS safety_policy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
