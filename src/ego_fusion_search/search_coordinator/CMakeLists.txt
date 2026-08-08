cmake_minimum_required(VERSION 3.0.2)
project(search_coordinator)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES search_coordinator_core
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(search_coordinator_core src/coordinator.cpp)
target_link_libraries(search_coordinator_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(coordinator_test test/coordinator_test.cpp)
  if(TARGET coordinator_test)
    target_link_libraries(
      coordinator_test search_coordinator_core ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS search_coordinator_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
