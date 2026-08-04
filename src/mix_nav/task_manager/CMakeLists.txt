cmake_minimum_required(VERSION 3.0.2)
project(task_manager)

# --- 1. 寻找所有需要的 catkin 包 ---
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
)

# --- 2. 使用 pkg-config 寻找 jsoncpp (这种方式在您的系统上有效) ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

# --- 3. 声明 catkin 包的构建信息 ---
# 这是非常重要的一步，它会正确配置包的导出信息
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES mission_manager_lib # 如果您希望其他ROS包能链接这个库，就取消本行注释
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs
  # DEPENDS JsonCpp # 系统依赖通常不需要放在这里
)

# --- 4. 添加头文件目录 ---
# 包含本包的 "include" 目录，catkin的头文件目录，以及jsoncpp的头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

# --- 5. 声明编译目标 ---
add_library(mission_definition_lib
  src/mission_definition.cpp
)

add_library(mission_progress_lib
  src/mission_progress.cpp
)

# 首先，编译 mission_manager.cpp 成为一个库
add_library(mission_manager_lib
  src/mission_manager.cpp
)

# 然后，编译 multi_mission_launcher.cpp 成为一个可执行文件
add_executable(multi_mission_launcher
  src/multi_mission_launcher.cpp
)

# --- 6. 链接库 ---
# 为可执行文件链接它所需要的所有库
target_link_libraries(multi_mission_launcher
  mission_manager_lib      # 链接我们自己编译的库
  mission_definition_lib
  ${catkin_LIBRARIES}      # 链接catkin提供的所有库
  ${JSONCPP_LIBRARIES}     # 链接jsoncpp库
)

# 我们的 mission_manager_lib 库本身也需要链接一些库
target_link_libraries(mission_manager_lib
  mission_progress_lib
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

target_link_libraries(mission_definition_lib
  ${JSONCPP_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(mission_progress_test test/mission_progress_test.cpp)
  if(TARGET mission_progress_test)
    target_link_libraries(mission_progress_test
      mission_progress_lib ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(mission_definition_test test/mission_definition_test.cpp)
  if(TARGET mission_definition_test)
    target_link_libraries(mission_definition_test
      mission_definition_lib ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})
  endif()

  catkin_add_nosetests(test/test_mission_clearance.py)
  add_rostest(test/mission_active.test)
endif()

# 注意：我们不再需要之前那个复杂的 install() 部分。
# catkin_make 或 catkin build 会自动将可执行文件放到 devel/lib/task_manager/ 目录下，
# 这正是 roslaunch 和 rosrun 需要的地方。
