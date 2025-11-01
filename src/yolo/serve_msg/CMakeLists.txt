cmake_minimum_required(VERSION 3.0.2)
project(serve_msg)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  darknet_ros_msgs
  topic_tools
  message_generation
)

add_service_files(
  FILES
  RequestTarget.srv
  ReleaseTarget.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    darknet_ros_msgs
    topic_tools
    message_runtime
  # DEPENDS       # (Uncomment for system dependencies)
)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


  
