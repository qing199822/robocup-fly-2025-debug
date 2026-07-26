cmake_minimum_required(VERSION 3.0.2)
project(competition_compliance)

find_package(catkin REQUIRED COMPONENTS geometry_msgs rospy tf tf2_ros)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS geometry_msgs rospy tf tf2_ros)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_model.py)
endif()
