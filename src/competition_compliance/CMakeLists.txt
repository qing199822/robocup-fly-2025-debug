cmake_minimum_required(VERSION 3.0.2)
project(competition_compliance)

find_package(catkin REQUIRED COMPONENTS geometry_msgs rospy tf tf2_ros)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS geometry_msgs rospy tf tf2_ros)

catkin_install_python(PROGRAMS
  scripts/prepare_model.py
  scripts/sensor_tf_publisher.py
  scripts/verify_full.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_model.py)
  catkin_add_nosetests(test/test_manifest.py)
  catkin_add_nosetests(test/test_launch_contract.py)
  catkin_add_nosetests(test/test_sensor_tf_publisher.py)
  catkin_add_nosetests(test/test_tf_math.py)
  catkin_add_nosetests(test/test_ownership.py)
endif()
