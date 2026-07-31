cmake_minimum_required(VERSION 3.0.2)
project(race_stack)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/waypoint_manager.py
    scripts/sign_detector.py
    scripts/lidar_safety.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
