cmake_minimum_required(VERSION 3.0.2)
project(cleanbot_course_project)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  move_base_msgs
  trajectory_msgs
  actionlib
  actionlib_msgs
  tf
  tf2_ros
  cv_bridge
  image_transport
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/image_preprocess_node.py
  scripts/aruco_marker_detector.py
  scripts/coverage_task_manager.py
  scripts/person_follow_node.py
  scripts/target_3d_locator.py
  scripts/arm_preset_action_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config worlds docs rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".git" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.pyo" EXCLUDE
)

install(PROGRAMS test/smoke_noetic.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
