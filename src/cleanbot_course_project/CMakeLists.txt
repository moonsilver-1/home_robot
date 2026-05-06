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
  scripts/image_sample_collector.py
  scripts/follower_controller.py
  scripts/camera_follow_node.py
  scripts/object_recognition_node.py
  scripts/opencv_template_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config worlds docs urdf models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".git" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.pyo" EXCLUDE
)

install(PROGRAMS test/smoke_noetic.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
