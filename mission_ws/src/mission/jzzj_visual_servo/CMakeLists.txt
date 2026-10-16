cmake_minimum_required(VERSION 3.0.2)
project(jzzj_visual_servo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/jzzj_visual_servo_node.py
  DESTINATION \${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch params
  DESTINATION \${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION \${CATKIN_PACKAGE_SHARE_DESTINATION}
)
