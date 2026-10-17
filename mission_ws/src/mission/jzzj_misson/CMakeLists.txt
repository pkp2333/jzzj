cmake_minimum_required(VERSION 3.0.2)
project(jzzj_misson)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/jzzj_misson_node.py
  DESTINATION \${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch params
  DESTINATION \${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION \${CATKIN_PACKAGE_SHARE_DESTINATION}
)
