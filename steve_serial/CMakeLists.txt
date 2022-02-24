cmake_minimum_required(VERSION 2.8.3)

project(steve_serial)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

#catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
