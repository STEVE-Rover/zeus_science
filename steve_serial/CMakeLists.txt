cmake_minimum_required(VERSION 2.8.3)

project(zeus_serial)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  testmsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy message_runtime
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
