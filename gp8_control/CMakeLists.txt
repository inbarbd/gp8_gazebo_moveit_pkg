cmake_minimum_required(VERSION 3.0.2)
project(gp8_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   float_srv.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gp8_control
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

