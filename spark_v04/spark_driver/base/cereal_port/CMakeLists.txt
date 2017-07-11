cmake_minimum_required(VERSION 2.8.3)
project(cereal_port)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cereal_port
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
)
add_library(cereal_port src/CerealPort.cpp)

#############
## Install ##
#############
install(TARGETS cereal_port
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
