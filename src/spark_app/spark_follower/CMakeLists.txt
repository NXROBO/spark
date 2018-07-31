cmake_minimum_required(VERSION 2.8.3)
project(spark_follower) 

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs  std_srvs pcl_ros message_generation nodelet)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES t_schedule_node
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib nxrobo_msgs common
)

###########
## Build ##
###########
## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(nxfollower_nodelet src/nxfollower_nodelet.cpp)
target_link_libraries(nxfollower_nodelet ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS nxfollower_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
) 

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 

install(DIRECTORY
  plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  launch/follow_run.sh
  DESTINATION ./
) 
