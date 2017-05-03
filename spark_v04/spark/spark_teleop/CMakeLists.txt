cmake_minimum_required(VERSION 2.8.3)
project(spark_teleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES teleop
  CATKIN_DEPENDS roscpp rospy std_msgs tf
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
 add_executable(spark_teleop_node src/teleop_node.cpp)
 target_link_libraries(spark_teleop_node
   ${catkin_LIBRARIES}
 )
 add_dependencies(spark_teleop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS spark_teleop_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
 
 install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
