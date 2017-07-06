cmake_minimum_required(VERSION 2.8.3)
project(spark_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  GyroMessage.msg
  SparkBaseOdom.msg
  SparkBaseDock.msg
  SparkBaseSensor.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy std_msgs tf
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ../cereal_port/include
)

set(spark_base_node_SRCS
  src/spark_base.cpp
  src/spark_base_interface.cpp
)

###########
## Build ##
###########
## Declare a C++ executable
add_executable(spark_base_node ${spark_base_node_SRCS})
target_link_libraries(spark_base_node
  ${catkin_LIBRARIES}
  cereal_port
)
add_dependencies(spark_base_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp
)


#############
## Install ##
#############

install(TARGETS spark_base_node
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
