cmake_minimum_required(VERSION 2.8.3)
project(spark_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  spark_base
  spark_carry_object
)


###################################
## catkin specific configuration ##
###################################=
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spark_test
#  CATKIN_DEPENDS roscpp rospy std_msgs spark_base
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(spark_dock_node src/spark_dock_node.cpp)
add_dependencies(spark_dock_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(spark_dock_node ${catkin_LIBRARIES})

add_executable(spark_test_topic_node src/spark_test_topic.cpp)
add_dependencies(spark_test_topic_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(spark_test_topic_node ${catkin_LIBRARIES})


add_executable(spark_turn_left_right_node src/spark_turn_left_right.cpp)
add_dependencies(spark_turn_left_right_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(spark_turn_left_right_node ${catkin_LIBRARIES})

add_executable(spark_test_motion_node src/spark_test_motion.cpp)
add_dependencies(spark_test_motion_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(spark_test_motion_node ${catkin_LIBRARIES})


add_executable(spark_turn_left_right_st_node src/spark_turn_left_right_st.cpp)
add_dependencies(spark_turn_left_right_st_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(spark_turn_left_right_st_node ${catkin_LIBRARIES})

#############
## Install ##
#############

#install(TARGETS spark_test_topic_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install(TARGETS spark_test_motion_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)
# 
#install(DIRECTORY
#  launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
