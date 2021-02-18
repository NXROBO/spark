cmake_minimum_required(VERSION 2.8.3)
project(iiiroboticslidar2)

set(iiiroboticslidar2_SDK_PATH "./sdk/")

FILE(GLOB iiiroboticslidar2_SDK_SRC 
  "${iiiroboticslidar2_SDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
)

include_directories(
  ${iiiroboticslidar2_SDK_PATH}/include
  ${iiiroboticslidar2_SDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(iiirobotics_lidar2_node src/node.cpp ${iiiroboticslidar2_SDK_SRC})
target_link_libraries(iiirobotics_lidar2_node ${catkin_LIBRARIES} -lrt)

add_executable(iiirobotics_lidar2_node_client src/client.cpp)
target_link_libraries(iiirobotics_lidar2_node_client ${catkin_LIBRARIES})

install(TARGETS iiirobotics_lidar2_node iiirobotics_lidar2_node_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
