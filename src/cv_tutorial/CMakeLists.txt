cmake_minimum_required(VERSION 2.8.3)
project(cv_tutorial)

#set(OpenCV_DIR /usr/share/OpenCV)
find_package(OpenCV)
find_package(catkin REQUIRED COMPONENTS  
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(cv_tutorial   src/cv_tutorial.cpp)
target_link_libraries(cv_tutorial ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "    config path: ${OpenCV_CONFIG_PATH}")
