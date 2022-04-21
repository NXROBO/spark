# Copyright(c) 2020 eaibot limited.
cmake_minimum_required(VERSION 2.8)

project(ydlidar_ros_driver)

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()

############## YDLIDAR SDK START#####################################
#find ydlidar_sdk package

find_package(ydlidar_sdk CONFIG QUIET) 
if(NOT ydlidar_sdk_FOUND) 
    message("${BoldRed}注意：检测到EAI雷达G6的SDK包未安装，请先安装相应的SDK包${ColourReset}")
    message("${BoldRed}安装方法：${ColourReset}")
    message("${BoldGreen}$ cd ~/spark${ColourReset}")
    message("${BoldGreen}$ cd ./src/spark_driver/lidar/YDLidar-SDK/${ColourReset}")
    message("${BoldGreen}$ mkdir build${ColourReset}")
    message("${BoldGreen}$ cd build/${ColourReset}")
    message("${BoldGreen}$ cmake ..${ColourReset}")
    message("${BoldGreen}$ make${ColourReset}")
    message("${BoldGreen}$ sudo make install${ColourReset}")
    message("${BoldGreen}$ cd ~/spark${ColourReset}")
    message("${BoldRed}执行完毕后再重新执行catkin_make${ColourReset}")
    message("${BoldYellow}当然，如果你的雷达不是EAI G6，也不想安装相关的SDK，请执行以下命令${ColourReset}")
    message("${BoldYellow}$ touch ~/spark/src/spark_driver/lidar/ydlidar_g6/CATKIN_IGNORE${ColourReset}")
    message("${BoldYellow}忽略此功能包，不编译它${ColourReset}")
    message(FATAL_ERROR "${Red}${ColourReset}")

else()
    find_package(ydlidar_sdk REQUIRED)
    message("ydlidar_sdk已找到！")
endif()

############## YDLIDAR SDK END#####################################

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  	message_generation
	roscpp
	rospy
	sensor_msgs
	)

##################LaserFan ##########################
#add_message_files(DIRECTORY msg
#  FILES
#  LaserFan.msg
#)

#generate_messages(DEPENDENCIES
#  std_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects als    o need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS
	roscpp rospy sensor_msgs message_runtime
	)
#---------------------------------------------------------------------------------------
# Set default build to release
#---------------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

#---------------------------------------------------------------------------------------
# Compiler config
#---------------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#Include directories
include_directories(${catkin_INCLUDE_DIRS} ${YDLIDAR_SDK_INCLUDE_DIRS}  
 ${PROJECT_SOURCE_DIR}/sdk
  ${PROJECT_SOURCE_DIR}/sdk/src)

#link library directories
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})



#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node src/ydlidar_ros_driver.cpp ${SDK_SRC})

#---------------------------------------------------------------------------------------
# add projects that depend on
#---------------------------------------------------------------------------------------
#add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

#---------------------------------------------------------------------------------------
# link libraries
#---------------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}_node
	${catkin_LIBRARIES}
	${YDLIDAR_SDK_LIBRARIES}
	)


#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------

install(TARGETS ${PROJECT_NAME}_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

