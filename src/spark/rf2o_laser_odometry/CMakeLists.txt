PROJECT(rf2o_laser_odometry)

CMAKE_MINIMUM_REQUIRED(VERSION 3.3)
# Require C++17

if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
	set(CMAKE_CXX_STANDARD 17)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  std_msgs
  tf  
)


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
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(MRPT CONFIG QUIET) 
if(NOT MRPT_FOUND) 
    message("${BoldRed}注意：检测到mrpt包未安装，请先安装相应的依赖包${ColourReset}")
    message("${BoldRed}安装方法：${ColourReset}")
    message("${BoldGreen}$ sudo apt install ros-kinetic-mrpt-slam ros-kinetic-mrpt-icp-slam-2d${ColourReset}")
    message("${BoldRed}执行完毕后再重新执行catkin_make${ColourReset}")

    message(FATAL_ERROR "${Red}${ColourReset}")

else()
    find_package(MRPT REQUIRED)
    message("MRPT已找到！")
endif()

find_package(MRPT REQUIRED)
MESSAGE(STATUS "Found MRPT: " ${MRPT_VERSION})

IF(MRPT_VERSION VERSION_LESS 1.9.9)
        # MRPT<2.0
        find_package(MRPT REQUIRED base obs maps slam)
ELSE()
        # MRPT>=2.0
        find_package(MRPT REQUIRED obs maps slam poses core)
ENDIF()


#include_directories(${MRPT_INCLUDE_DIRS})
#MESSAGE( STATUS "MRPT_INCLUDE_DIRS: " ${MRPT_INCLUDE_DIRS})
#link_directories(${MRPT_LIBRARY_DIRS})
#MESSAGE( STATUS "MRPT_LIBRARY_DIRS: " ${MRPT_LIBS})



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES laser_odometry
 CATKIN_DEPENDS nav_msgs roscpp sensor_msgs std_msgs tf
 #DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${MRPT_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(rf2o_laser_odometry_node src/CLaserOdometry2D.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rf2o_laser_odometry_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${EIGEN_LIBRARIES}
   ${MRPT_LIBS}
)
