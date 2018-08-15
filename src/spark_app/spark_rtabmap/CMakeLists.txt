cmake_minimum_required(VERSION 2.8.3)
project(spark_rtabmap)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS tf roscpp sensor_msgs)

# What other packages will need to use this package
catkin_package(
    CATKIN_DEPENDS tf roscpp sensor_msgs
)


###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

# Add_executables
#add_executable(laser_footprint_filter src/laser_footprint_filter.cpp)
#target_link_libraries(laser_footprint_filter ${catkin_LIBRARIES})


#############
## Install ##
#############

# Mark executables and/or libraries for installation
#install(TARGETS laser_footprint_filter
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# Mark anything (useful) else for installation

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
