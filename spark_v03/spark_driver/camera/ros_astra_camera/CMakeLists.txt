cmake_minimum_required(VERSION 2.8.3)
project(astra_camera)

find_package(catkin REQUIRED camera_info_manager dynamic_reconfigure image_transport  nodelet sensor_msgs roscpp message_generation)

find_package(Boost REQUIRED COMPONENTS system thread)

find_path(OpenNI2_INCLUDEDIR
	  NAMES OpenNI.h
	  HINTS ../../../../tools/astra_openni2/Include
	  )
	  
find_library(OpenNI2_LIBRARIES
     NAMES OpenNI2 OniFile ORBBEC PSLink
     HINTS ../../../../tools/astra_openni2/Redist ../../../../tools/astra_openni2/Redist/OpenNI2/Drivers
     PATH_SUFFIXES lib
     )

message(STATUS ${OpenNI2_LIBRARIES})


generate_dynamic_reconfigure_options(cfg/Astra.cfg)
add_service_files(FILES
  GetSerial.srv)
generate_messages()
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES astra_wrapper
  CATKIN_DEPENDS camera_info_manager dynamic_reconfigure image_transport nodelet sensor_msgs roscpp message_runtime
  DEPENDS libastra
)

#link_directories(${CMAKE_CURRENT_BINARY_DIR}/openni2/Redist)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${OpenNI2_INCLUDEDIR}
                    )


add_library(astra_wrapper
   src/astra_convert.cpp
   src/astra_device.cpp
   src/astra_device_info.cpp
   src/astra_timer_filter.cpp
   src/astra_frame_listener.cpp
   src/astra_device_manager.cpp
   src/astra_exception.cpp
   src/astra_video_mode.cpp
)
target_link_libraries(astra_wrapper ${catkin_LIBRARIES} 
  ${OpenNI2_LIBRARIES}
  ${Boost_LIBRARIES} )
add_dependencies(astra_wrapper astra_openni2 ${PROJECT_NAME}_generate_messages_cpp)

add_library(astra_driver_lib
                src/astra_driver.cpp
)
target_link_libraries(astra_driver_lib astra_wrapper ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(astra_driver_lib astra_openni2 ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_library(astra_camera_nodelet
   ros/astra_camera_nodelet.cpp
)
target_link_libraries(astra_camera_nodelet astra_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(astra_camera_nodelet astra_openni2 ${PROJECT_NAME}_gencfg)

add_executable(astra_camera_node
   ros/astra_camera_node.cpp
)
target_link_libraries(astra_camera_node astra_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(astra_camera_node astra_openni2 ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(astra_list_devices
   src/list_devices.cpp
)
target_link_libraries(astra_list_devices astra_wrapper)

add_executable(astra_test_wrapper test/test_wrapper.cpp )
target_link_libraries(astra_test_wrapper astra_wrapper ${Boost_LIBRARIES})
if (UNIX AND NOT APPLE)
  add_executable(astra_usb_reset src/usb_reset.c)
  set(ADDITIONAL_EXECUTABLES "astra_usb_reset")
endif()

install(TARGETS astra_wrapper astra_camera_nodelet astra_camera_node astra_list_devices astra_driver_lib ${ADDITIONAL_EXECUTABLES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES ../../../../tools/astra_openni2/Redist/libOpenNI2.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/
)
install(DIRECTORY ../../../../tools/astra_openni2/Redist/OpenNI2
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/
)
#install(FILES ${CMAKE_BINARY_DIR}/openni2/Redist/libOpenNI2.so
#  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/
#)
#
#install(DIRECTORY ${CMAKE_BINARY_DIR}/openni2/Redist/OpenNI2
#  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/
#)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# add xml file
install(FILES astra_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
