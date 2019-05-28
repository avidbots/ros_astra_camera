cmake_minimum_required(VERSION 2.8.3)
project(astra_camera)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED camera_info_manager dynamic_reconfigure image_transport  nodelet sensor_msgs roscpp message_generation)

find_package(Boost REQUIRED COMPONENTS system thread)

if(MSVC64 OR MINGW64)
  set(X86_64 1)
elseif(MINGW OR (MSVC AND NOT CMAKE_CROSSCOMPILING))
  set(X86 1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "amd64.*|x86_64.*|AMD64.*")
  set(X86_64 1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686.*|i386.*|x86.*|amd64.*|AMD64.*")
  set(X86 1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm.*|ARM.*)")
  set(ARM 1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64.*|AARCH64.*)")
  set(AARCH64 1)
endif()

if(ARM OR AARCH64)
  option(FILTER "OB Filter library" OFF)
else()
  option(FILTER "OB Filter library" ON)
endif()

message(STATUS "arm is ${ARM}, aarch is ${AARCH64}, x86 is ${X86}, x64 is ${X86_64}, filter is ${FILTER}")

if( ${FILTER} STREQUAL "ON" )
set(obfilter "On")
else()
set(obfilter "Off")
endif()

generate_dynamic_reconfigure_options(cfg/Astra.cfg)
add_service_files(FILES
  GetSerial.srv)

add_message_files(
  DIRECTORY msg
  FILES
  astra_registration_info.msg
  )

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES astra_wrapper
  CATKIN_DEPENDS camera_info_manager dynamic_reconfigure image_transport nodelet sensor_msgs roscpp message_runtime
  #DEPENDS libastra
)

link_directories(3rdparty/OpenNI2/Redist)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    3rdparty/OpenNI2/Include
                    3rdparty/LibUSB/Include
                    )

add_library(astra_wrapper
   src/astra_convert.cpp
   src/astra_device.cpp
   src/astra_advanced_device.cpp
   src/astra_device_info.cpp
   src/astra_timer_filter.cpp
   src/astra_frame_listener.cpp
   src/astra_frame_reader.cpp
   src/astra_device_manager.cpp
   src/astra_exception.cpp
   src/astra_video_mode.cpp
   src/COBDevice.cpp
)
target_link_libraries(astra_wrapper ${catkin_LIBRARIES} -lOpenNI2 -lrt -lusb-1.0
  ${Boost_LIBRARIES} )
add_dependencies(astra_wrapper ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_library(astra_driver_lib
                src/astra_driver.cpp
                src/astra_multi_driver.cpp
)
target_link_libraries(astra_driver_lib astra_wrapper ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lOpenNI2 )
add_dependencies(astra_driver_lib ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_library(astra_camera_nodelet
   ros/astra_camera_nodelet.cpp
)
target_link_libraries(astra_camera_nodelet astra_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lOpenNI2 )
add_dependencies(astra_camera_nodelet ${PROJECT_NAME}_gencfg)

add_executable(astra_camera_node
   ros/astra_camera_node.cpp
)
target_link_libraries(astra_camera_node astra_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} -lOpenNI2 )
add_dependencies(astra_camera_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

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

install(FILES 3rdparty/OpenNI2/Redist/libOpenNI2.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/
)

install(DIRECTORY 3rdparty/OpenNI2/Redist/OpenNI2
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# add xml file
install(FILES astra_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
