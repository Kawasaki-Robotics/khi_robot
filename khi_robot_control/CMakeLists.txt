cmake_minimum_required(VERSION 2.8.3)
project(khi_robot_control)

find_package(catkin REQUIRED
  COMPONENTS
    controller_manager
    diagnostic_updater
    hardware_interface
    joint_limits_interface
    khi_robot_msgs
    realtime_tools
    trajectory_msgs
    transmission_interface)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    diagnostic_updater
    khi_robot_msgs
    trajectory_msgs)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## KRNX
## TODO: external library path
if( ${CMAKE_SYSTEM_PROCESSOR} MATCHES ".*x86_64.*" )
  set(krnx_lib_name krnx-x86_64)
elseif( ${CMAKE_SYSTEM_PROCESSOR} MATCHES ".*aarch64.*" )
  set(krnx_lib_name krnx-aarch64)
elseif( ${CMAKE_SYSTEM_PROCESSOR} MATCHES ".*i386.*" )
  set(krnx_lib_name krnx-i386)
elseif( ${CMAKE_SYSTEM_PROCESSOR} MATCHES ".*armhf.*" )
  set(krnx_lib_name krnx-armhf)
elseif( ${CMAKE_SYSTEM_PROCESSOR} MATCHES ".*armel.*" )
  set(krnx_lib_name krnx-armel)
else()
  message(WARNING "\nCould not detect system processor architecture: assuming 'x86_64'. This may result in non-working binaries.")
  set(krnx_lib_name krnx-x86_64)
endif()

add_library(krnx_lib SHARED IMPORTED)
set_target_properties(krnx_lib PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/lib/lib${krnx_lib_name}.so)
set(krnx_LIBRARIES krnx_lib)

## Declare a C++ library
add_library(khi_robot_client src/khi_robot_krnx_driver.cpp src/khi_robot_client.cpp )

## Declare a C++ executable for ROS_CONTROL
add_executable(main src/main.cpp src/khi_robot_hardware_interface.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(khi_robot_client ${catkin_LIBRARIES} ${krnx_LIBRARIES})
target_link_libraries(main ${catkin_LIBRARIES} khi_robot_client)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
