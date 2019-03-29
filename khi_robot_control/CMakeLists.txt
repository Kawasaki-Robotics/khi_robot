cmake_minimum_required(VERSION 2.8.3)
project(khi_robot_control)

find_package(catkin REQUIRED COMPONENTS controller_manager diagnostic_updater transmission_interface hardware_interface joint_limits_interface trajectory_msgs khi_robot_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS diagnostic_updater trajectory_msgs khi_robot_msgs)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## KRNX
## TODO: external library path
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
if( ${CMAKE_SYSTEM_PROCESSOR} MATCHES ".*x86_64.*" )
  set(krnx_LIBRARY krnx-x86_64)
elseif( ${CMAKE_SYSTEM_PROCESSOR} MATCHES ".*aarch64.*" )
  set(krnx_LIBRARY krnx-aarch64)
elseif( ${CMAKE_SYSTEM_PROCESSOR} MATCHES ".*i386.*" )
  set(krnx_LIBRARY krnx-i386)
elseif( ${CMAKE_SYSTEM_PROCESSOR} MATCHES ".*armhf.*" )
  set(krnx_LIBRARY krnx-armhf)
elseif( ${CMAKE_SYSTEM_PROCESSOR} MATCHES ".*armel.*" )
  set(krnx_LIBRARY krnx-armel)
else()
  set(krnx_LIBRARY krnx-x86_64)
endif()

## Declare a C++ library
add_library(khi_robot_client src/khi_robot_krnx_driver.cpp src/khi_robot_client.cpp )

## Declare a C++ executable for ROS_CONTROL
add_executable(main src/main.cpp src/khi_robot_hardware_interface.cpp)

## Specify that we depend on msgs package
add_dependencies(khi_robot_client ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(khi_robot_client ${catkin_LIBRARIES} ${krnx_LIBRARY})
target_link_libraries(main ${catkin_LIBRARIES} khi_robot_client)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
