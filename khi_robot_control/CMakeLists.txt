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
message(STATUS "CMAKE_SYSTEM_NAME : ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMAKE_SYSTEM_PROCESSOR : ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "CMAKE_HOST_SYSTEM_NAME : ${CMAKE_HOST_SYSTEM_NAME}")
message(STATUS "CMAKE_HOST_SYSTEM_PROCESSOR : ${CMAKE_HOST_SYSTEM_PROCESSOR}")
message(STATUS "CMAKE_LIBRARY_ARCHITECTURE : ${CMAKE_LIBRARY_ARCHITECTURE}")
if( ${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*x86_64.*" )
  set(krnx_lib_name krnx-x86_64)
elseif( ${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*aarch64.*" )
  set(krnx_lib_name krnx-aarch64)
elseif( ${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*i386.*" )
  set(krnx_lib_name krnx-i386)
elseif( ${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*arm.*gnueabihf" )
  set(krnx_lib_name krnx-armhf)
elseif( ${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*arm.*gnueabi" )
  set(krnx_lib_name krnx-armel)
else()
  message(WARNING "\nCould not detect system processor architecture: assuming 'x86_64'. This may result in non-working binaries.")
  set(krnx_lib_name krnx-x86_64)
endif()

find_library(krnx_LIBRARIES lib${krnx_lib_name}.so PATHS ${PROJECT_SOURCE_DIR}/lib)
message(STATUS "Found KRNX library at: ${krnx_LIBRARIES}")

## Specify C++11
add_definitions(-std=c++11)

## Declare a C++ library
add_library(khi_robot_client src/khi_robot_client.cpp src/khi_robot_krnx_driver.cpp)

## Declare a C++ executable for ROS_CONTROL
add_executable(main src/main.cpp src/khi_robot_hardware_interface.cpp)

## Specify that we depend on msgs package
add_dependencies(khi_robot_client ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(khi_robot_client ${catkin_LIBRARIES} ${krnx_LIBRARIES})
target_link_libraries(main ${catkin_LIBRARIES} khi_robot_client)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/test_main.xml)
endif()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES ${krnx_LIBRARIES}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS main khi_robot_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
