cmake_minimum_required(VERSION 2.8.3)
project(khi_robot_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    rospy
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/khi_robot_control/test_khi_robot_control_duaro.xml)
  add_rostest(tests/khi_robot_control/test_khi_robot_control_rs007n.xml)
  add_rostest(tests/khi_robot_control/test_khi_robot_control_rs007l.xml)
  add_rostest(tests/khi_robot_control/test_khi_robot_control_rs013n.xml)
  add_rostest(tests/khi_robot_control/test_khi_robot_control_rs080n.xml)
endif()
