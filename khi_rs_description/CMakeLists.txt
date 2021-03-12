cmake_minimum_required(VERSION 2.8.3)
project(khi_rs_description)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test_rs007l.xml)
  roslaunch_add_file_check(tests/roslaunch_test_rs007n.xml)
  roslaunch_add_file_check(tests/roslaunch_test_rs013n.xml)
  roslaunch_add_file_check(tests/roslaunch_test_rs080n.xml)
endif()

install(DIRECTORY launch config meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
