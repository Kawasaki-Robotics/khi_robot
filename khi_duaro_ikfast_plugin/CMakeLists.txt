cmake_minimum_required(VERSION 2.8.12)
project(khi_duaro_ikfast_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
)

include_directories(include)

set(IKFAST_LIBRARY_NAME_L duaro_lower_arm_moveit_ikfast_plugin)
set(IKFAST_LIBRARY_NAME_U duaro_upper_arm_moveit_ikfast_plugin)

find_package(LAPACK REQUIRED)

add_library(${IKFAST_LIBRARY_NAME_L} src/duaro_lower_arm_ikfast_moveit_plugin.cpp)
add_library(${IKFAST_LIBRARY_NAME_U} src/duaro_upper_arm_ikfast_moveit_plugin.cpp)
target_link_libraries(${IKFAST_LIBRARY_NAME_L} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})
target_link_libraries(${IKFAST_LIBRARY_NAME_U} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

install(TARGETS ${IKFAST_LIBRARY_NAME_L} ${IKFAST_LIBRARY_NAME_U} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
  duaro_lower_arm_moveit_ikfast_plugin_description.xml
  duaro_upper_arm_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)