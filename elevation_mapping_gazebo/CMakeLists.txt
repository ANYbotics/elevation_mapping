cmake_minimum_required(VERSION 2.8.3)
project(elevation_mapping_gazebo)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    gazebo_plugins
    gazebo_ros
    roscpp
    urdf
    xacro
)

find_package(gazebo REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES
  CATKIN_DEPENDS
    gazebo_plugins
    gazebo_ros
    roscpp
    urdf
    xacro
  DEPENDS
    gazebo
)

# Create symlink for Gazebo to find the models.
execute_process(
  COMMAND cmake -E create_symlink ${CMAKE_SOURCE_DIR}/models
    ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/models
)

execute_process(
  COMMAND cmake -E create_symlink ${CMAKE_SOURCE_DIR}/meshes
    ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/meshes
)

add_library(odometry_motion_gazebo_ros_control
  src/OdometryMotionGazeboRosControl.cpp
)
add_dependencies(odometry_motion_gazebo_ros_control
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(odometry_motion_gazebo_ros_control
  ${catkin_LIBRARIES}
)

install(
  TARGETS odometry_motion_gazebo_ros_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/odometry_motion_ros_control/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(
  DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
