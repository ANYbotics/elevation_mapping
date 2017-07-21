cmake_minimum_required(VERSION 2.8.3)
project(elevation_mapping_demos)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  elevation_mapping
  grid_map_rviz_plugin
  rviz
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES
  CATKIN_DEPENDS
    elevation_mapping
    grid_map_rviz_plugin
    rviz
  #DEPENDS
)

#############
## Install ##
#############

install(DIRECTORY config doc launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/ground_truth_pose_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
