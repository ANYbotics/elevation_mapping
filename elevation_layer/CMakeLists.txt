# use c++11
set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

cmake_minimum_required(VERSION 2.8.3)
project(elevation_layer)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  costmap_2d
  dynamic_reconfigure
  grid_map_ros
)

# add dynamic reconfigure configs
generate_dynamic_reconfigure_options(
  cfg/ElevationPlugin.cfg
)

## declare catkin package
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    elevation_layer
  CATKIN_DEPENDS
    roscpp
    tf
    costmap_2d
    dynamic_reconfigure
    grid_map_ros
)



## build ##

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## declare a cpp library
add_library(elevation_layer
  src/elevation_layer.cpp
)


## cmake target dependencies of the executable/library

# build config headers
add_dependencies(elevation_layer elevation_layer_gencfg)

## libraries to link a library or executable target against
target_link_libraries(elevation_layer
  ${catkin_LIBRARIES}
)



## install ##

## executables and/or libraries for installation
install(TARGETS elevation_layer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## cpp-header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
