cmake_minimum_required(VERSION 2.8.3)
project(elevation_mapping_costmap_2d_plugin)

## Use C++11
add_definitions(-std=c++11)
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
#add_definitions(-std=c++11 -Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  costmap_2d
  dynamic_reconfigure
  grid_map_ros
  filters
)

# Find system libraries
find_package(Boost REQUIRED)

# add dynamic reconfigure configs
generate_dynamic_reconfigure_options(
  cfg/ElevationPlugin.cfg
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
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    tf
    costmap_2d
    dynamic_reconfigure
    grid_map_ros
    filters
  DEPENDS
    Boost
)

## build ##

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

## declare a cpp library
add_library(${PROJECT_NAME}
  src/elevation_layer.cpp
)


## cmake target dependencies of the executable/library

# build config headers
add_dependencies(${PROJECT_NAME} elevation_mapping_costmap_2d_plugin_gencfg)

## libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


## install ##

## executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## cpp-header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
