cmake_minimum_required(VERSION 2.8.3)
project(elevation_mapping)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  grid_map
  grid_map_core
  grid_map_msgs
  pcl_ros
  roscpp
  message_filters
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  eigen_conversions
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)
find_package(PkgConfig REQUIRED)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  pkg_check_modules(kindr kindr REQUIRED)
endif()

pkg_check_modules(SM_COMMON schweizer-messer-common REQUIRED)
pkg_check_modules(SM_TIMING schweizer-messer-timing REQUIRED)


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
    INCLUDE_DIRS include
#  LIBRARIES elevation_mapping
    CATKIN_DEPENDS std_msgs sensor_msgs grid_map_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Kindr_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${kindr_INCLUDE_DIRS}
  ${SM_COMMON_INCLUDE_DIRS}
  ${SM_TIMING_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(elevation_mapping
#   src/${PROJECT_NAME}/elevation_mapping.cpp
# )

## Declare a cpp executable
add_executable(
  ${PROJECT_NAME}
  src/elevation_mapping_node.cpp
  src/ElevationMapping.cpp
  src/ElevationMap.cpp
  src/RobotMotionMapUpdater.cpp
  src/sensor_processors/SensorProcessorBase.cpp
  src/sensor_processors/KinectSensorProcessor.cpp
  src/sensor_processors/StereoSensorProcessor.cpp
  src/sensor_processors/LaserSensorProcessor.cpp
  src/sensor_processors/PerfectSensorProcessor.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${SM_COMMON_LIBRARIES}
  ${SM_TIMING_LIBRARIES}
)
