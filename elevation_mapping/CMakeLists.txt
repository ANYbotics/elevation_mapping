cmake_minimum_required(VERSION 2.8.3)
project(elevation_mapping)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  grid_map_core
  grid_map_ros
  grid_map_msgs
  pcl_ros
  roscpp
  message_filters
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  eigen_conversions
  kindr
  kindr_ros
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

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
  INCLUDE_DIRS
    include
    ${Eigen_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}_library
  CATKIN_DEPENDS
    grid_map_core
    grid_map_ros
    grid_map_msgs
    pcl_ros
    roscpp
    message_filters
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    eigen_conversions
    kindr
    kindr_ros
  DEPENDS
    Boost
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
  ${Eigen_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_library
  src/ElevationMapping.cpp
  src/ElevationMap.cpp
  src/RobotMotionMapUpdater.cpp
  src/sensor_processors/SensorProcessorBase.cpp
  src/sensor_processors/StructuredLightSensorProcessor.cpp
  src/sensor_processors/StereoSensorProcessor.cpp
  src/sensor_processors/LaserSensorProcessor.cpp
  src/sensor_processors/PerfectSensorProcessor.cpp
)

target_link_libraries(${PROJECT_NAME}_library
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}
  src/elevation_mapping_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_library
)

#############
## Testing ##
#############

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/test_elevation_mapping.cpp
  test/ElevationMapTest.cpp
  test/WeightedEmpiricalCumulativeDistributionFunctionTest.cpp
)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_library)
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_library ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
