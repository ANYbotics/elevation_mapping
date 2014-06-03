cmake_minimum_required(VERSION 2.8.3)
project(elevation_map_msg)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ElevationMap.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetSubmap.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
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
    INCLUDE_DIRS include
    LIBRARIES transformation_math eigen_conversions elevation_map_msg_helpers
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
    DEPENDS Eigen Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library

# Transformation math
add_library(transformation_math
   src/TransformationMath.cpp
)

# Eigen conversions
add_library(eigen_conversions
   src/EigenConversions.cpp
)

target_link_libraries(eigen_conversions
  ${catkin_LIBRARIES}
)

add_dependencies(eigen_conversions
    elevation_map_msg_helpers
)

# Elevation map msg helpers
add_library(elevation_map_msg_helpers
   src/ElevationMapMsgHelpers.cpp
)

target_link_libraries(elevation_map_msg_helpers
  transformation_math
  ${catkin_LIBRARIES}
)

add_dependencies(elevation_map_msg_helpers
    transformation_math
    ${PROJECT_NAME}_generate_messages_cpp
)

#############
## Testing ##
#############

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/test_elevation_map_msg.cpp test/TransformationMathTest.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test transformation_math)
endif()
