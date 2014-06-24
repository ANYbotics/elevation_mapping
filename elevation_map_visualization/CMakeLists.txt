cmake_minimum_required(VERSION 2.8.3)
project(elevation_map_visualization)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  elevation_map_msg
  visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)

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
#  LIBRARIES elevation_map_visualization
  CATKIN_DEPENDS elevation_map_msg visualization_msgs
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
)

## Declare a cpp library
# add_library(elevation_map_visualization
#   src/${PROJECT_NAME}/elevation_map_visualization.cpp
# )

## Declare a cpp executable
add_executable(
	${PROJECT_NAME}
	src/elevation_map_visualization_node.cpp
	src/ElevationMapVisualization.cpp
	src/ElevationMapVisualizationHelpers.cpp
	src/visualizations/VisualizationBase.cpp
	src/visualizations/ElevationVisualization.cpp
	src/visualizations/VarianceVisualization.cpp
	src/visualizations/MapRegionVisualization.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(elevation_map_visualization_node elevation_map_visualization_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_elevation_map_visualization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)