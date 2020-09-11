cmake_minimum_required(VERSION 3.5.1)
project(elevation_mapping)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(
  CATKIN_PACKAGE_DEPENDENCIES 
    eigen_conversions
    grid_map_core
    grid_map_ros
    grid_map_msgs
    kindr
    kindr_ros
    message_filters
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf_conversions
)

find_package(catkin REQUIRED 
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${Eigen_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}_library
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

include_directories(
  include
  SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

###########
# Library #
###########

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

##############
# Executable #
##############

add_executable(${PROJECT_NAME}
  src/elevation_mapping_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_library
)

#############
## Install ##
#############

install(
  TARGETS 
    ${PROJECT_NAME}
    ${PROJECT_NAME}_library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY 
    include/${PROJECT_NAME}/
  DESTINATION 
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY 
    config
  DESTINATION 
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
      ${CATKIN_PACKAGE_DEPENDENCIES}
  )

  catkin_add_gtest(test_${PROJECT_NAME}
    test/ElevationMapTest.cpp
    test/test_elevation_mapping.cpp
    test/WeightedEmpiricalCumulativeDistributionFunctionTest.cpp
  )
endif()

if(TARGET test_${PROJECT_NAME})
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}_library
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
    SYSTEM
      ${catkin_INCLUDE_DIRS}
  )

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
      add_gtest_coverage(
        TEST_BUILD_TARGETS test_${PROJECT_NAME}
      )
  endif()
endif()

###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
