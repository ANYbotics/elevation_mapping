cmake_minimum_required(VERSION 3.5.1)
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

install(DIRECTORY config doc launch rviz sample_data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/tf_to_pose_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##############
##   Test   ##
##############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
    roslaunch
    )
  catkin_add_gtest(test_${PROJECT_NAME}
    test/empty_test.cpp
  )

  target_include_directories(test_${PROJECT_NAME}
    PRIVATE
      include
    SYSTEM PUBLIC
      ${catkin_INCLUDE_DIRS}
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${catkin_LIBRARIES}
    gtest_main
  )
  roslaunch_add_file_check(launch)

  ###################
  ## Code_coverage ##
  ###################
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        test_${PROJECT_NAME}
    )
  endif()
endif()