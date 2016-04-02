# Minimum cmake version
cmake_minimum_required(VERSION 2.8.3)

# Create project
project(elevation_change_msgs)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    geometry_msgs
    traversability_msgs
    message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    Obstacle.msg
    ObstacleResult.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    DetectObstacle.srv
    Toggle.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    traversability_msgs
)

# Generate a package
catkin_package(
  CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    traversability_msgs
    message_runtime
)