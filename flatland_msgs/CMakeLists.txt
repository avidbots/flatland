cmake_minimum_required(VERSION 3.0.2)
project(flatland_msgs)

# Ensure we're using c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)

################
## Build MSGS ##
################
add_message_files(FILES
  DebugTopicList.msg
  Collision.msg
  Collisions.msg
  Vector2.msg
)

add_service_files(FILES
  SpawnModel.srv
  DeleteModel.srv
  MoveModel.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
