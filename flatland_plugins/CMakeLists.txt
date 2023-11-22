cmake_minimum_required(VERSION 3.5)

project(flatland_plugins)

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()


## Find macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(flatland_server REQUIRED)
find_package(flatland_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(OpenCV REQUIRED)
#  rostest


find_package(Eigen3 REQUIRED)# lua5.1
find_package(Lua 5.1 QUIET)

find_package(Boost REQUIRED COMPONENTS date_time system filesystem)

##############
## coverage ##
##############

# set(COVERAGE "OFF" CACHE STRING "Enable coverage generation.")

# message(STATUS "Using COVERAGE: ${COVERAGE}")
# if("${COVERAGE}" STREQUAL "ON")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
# endif()


###########
## Build ##
###########

# Declare a C++ library
add_library(flatland_plugins_lib SHARED
  src/laser.cpp
  src/tricycle_drive.cpp
  src/diff_drive.cpp
  src/model_tf_publisher.cpp
  src/update_timer.cpp
  src/bumper.cpp
  src/tween.cpp
  src/bool_sensor.cpp
  src/world_modifier.cpp
  src/world_random_wall.cpp
  src/gps.cpp
)

ament_target_dependencies(flatland_plugins_lib rclcpp flatland_server flatland_msgs pluginlib std_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs yaml_cpp_vendor)

target_include_directories(flatland_plugins_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/thirdparty/tweeny>
  $<INSTALL_INTERFACE:include/thirdparty/tweeny>
  ${Eigen3_INCLUDE_DIRS}
  ${LUA_INCLUDE_DIR}
  ${yaml_cpp_vendor_INCLUDE_DIRS})


target_link_libraries(flatland_plugins_lib
  ${Eigen3_LIBRARIES}
  ${LUA_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

ament_export_targets(export_flatland_plugins_lib HAS_LIBRARY_TARGET)
# ament_export_libraries(flatland_lib)
ament_export_dependencies(rclcpp flatland_server flatland_msgs pluginlib geometry_msgs tf2 tf2_ros tf2_geometry_msgs yaml_cpp_vendor)

#############
## Install ##
#############

pluginlib_export_plugin_description_file(flatland_server plugin_description.xml)

install(
  TARGETS flatland_plugins_lib
  EXPORT export_flatland_plugins_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_flatland_plugins.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(diff_drive_test test/diff_drive_test.cpp)
  target_link_libraries(diff_drive_test flatland_plugins_lib)

  ament_add_gtest(tricycle_drive_test test/tricycle_drive_test.cpp)
  target_link_libraries(tricycle_drive_test flatland_plugins_lib)

  ament_add_gtest(model_tf_publisher_test test/model_tf_publisher_test.cpp)
  target_link_libraries(model_tf_publisher_test flatland_plugins_lib)

  ament_add_gtest(tween_test test/tween_test.cpp)
  target_link_libraries(tween_test flatland_plugins_lib)

  ament_add_gtest(gps_test test/gps_test.cpp)
  target_link_libraries(gps_test flatland_plugins_lib)

  ament_add_gtest(bumper_test test/bumper_test.cpp)
  target_link_libraries(bumper_test flatland_plugins_lib)

  ament_add_gtest(laser_test test/laser_test.cpp)
  target_link_libraries(laser_test flatland_plugins_lib)

  ament_add_gtest(update_timer_test test/update_timer_test.cpp)
  target_link_libraries(update_timer_test flatland_plugins_lib)
endif()

ament_package()
