cmake_minimum_required(VERSION 3.10)

project(flatland_controls_plugins)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(flatland_server REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(flatland_controls_plugins_lib SHARED
  src/drive_wheel.cpp
  src/steering_motor.cpp
)

target_include_directories(flatland_controls_plugins_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(flatland_controls_plugins_lib PUBLIC
  flatland_server::flatland_lib
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${control_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

ament_export_targets(export_flatland_controls_plugins_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(control_msgs flatland_server interactive_markers pluginlib rclcpp sensor_msgs)

pluginlib_export_plugin_description_file(flatland_server plugin_description.xml)

install(
  TARGETS flatland_controls_plugins_lib
  EXPORT export_flatland_controls_plugins_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(OpenCV REQUIRED)
  ament_add_gtest(joint_actuator_test test/joint_actuator_test.cpp)
  target_include_directories(joint_actuator_test PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(joint_actuator_test flatland_controls_plugins_lib)
endif()

ament_package()