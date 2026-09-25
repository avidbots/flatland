cmake_minimum_required(VERSION 3.10)
project(flatland_viz)

# C++23 is the lowest common denominator; Lyrical and newer can use C++26
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(flatland_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt6 QUIET COMPONENTS Widgets)
if(NOT Qt6_FOUND)
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
endif()

add_executable(flatland_viz
  src/flatland_viz_node.cpp
  src/flatland_viz.cpp
  include/flatland_viz/flatland_viz.h
)
target_include_directories(flatland_viz PRIVATE include)
target_link_libraries(flatland_viz
  Qt::Widgets
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  ${flatland_msgs_TARGETS}
)

install(TARGETS flatland_viz
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

