cmake_minimum_required(VERSION 3.10)
project(flatland_ros2_control_examples)

add_compile_options(-Werror)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY models worlds launch config
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/twist_to_joint_commands.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(interactive_markers REQUIRED)
  find_package(flatland_controls_plugins REQUIRED)
  find_package(flatland_server REQUIRED)
  find_package(flatland_plugins REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(OpenCV REQUIRED)
  ament_add_gtest(worlds_test test/worlds_test.cpp)
  target_include_directories(worlds_test PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(worlds_test flatland_server::flatland_lib
    flatland_controls_plugins::flatland_controls_plugins_lib rclcpp::rclcpp)

  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME controller_config_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/controller_config_test.py)
endif()

ament_package()