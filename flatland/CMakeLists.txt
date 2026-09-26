cmake_minimum_required(VERSION 3.10)

project(flatland NONE)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  endif()
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


