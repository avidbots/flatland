cmake_minimum_required(VERSION 2.8.3)
project(flatland_plugins)

# Ensure we're using c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rostest
  pluginlib
  roscpp
  flatland_server
)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)

# yaml-cpp
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

##############
## coverage ##
##############

set(COVERAGE "OFF" CACHE STRING "Enable coverage generation.")

message(STATUS "Using COVERAGE: ${COVERAGE}")
if("${COVERAGE}" STREQUAL "ON")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
endif()


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES flatland_plugins_lib
  CATKIN_DEPENDS pluginlib roscpp flatland_server
  DEPENDS YAML_CPP Eigen3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(flatland_plugins_lib
  src/laser.cpp
  src/dummy_model_plugin.cpp
  src/model_tf_publisher.cpp
  src/update_timer.cpp
)

add_dependencies(flatland_plugins_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(flatland_plugins_lib
  ${catkin_LIBRARIES}
  yaml-cpp
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark executables and/or libraries for installation
install(TARGETS flatland_plugins_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Install launchfiles
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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

if(CATKIN_ENABLE_TESTING)

  add_rostest_gtest(laser_test test/laser_test.test
                    test/laser_test.cpp)
  target_link_libraries(laser_test flatland_plugins_lib flatland_lib)

  add_rostest_gtest(dummy_model_plugin_test test/dummy_model_plugin_test.test
                    test/dummy_model_plugin_test.cpp)
  target_link_libraries(dummy_model_plugin_test flatland_plugins_lib flatland_lib)

endif()
