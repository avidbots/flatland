cmake_minimum_required(VERSION 2.8.3)
project(flatland_server)

# Ensure we're using c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rostest
  pluginlib
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  interactive_markers
  flatland_msgs
)

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

# lua5.1
find_package(Lua 5.1 QUIET)

# OpenCV
find_package(OpenCV REQUIRED)

# Boost
find_package(Boost REQUIRED COMPONENTS date_time system thread)
find_package(Threads)

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
  INCLUDE_DIRS include thirdparty
  LIBRARIES flatland_lib flatland_Box2D
  CATKIN_DEPENDS pluginlib roscpp std_msgs tf2 visualization_msgs tf2_geometry_msgs geometry_msgs
  DEPENDS OpenCV YAML_CPP
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${LUA_INCLUDE_DIR}
  thirdparty
)

add_subdirectory("thirdparty/Box2D")

## Flatland server library
add_library(flatland_lib
  src/simulation_manager.cpp
  src/world.cpp
  src/layer.cpp
  src/model.cpp
  src/entity.cpp
  src/debug_visualization.cpp
  src/geometry.cpp
  src/body.cpp
  src/joint.cpp
  src/model_body.cpp
  src/collision_filter_registry.cpp
  src/model_plugin.cpp
  src/world_plugin.cpp
  src/plugin_manager.cpp
  src/interactive_marker_manager.cpp
  src/timekeeper.cpp
  src/service_manager.cpp
  src/yaml_reader.cpp
  src/dummy_model_plugin.cpp 
  src/dummy_world_plugin.cpp
  src/yaml_preprocessor.cpp
)

add_dependencies(flatland_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(flatland_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LUA_LIBRARIES}
  flatland_Box2D
  yaml-cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(flatland_server_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(flatland_server src/flatland_server_node.cpp)
add_dependencies(flatland_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(flatland_server
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LUA_LIBRARIES}
  flatland_Box2D
  yaml-cpp
  flatland_lib
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
install(TARGETS flatland_server flatland_lib
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

install(DIRECTORY test/conestogo_office_test
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)

# Install plugins file
install(FILES flatland_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(null_test test/null.cpp)
  
  add_rostest_gtest(load_world_test
    test/load_world_test.test
    test/load_world_test.cpp)
  target_link_libraries(load_world_test
    flatland_lib)

  add_rostest_gtest(model_test test/model_test.test test/model_test.cpp)
  target_link_libraries(model_test flatland_lib)

  catkin_add_gtest(geometry_test test/geometry_test.cpp)
  target_link_libraries(geometry_test
    flatland_lib)

  catkin_add_gtest(collision_filter_registry_test
    test/collision_filter_registry_test.cpp)
  target_link_libraries(collision_filter_registry_test
    flatland_lib)

  add_rostest_gtest(plugin_manager_test
    test/plugin_manager_test.test
    test/plugin_manager_test.cpp)
  target_link_libraries(plugin_manager_test
    flatland_lib)

  add_rostest_gtest(service_manager_test
    test/service_manager_test.test
    test/service_manager_test.cpp)
  target_link_libraries(service_manager_test
    flatland_lib)


  add_rostest_gtest(debug_visualization_test
    test/debug_visualization.test
    test/debug_visualization_test.cpp)
  target_link_libraries(debug_visualization_test
    flatland_lib)

  add_rostest_gtest(dummy_model_plugin_test 
                    test/dummy_model_plugin_test.test 
                    test/dummy_model_plugin_test.cpp) 
  target_link_libraries(dummy_model_plugin_test
                        flatland_lib) 
                        
  add_rostest_gtest(dummy_world_plugin_test 
                    test/dummy_world_plugin_test.test 
                    test/dummy_world_plugin_test.cpp) 
  target_link_libraries(dummy_world_plugin_test
                        flatland_lib yaml-cpp) 

  add_rostest_gtest(yaml_preprocessor_test
                    test/yaml_preprocessor/yaml_preprocessor_test.test 
                    test/yaml_preprocessor/yaml_preprocessor_test.cpp) 
  target_link_libraries(yaml_preprocessor_test
                        flatland_lib yaml-cpp)


endif()
