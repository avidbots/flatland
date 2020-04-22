cmake_minimum_required(VERSION 3.5)

project(flatland_server)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(flatland_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# lua5.1
find_package(Lua 5.1 QUIET)

# OpenCV
find_package(OpenCV REQUIRED)

# Box2D
find_package(Box2D REQUIRED)

# Boost
find_package(Boost REQUIRED COMPONENTS date_time system thread filesystem)
find_package(Threads)

##############
## coverage ##
##############

#set(COVERAGE "OFF" CACHE STRING "Enable coverage generation.")

#message(STATUS "Using COVERAGE: ${COVERAGE}")
#if("${COVERAGE}" STREQUAL "ON")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
#endif()


###########
## Build ##
###########

## Flatland server library
add_library(flatland_lib SHARED
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

target_include_directories(flatland_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${LUA_INCLUDE_DIR})

ament_target_dependencies(flatland_lib rclcpp flatland_msgs yaml_cpp_vendor pluginlib geometry_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs visualization_msgs interactive_markers)

target_link_libraries(flatland_lib
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${LUA_LIBRARIES}
  ${BOX2D_LIBRARIES}
  yaml-cpp
)

## Declare a C++ executable
add_executable(flatland_server src/flatland_server_node.cpp)
ament_target_dependencies(flatland_server rclcpp flatland_msgs yaml_cpp_vendor pluginlib geometry_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs visualization_msgs interactive_markers)

target_include_directories(flatland_server
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${LUA_INCLUDE_DIR}
    ${yaml_cpp_vendor_INCLUDE_DIRS})

## Specify libraries to link a library or executable target against
target_link_libraries(flatland_server
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${LUA_LIBRARIES}
  ${BOX2D_LIBRARIES}
  yaml-cpp
  flatland_lib
)


ament_export_interfaces(export_flatland_lib HAS_LIBRARY_TARGET)
ament_export_libraries(flatland_lib)
ament_export_dependencies(rclcpp flatland_msgs yaml_cpp_vendor pluginlib geometry_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs)
ament_export_include_directories(include)
ament_package()

#############
## Install ##
#############

pluginlib_export_plugin_description_file(flatland_lib plugin_description.xml)

install(
  TARGETS flatland_lib
  EXPORT export_flatland_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Mark executables and/or libraries for installation
install(TARGETS flatland_server
  DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launchfiles
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY test/conestogo_office_test
    DESTINATION share/${PROJECT_NAME}/test
)

# TODO
# Install plugins file
#install(FILES flatland_plugins.xml
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############
if(FALSE) # fake a block comment
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
endif()
