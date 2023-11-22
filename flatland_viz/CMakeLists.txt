cmake_minimum_required(VERSION 3.5)
project(flatland_viz)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)  
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(std_srvs REQUIRED)
find_package(flatland_server REQUIRED)
find_package(flatland_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)

# Boost
find_package(Boost REQUIRED COMPONENTS date_time system filesystem)

##############
## coverage ##
##############

#set(COVERAGE "OFF" CACHE STRING "Enable coverage generation.")

#message(STATUS "Using COVERAGE: ${COVERAGE}")
#if("${COVERAGE}" STREQUAL "ON")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
#endif()

########################
## Ogre Include Stuff ##
########################

find_package(PkgConfig REQUIRED)

pkg_check_modules(OGRE_OV OGRE OGRE-Overlay)

# Old versions of OGRE (pre 1.9) included OGRE-Overlay in the main package
# (i.e. there was no OGRE-Overlay component).  So if the above
# pkg_check_modules() failed, try looking for just OGRE.
if(NOT OGRE_OV_FOUND)
  pkg_check_modules(OGRE_OV REQUIRED OGRE)
endif(NOT OGRE_OV_FOUND)

###################################
## catkin specific configuration ##
###################################
#catkin_package(
#  INCLUDE_DIRS include ${OGRE_OV_INCLUDE_DIRS}
#  CATKIN_DEPENDS roscpp rviz flatland_server
#)


###########
## Build ##
###########

include_directories(include ${OGRE_OV_INCLUDE_DIRS})



#link_directories(${catkin_LIBRARY_DIRS})

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
set(QT_LIBRARIES Qt5::Widgets)

add_definitions(-DQT_NO_KEYWORDS)

## Declare a C++ executable
add_executable(flatland_viz
  src/flatland_viz_node.cpp
  src/flatland_viz.cpp
  include/flatland_viz/flatland_viz.h
  src/flatland_window.cpp
  include/flatland_viz/flatland_window.h
  src/load_model_dialog.cpp
  include/flatland_viz/load_model_dialog.h
  src/spawn_model_tool.cpp
  include/flatland_viz/spawn_model_tool.h
  src/load_model_dialog.cpp
  include/flatland_viz/load_model_dialog.h
  #src/pause_sim_tool.cpp
  #include/flatland_viz/pause_sim_tool.h
)

target_include_directories(flatland_viz
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>    
    ${OGRE_OV_INCLUDE_DIRS})


#add_dependencies(flatland_viz ${catkin_EXPORTED_TARGETS})

ament_target_dependencies(flatland_viz rclcpp flatland_msgs flatland_server visualization_msgs interactive_markers rviz_common rviz_rendering rviz_default_plugins std_srvs)

target_link_libraries(flatland_viz
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
)

add_library(flatland_viz_plugins
  src/load_model_dialog.cpp
  include/flatland_viz/load_model_dialog.h
  src/spawn_model_tool.cpp
  include/flatland_viz/spawn_model_tool.h
  src/load_model_dialog.cpp
  include/flatland_viz/load_model_dialog.h
  #src/pause_sim_tool.cpp
  #include/flatland_viz/pause_sim_tool.h
  )

target_include_directories(flatland_viz_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OGRE_OV_INCLUDE_DIRS})

#add_dependencies(flatland_viz_plugins ${catkin_EXPORTED_TARGETS})
ament_target_dependencies(flatland_viz_plugins rclcpp flatland_msgs flatland_server rviz_common rviz_rendering rviz_default_plugins std_srvs)

target_link_libraries(flatland_viz_plugins
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
)

ament_package()

#############
## Install ##
#############

pluginlib_export_plugin_description_file(flatland_viz plugin_description.xml)

# Mark executables and/or libraries for installation
install(TARGETS flatland_viz flatland_viz_plugins
  DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include
)

#############
## Testing ##
#############

if(BUILD_TESTING )

endif()
