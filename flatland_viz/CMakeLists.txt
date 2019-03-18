cmake_minimum_required(VERSION 2.8.3)
project(flatland_viz)

# Get the LSB release name (14.04 or 16.04) string into variable ${LSB_RELEASE_VALUE}
FIND_PROGRAM(LSB_RELEASE_EXECUTABLE lsb_release)
EXECUTE_PROCESS(COMMAND ${LSB_RELEASE_EXECUTABLE} -s -r
          OUTPUT_VARIABLE LSB_RELEASE_VALUE
          OUTPUT_STRIP_TRAILING_WHITESPACE)

if(${LSB_RELEASE_VALUE} STREQUAL "14.04")
message("Skipping flatland_viz because it is incompatible with 14.04")
return()
endif()

# Ensure we're using c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rostest
  roscpp
  rviz
  flatland_server
  flatland_msgs
)

##############
## coverage ##
##############

set(COVERAGE "OFF" CACHE STRING "Enable coverage generation.")

message(STATUS "Using COVERAGE: ${COVERAGE}")
if("${COVERAGE}" STREQUAL "ON")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
endif()

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
catkin_package(
  INCLUDE_DIRS include ${OGRE_OV_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rviz flatland_server
)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${OGRE_OV_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

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
  src/pause_sim_tool.cpp
  include/flatland_viz/pause_sim_tool.h
)
add_dependencies(flatland_viz ${catkin_EXPORTED_TARGETS})

target_link_libraries(flatland_viz
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(flatland_viz_plugins
  src/load_model_dialog.cpp
  include/flatland_viz/load_model_dialog.h
  src/spawn_model_tool.cpp
  include/flatland_viz/spawn_model_tool.h
  src/pause_sim_tool.cpp
  include/flatland_viz/pause_sim_tool.h)

add_dependencies(flatland_viz_plugins ${catkin_EXPORTED_TARGETS})

target_link_libraries(flatland_viz_plugins
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS flatland_viz flatland_viz_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
   plugin_description.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

endif()
