cmake_minimum_required(VERSION 3.10)
project(flatland_box2d)

# C++23 is the lowest common denominator; Lyrical and newer can use C++26
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

find_package(ament_cmake REQUIRED)

include(FetchContent)
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build Box2D as one shared runtime" FORCE)
set(BOX2D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX2D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(box2d
  GIT_REPOSITORY https://github.com/erincatto/box2d.git
  GIT_TAG 8c661469c9507d3ad6fbd2fea3f1aa71669c2fe3 # v3.1.1
  PATCH_COMMAND
    ${CMAKE_COMMAND}
      -DBOX2D_SOURCE_DIR=<SOURCE_DIR>
      -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/set_box2d_linear_slop.cmake"
)
FetchContent_MakeAvailable(box2d)

add_library(flatland_box2d_lib INTERFACE)
target_link_libraries(flatland_box2d_lib INTERFACE box2d)

ament_export_targets(export_flatland_box2d HAS_LIBRARY_TARGET)

ament_package(CONFIG_EXTRAS cmake/box2d-target.cmake)

install( 
  TARGETS flatland_box2d_lib
  EXPORT export_flatland_box2d
)