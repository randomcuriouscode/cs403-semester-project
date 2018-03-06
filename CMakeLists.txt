cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

ROSBUILD_INIT()

ROSBUILD_ADD_BOOST_DIRECTORIES()

include_directories(include)
add_compile_options(-std=c++11)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

ROSBUILD_GENMSG()
ROSBUILD_GENSRV()

SET(libs roslib roscpp)

add_library(kinect_mapper src/libs/kinect_mapper.cpp)
add_library(pathfinding src/libs/kinect_mapper.cpp)
add_library(tracking src/libs/pathfinding.cpp)

SET(usrlibs kinect_mapper pathfinding tracking)

ROSBUILD_ADD_EXECUTABLE(followbot src/follow_core.cpp)
TARGET_LINK_LIBRARIES(followbot ${libs} ${usrlibs})

rosbuild_add_gtest(test_example test/test_example.cpp)
TARGET_LINK_LIBRARIES(test_example ${libs} ${usrlibs})

ROSBUILD_LINK_BOOST(followbot system)
