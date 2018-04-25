cmake_minimum_required(VERSION 2.8.11)

##########################################################################
### Third party libraries
##########################################################################

set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /usr/local/lib/)
set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} /usr/local/include)

find_package(  yaml-cpp REQUIRED )

add_custom_target(mytargetname ALL 
		COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/bin/test)

##########################################################################
### ROSBUILD STUFF
##########################################################################

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
SET(otherlibs yaml-cpp)

ROSBUILD_ADD_EXECUTABLE(followbot src/follow_core.cpp)
TARGET_LINK_LIBRARIES(followbot ${libs} ${usrlibs} ${otherlibs})

rosbuild_add_gtest(test/test_example test/test_example.cpp)
TARGET_LINK_LIBRARIES(test/test_example ${libs} ${usrlibs} ${otherlibs})

rosbuild_add_gtest(test/test_kinectmapper test/test_kinectmapper.cpp)
TARGET_LINK_LIBRARIES(test/test_kinectmapper ${libs} ${usrlibs} ${otherlibs})

rosbuild_add_gtest(test/test_pathfinding test/test_pathfinding.cpp)
TARGET_LINK_LIBRARIES(test/test_pathfinding ${libs} ${usrlibs} ${otherlibs})

ROSBUILD_LINK_BOOST(followbot system)
