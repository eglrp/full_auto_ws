# This CMakeLists.txt file is designed to correctly switch two CMakeLists,
# one for rosbuild and one for catkin.
#
# To use this file, create two CMakeLists:
#   - catkin.cmake : the CMakeLists used in ROS catkin builds
#   - rosbuild.cmake : the CMakeLists used for ROS rosbuild builds
#
cmake_minimum_required(VERSION 2.8.3)
project(cerestags)
include(${PROJECT_SOURCE_DIR}/catkin.cmake)
