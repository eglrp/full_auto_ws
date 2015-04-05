# Set up the ROS Catkin package settings.
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  cmake_modules
  message_generation
  geometry_msgs
)
#find_package(OpenCV REQUIRED)
#find_package(Eigen REQUIRED)

# Import the yaml-cpp libraries.
include(FindPkgConfig)
pkg_check_modules(Yaml REQUIRED yaml-cpp)

#add_message_files(DIRECTORY msg FILES
#   AprilTagDetection.msg
#   AprilTagDetections.msg
#)

generate_messages(DEPENDENCIES
    std_msgs
    geometry_msgs
)

# Set up the ROS Catkin package settings
catkin_package(
  INCLUDE_DIRS 
#include
 # CATKIN_DEPENDS std_msgs
  #               roscpp
   #              cmake_modules
    #             message_generation
#		 message_runtime
 #                geometry_msgs
        ../../../../../../../opt/ros/hydro/include

)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
    ${Yaml_INCLUDE_DIRS}
)

add_executable(ceres_control src/ceres_control.cpp)
target_link_libraries(ceres_control ${catkin_LIBRARIES})
target_link_libraries(ceres_control ${Yaml_LIBRARIES})
add_dependencies(ceres_control ${PROJECT_NAME}_gencpp)

install(TARGETS ceres_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)
