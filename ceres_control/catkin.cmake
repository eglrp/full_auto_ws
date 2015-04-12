# Set up the ROS Catkin package settings.
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  cmake_modules
  message_generation
  geometry_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-pthread ${CMAKE_CXX_FLAGS}")

add_message_files(DIRECTORY msg FILES
  DetectionStats.msg
  ChangeState.msg
)

generate_messages(DEPENDENCIES
    std_msgs
    geometry_msgs
)

# Set up the ROS Catkin package settings
catkin_package(
  INCLUDE_DIRS 
   include
   CATKIN_DEPENDS std_msgs
   roscpp
   cmake_modules
   message_generation
		message_runtime
   geometry_msgs
   # ../../../../../../../opt/ros/indigo/include

)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
)

add_executable(ceres_control src/ceres_control_fa.cpp)
target_link_libraries(ceres_control ${catkin_LIBRARIES})
target_link_libraries(ceres_control ${Yaml_LIBRARIES})
add_dependencies(ceres_control ${PROJECT_NAME}_gencpp)

install(TARGETS ceres_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)
