# Set up the ROS Catkin package settings.
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  cmake_modules
  message_generation
  geometry_msgs
)

# add_message_files(DIRECTORY msg FILES
   # AprilTagDetection.msg
   # AprilTagDetections.msg
# )

# generate_messages(DEPENDENCIES
#     std_msgs
#     geometry_msgs
# )

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

add_executable(cereslogger src/ceres_logger.cpp)
target_link_libraries(cereslogger ${catkin_LIBRARIES})
add_dependencies(cereslogger ${PROJECT_NAME}_gencpp)

install(TARGETS cereslogger
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)
