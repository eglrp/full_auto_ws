# Set up the ROS Catkin package settings.
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  cmake_modules
  message_generation
  geometry_msgs
)

# add_message_files(DIRECTORY msg FILES

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
)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
)

add_executable(ceres_logger src/ceres_logger.cpp)
target_link_libraries(ceres_logger ${catkin_LIBRARIES})
add_dependencies(ceres_logger ${PROJECT_NAME}_gencpp)

install(TARGETS cereslogger
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)
