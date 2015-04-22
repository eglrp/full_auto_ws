# Set up the ROS Catkin package settings.
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  std_msgs
  sensor_msgs
  image_transport
  roscpp
  cmake_modules
  message_generation
  geometry_msgs
)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-pthread ${CMAKE_CXX_FLAGS}")

add_message_files(DIRECTORY msg FILES
    AprilTagDetection.msg
    AprilTagDetections.msg
    DetectionStats.msg
)

add_service_files(FILES
  EnableZoh.srv
)


generate_messages(DEPENDENCIES
    std_msgs
    geometry_msgs
)

# Set up the ROS Catkin package settings
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge
                 std_msgs
                 sensor_msgs
                 image_transport
                 roscpp
                 cmake_modules
                 message_generation
                 geometry_msgs
)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
link_directories(
	 lib/
)

add_executable(ceresgrid-ekf src/ceresgrid-ekf.cpp)
target_link_libraries(ceresgrid-ekf ${catkin_LIBRARIES})
target_link_libraries(ceresgrid-ekf ${Eigen_LIBRARIES})
target_link_libraries(ceresgrid-ekf ${OpenCV_LIBRARIES})
target_link_libraries(ceresgrid-ekf libapriltags.a)
add_dependencies(ceresgrid-ekf ${PROJECT_NAME}_gencpp)

install(TARGETS ceresgrid-ekf
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)
