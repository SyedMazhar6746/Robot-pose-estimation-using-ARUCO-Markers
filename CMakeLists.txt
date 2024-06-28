cmake_minimum_required(VERSION 2.8.3)
project(pose_estimation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++20)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  tf2_ros
  geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

## Uncomment if using the Eigen3 package from ROS (not necessary if using system Eigen3)
# find_package(Eigen3 REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(image_sub src/image_sub.cpp src/image_callback.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(image_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(image_sub
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS image_sub
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/image_sub.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
