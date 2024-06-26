cmake_minimum_required(VERSION 3.0.2)
project(alan_state_estimation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin COMPONENTS
  nodelet
  roscpp
  std_msgs
  roslaunch
  geometry_msgs
  mavros_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
  genmsg
  visualization_msgs
  tf
)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

roslaunch_add_file_check(launch)


add_message_files(
  FILES
  alan_log.msg
)


generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES offb
  CATKIN_DEPENDS 
  geometry_msgs 
  mavros_msgs 
  roscpp
  std_msgs
  message_generation
)

set(Sophus_LIBRARIES libSophus.so)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

add_library(alan
  src/aruco.cpp
  src/yolo.cpp
  src/led.cpp
  src/msgsync.cpp
  src/tools/RosTopicConfigs.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(
  alan
  ${catkin_LIBRARIES}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)
add_dependencies(
  alan 
  alan_state_estimation_generate_messages_cpp
  alan_landing_planning_generate_messages_cpp
  alan_visualization_generate_messages_cpp
)

add_executable(calibrate
  src/tools/calibrate.cpp
)

target_link_libraries(calibrate
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(gazebo_uav
  src/tools/gazebo_uav.cpp
)

target_link_libraries(gazebo_uav
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)





