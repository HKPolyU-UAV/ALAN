cmake_minimum_required(VERSION 3.0.2)
project(alan_ugv)

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
)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

roslaunch_add_file_check(launch)


add_message_files(
  FILES
  alan_ugv_blank.msg
)


generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  #  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  geometry_msgs 
  mavros_msgs 
  roscpp
  std_msgs
  message_generation
)

set(Sophus_LIBRARIES libSophus.so)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

add_executable(virtual_car
  src/virtual_car.cpp
  src/RosTopicConfigs.cpp
)

target_link_libraries(virtual_car
  ${catkin_LIBRARIES}
)

add_executable(physique_car
  src/physique_car.cpp
  src/RosTopicConfigs.cpp
)

target_link_libraries(physique_car
  ${catkin_LIBRARIES}
)
