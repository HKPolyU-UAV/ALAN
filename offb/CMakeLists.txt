cmake_minimum_required(VERSION 3.0.2)
project(offb)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
  genmsg
  visualization_msgs
  pcl_conversions
  pcl_ros 

)

find_package(OpenCV 4.5 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(
 FILES
 obj.msg
)


generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offb
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs message_generation
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

)

add_executable(camera
    src/camera.cpp
    src/include/run_yolo.cpp)

target_link_libraries(camera
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

add_executable(vis
    src/vis.cpp
    src/include/run_yolo.cpp)
target_link_libraries(vis
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES})

add_executable(trial
    src/trial.cpp
    src/include/run_yolo.cpp)

target_link_libraries(trial
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES})

add_executable(kf
   src/kf.cpp
   src/include/run_yolo.cpp)

target_link_libraries(kf
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})











