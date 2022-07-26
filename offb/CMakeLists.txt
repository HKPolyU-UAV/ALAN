cmake_minimum_required(VERSION 3.0.2)
project(alan)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
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
  pcl_conversions
  pcl_ros 

)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
# find_package(PCL 1.2 REQUIRED)

roslaunch_add_file_check(launch)


add_message_files(
 FILES
 obj.msg
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
#  DEPENDS system_lib
)

# for ncnn
# FIND_PACKAGE( OpenMP REQUIRED)
#     if(OPENMP_FOUND)
#       message("OPENMP FOUND")
#       set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#       set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#       set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#     endif()

set(Sophus_LIBRARIES libSophus.so)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  # ${CMAKE_CURRENT_SOURCE_DIR}/src/include/ncnn
)

add_library(alan_pose_estimation
    src/alan_state_estimation/aruco.cpp
    src/alan_state_estimation/yolo.cpp
    src/alan_state_estimation/led.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
  alan_pose_estimation
  ${catkin_LIBRARIES}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)




# #1
# add_executable(yolo
#     src/yolo.cpp)
# target_link_libraries(yolo
#    ${OpenCV_LIBRARIES}
#    ${catkin_LIBRARIES})



# #2
# add_executable(aruco
#     src/aruco.cpp)
# target_link_libraries(aruco
#    ${OpenCV_LIBRARIES}
#    ${catkin_LIBRARIES}
#    ${PCL_LIBRARIES}
#    ${Sophus_LIBRARIES})

#3
add_executable(kf
   src/alan_state_estimation/kf.cpp)
target_link_libraries(kf
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

#4
add_executable(imu
   src/alan_state_estimation/imu.cpp)
target_link_libraries(imu
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

# #5
# add_executable(pnp
#    src/pnp.cpp)
# target_link_libraries(pnp
#   ${OpenCV_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES})

# add_executable(vis
#     src/vis.cpp)
# target_link_libraries(vis
#     ${OpenCV_LIBRARIES}
#     ${catkin_LIBRARIES})

# add_executable(ncnn_yolo
#    src/ncnn_yolo.cpp)
# target_link_libraries(ncnn_yolo
#   ${OpenCV_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ncnn
#   ${CMAKE_CURRENT_SOURCE_DIR}/src/include/ncnn/lib/libncnn.a
#   )

###################################################


