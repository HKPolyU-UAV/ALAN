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
  CATKIN_DEPENDS 
  geometry_msgs 
  mavros_msgs 
  roscpp
  std_msgs
  message_generation
#  DEPENDS system_lib
)

FIND_PACKAGE( OpenMP REQUIRED)
    if(OPENMP_FOUND)
      message("OPENMP FOUND")
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endif()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /home/patty/ncnn/build/install/include/ncnn
)
link_directories(
  /home/patty/ncnn/build/install/lib
)

add_executable(camera
    src/camera.cpp
    src/include/cnn.cpp)
target_link_libraries(camera
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

add_executable(vis
    src/vis.cpp
    src/include/cnn.cpp)
target_link_libraries(vis
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES})

add_executable(trial
    src/trial.cpp
    src/include/cnn.cpp)

target_link_libraries(trial
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES})

add_executable(kf
   src/kf.cpp
   src/include/cnn.cpp)

target_link_libraries(kf
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(yolo
   src/yolo.cpp
   src/include/cnn.cpp)

target_link_libraries(yolo
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ncnn
  /home/patty/ncnn/build/install/lib/libncnn.a
  )




###################################################


