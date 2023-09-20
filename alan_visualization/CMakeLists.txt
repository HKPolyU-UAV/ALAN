cmake_minimum_required(VERSION 3.0.2)
project(alan_visualization)

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

find_package(Eigen3 REQUIRED)

find_package(Sophus REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(decomp_util REQUIRED)

roslaunch_add_file_check(launch)


add_message_files(
 FILES
 Normal.msg
 Point.msg
 Tangent.msg
 Polyhedron.msg
 PolyhedronArray.msg
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
  ${DECOMP_UTIL_INCLUDE_DIRS}
)


add_executable(vis
   src/vis.cpp
   )
   
target_link_libraries(vis
  ${catkin_LIBRARIES}
  )

  add_dependencies(
    planner_node
    alan_visualization_generate_messages_cpp
    alan_landing_planning_generate_messages_cpp
    decomp_ros_msgs_generate_messages_cpp
  )



add_executable(data
src/data.cpp
)

target_link_libraries(data
${catkin_LIBRARIES}
)

add_dependencies(
    data
    alan_state_estimation_generate_messages_cpp
  )
###################################################


