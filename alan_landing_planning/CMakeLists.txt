cmake_minimum_required(VERSION 3.0.2)
project(alan_landing_planning)

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
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

roslaunch_add_file_check(launch)


add_message_files(
 FILES
 Object.msg
 AlanPlannerMsg.msg 
 StateMachine.msg
 Target.msg
 vector3D.msg
 vector4D.msg
 Traj.msg
 TrajArray.msg
)


generate_messages(DEPENDENCIES 
  std_msgs 
  sensor_msgs
  )

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
  ${IPOPT_LIBRARIES}
)

add_executable(planner_node
   src/planner_node.cpp
   src/planner_server.cpp
   
   src/bernstein.cpp
   src/osqpsolver.cpp
   src/traj_gen.cpp
   src/traj_sampling.cpp)

target_link_libraries(planner_node
  ${catkin_LIBRARIES}
  ${OSQP_LIBRARIES}
  osqp::osqp
  OsqpEigen
  )

  add_dependencies(
    planner_node
    alan_visualization_generate_messages_cpp
  )

# add_executable(lala
#    src/lala.cpp
   
#    src/bernstein.cpp
#    src/osqpsolver.cpp
#    src/traj_gen.cpp
#    src/traj_sampling.cpp)
   
# target_link_libraries(lala
#   ${catkin_LIBRARIES}
#   ${OSQP_LIBRARIES}
  
#   osqp::osqp
#   OsqpEigen
#   )

# add_dependencies(
#   lala
#   alan_visualization_generate_messages_cpp
# )
###################################################


