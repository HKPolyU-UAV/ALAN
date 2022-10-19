#include <ros/ros.h>
#include "include/planner_server.h"

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh("~");

    planner_server alan_planner_server(nh);

    alan_planner_server.mainserver();


    

    // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    //         ("mavros/state", 10, state_cb);
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/setpoint_position/local", 10);
    // ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    //         ("mavros/cmd/arming");
    // ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    //         ("mavros/set_mode");

    // //the setpoint publishing rate MUST be faster than 2Hz
    // ros::Rate rate(50.0);

    // // wait for FCU connection
    // // while(ros::ok() && !current_state.connected){
    // //     ros::spinOnce();
    // //     rate.sleep();
    // // }

    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 1.2;

    // //send a few setpoints before starting
    // // for(int i = 100; ros::ok() && i > 0; --i){
    // //     local_pos_pub.publish(pose);
    // //     ros::spinOnce();
    // //     rate.sleep();
    // // }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    // ros::Time last_request = ros::Time::now();

    // while(ros::ok())
    // {
    //     std::cout<<"mode: "<<current_state.mode<<std::endl;

    //     if( current_state.mode != "OFFBOARD" &&
    //         (ros::Time::now() - last_request > ros::Duration(5.0)))
    //     {
    //         if( set_mode_client.call(offb_set_mode) &&
    //             offb_set_mode.response.mode_sent)
    //             {
    //                 ROS_INFO("Offboard enabled");
    //             }   
    //         last_request = ros::Time::now();
    //     } 
    //     else 
    //     {
    //         if( !current_state.armed &&
    //             (ros::Time::now() - last_request > ros::Duration(5.0)))
    //         {
    //             if( arming_client.call(arm_cmd) &&
    //                 arm_cmd.response.success){
    //                 ROS_INFO("Vehicle armed");
    //             }
    //             last_request = ros::Time::now();
    //         }
    //     }

    //     local_pos_pub.publish(pose);

    //     ros::spinOnce();
    //     rate.sleep();
    // }

    return 0;
}