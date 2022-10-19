#ifndef PLANNER_SERVER_H
#define PLANNER_SERVER_H
#include "include/planner_server.h"

planner_server::planner_server(ros::NodeHandle& _nh)
: nh(_nh), last_request(ros::Time::now().toSec())
{
    uav_state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 1, &planner_server::uavStateCallback, this);

    uav_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");

    uav_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 1);
            //   /mavros/setpoint_position/local

    pub_fsm = nh.advertise<alan::StateMachine>
            ("/alan_fsm", 1);
    
    uav_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    alan_fsm_object.finite_state_machine = IDLE;

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.2;

    

}

planner_server::~planner_server()
{

}

void planner_server::uavStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    uav_current_state = *msg;
}

void planner_server::mainserver()
{
    ros::Rate rate(50.0);

    while(ros::ok())
    {
        // std::cout<<"mode: "<<uav_current_state.mode<<std::endl;

        if( uav_current_state.mode != "OFFBOARD" &&
            (ros::Time::now().toSec() - last_request > ros::Duration(2.0).toSec()))
        {
            if( uav_set_mode_client.call(uav_set_mode) &&
                uav_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
                alan_fsm_object.finite_state_machine = READY;
            }   
            last_request = ros::Time::now().toSec();
        } 
        else 
        {
            if( !uav_current_state.armed &&
                (ros::Time::now().toSec() - last_request > ros::Duration(2.0).toSec()))
            {
                if( uav_arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                    alan_fsm_object.finite_state_machine = TAKEOFF;
                }
                last_request = ros::Time::now().toSec();
            }
        }
        // cout<<"?"<<endl;
        local_pos_pub.publish(pose);

        // alan_fsm_object.finite_state_machine = IDLE;
        pub_fsm.publish(alan_fsm_object);

        ros::spinOnce();
        rate.sleep();

    }

}

#endif