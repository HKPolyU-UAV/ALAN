#ifndef PLANNER_SERVER_H
#define PLANNER_SERVER_H

#include "include/planner_server.h"

planner_server::planner_server(ros::NodeHandle& _nh)
: nh(_nh), last_request(ros::Time::now().toSec())
{
    //subscribe
    uav_state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 1, &planner_server::uavStateCallback, this);
    
    uav_AlanPlannerMsg_sub = nh.subscribe<alan::AlanPlannerMsg>
            ("/AlanPlannerMsg/uav/data", 1, &planner_server::uavAlanMsgCallback, this);

    //publish
    pub_fsm = nh.advertise<alan::StateMachine>
            ("/alan_fsm", 1);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 1);
            //   /mavros/setpoint_position/local
    
    //client
    uav_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");

    uav_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");
    
    uav_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    alan_fsm_object.finite_state_machine = IDLE;

    uav_traj_desired.pose.position.x = takeoff_hover_pt.x;
    uav_traj_desired.pose.position.y = takeoff_hover_pt.y;
    uav_traj_desired.pose.position.z = takeoff_hover_pt.z;

}

planner_server::~planner_server()
{

}

void planner_server::uavStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    uav_current_state = *msg;
}

void planner_server::uavAlanMsgCallback(const alan::AlanPlannerMsg::ConstPtr& msg)
{
    uav_current_AlanPlannerMsg = *msg;
    uav_current_AlanPlannerMsg.position.x;

    Eigen::Translation3d t_(
        uav_current_AlanPlannerMsg.position.x,
        uav_current_AlanPlannerMsg.position.y,
        uav_current_AlanPlannerMsg.position.z
        );

    Eigen::Quaterniond q_(
        uav_current_AlanPlannerMsg.orientation.ow,
        uav_current_AlanPlannerMsg.orientation.ox,
        uav_current_AlanPlannerMsg.orientation.oy,
        uav_current_AlanPlannerMsg.orientation.oz
        );
    
    uavOdomPose = t_ * q_;

    // cout<<uavOdomPose.matrix()<<endl;

    // cout<<uav_current_AlanPlannerMsg.frame<<endl;

    // cout<<"im here"<<uav_current_AlanPlannerMsg.position.x<<endl;
}

void planner_server::mainserver()
{
    ros::Rate rate(50.0);

    while(ros::ok())
    {
        fsm_manager();

        planner_pub();
        
        ros::spinOnce();
        rate.sleep();

    }

}

void planner_server::fsm_manager()
{
    if(fsm_state == IDLE)
    {
        if(print_or_not)
        {
            ROS_YELLOW_STREAM(IDLE);
            print_or_not = false;
        }
        if(get_ready())
        {
            fsm_state = ARMED;
            print_or_not = true;
        }
    }
    else if(fsm_state == ARMED)
    {
        if(print_or_not)
        {
            ROS_GREEN_STREAM(ARMED);
            print_or_not = false;
        }
        if(taking_off())
        {
            fsm_state = TOOKOFF;
            print_or_not = true;
            last_request = ros::Time::now().toSec();
        }

    }
    else if(fsm_state == TOOKOFF)
    {
        if(print_or_not)
        {
            ROS_GREEN_STREAM(TOOKOFF);
            print_or_not = false;
        }
        if(ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec())
        {
            fsm_state = RENDEZVOUS;
            print_or_not = true;
        }

    }
    else if(fsm_state == RENDEZVOUS)
    {
        if(print_or_not)
        {
            ROS_GREEN_STREAM(RENDEZVOUS);
            print_or_not = false;
        }

    }
    else
    {

    }

}

bool planner_server::get_ready()
{
    bool return_state = false;
    if( uav_current_state.mode != "OFFBOARD" &&
            (ros::Time::now().toSec() - last_request > ros::Duration(2.0).toSec()))
        {
            if( uav_set_mode_client.call(uav_set_mode) &&
                uav_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
                alan_fsm_object.finite_state_machine = ARMED;
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
                alan_fsm_object.finite_state_machine = TOOKOFF;
                return_state = true;
            }
            last_request = ros::Time::now().toSec();
        }
    }

    if(return_state)
        return true;
    else
        return false;

}

bool planner_server::taking_off()
{
    double dis = 
        pow(
            uav_current_AlanPlannerMsg.position.x - takeoff_hover_pt.x,
            2
        )
        + pow(
            uav_current_AlanPlannerMsg.position.y - takeoff_hover_pt.y,
            2
        )
        + pow(
            uav_current_AlanPlannerMsg.position.z - takeoff_hover_pt.z,
            2
        );
    
    dis = sqrt(dis);
    // cout<<dis<<endl;
    if(dis < 0.15)
        return true;
    else    
        return false;
}

bool planner_server::go_to_rendezvous_pt()
{


}



void planner_server::planner_pub()
{
    // cout<<uav_traj_desired.pose.position.x<<endl;
    local_pos_pub.publish(uav_traj_desired);
    alan_fsm_object.finite_state_machine = fsm_state;
    pub_fsm.publish(alan_fsm_object);

}

#endif