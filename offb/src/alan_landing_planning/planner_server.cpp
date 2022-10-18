// #include <iostream>
// #include "include/traj_gen.hpp"

// using namespace std;

// int main(int argc, char** argv)
// {
// 	ros::init(argc, argv, "lala");
//     ros::NodeHandle nh;
	
// 	int n_order = 7;
//     int m = 5;
//     int d_order = 4;

//     vector<double> s;
//     for(int i = 0; i < 5; i++)
//         s.push_back(1);
    
//     alan_traj::endpt_cond start;
//     start.p_ = 50;
//     start.v_ = 0;
//     start.a_ = 0;
//     start.j_ = 0;

//     alan_traj::endpt_cond end;
//     end.p_ = 280;
//     end.v_ = 0;
//     end.a_ = 0;
//     end.j_ = 0;

//     vector<alan_traj::corridor> cube_list;
//     alan_traj::corridor cube;

//     cube.x_max = 100;
//     cube.x_min = 0;//0;
//     //
//     cube_list.push_back(cube);

//     cube.x_max = 150;
//     cube.x_min = 50;//-OsqpEigen::INFTY;//50;
//     cube_list.push_back(cube);

//     cube.x_max = 230;
//     cube.x_min = 130;//-OsqpEigen::INFTY;//130;
//     cube_list.push_back(cube);
    
//     cube.x_max = 300;
//     cube.x_min = 200;//-OsqpEigen::INFTY;//200;
//     cube_list.push_back(cube);
    
//     cube.x_max = 330;
//     cube.x_min = 230;//-OsqpEigen::INFTY;//230;
//     cube_list.push_back(cube);

//     alan_traj::dynamic_constraints d_constraints;
//     d_constraints.v_max =  150;
//     d_constraints.v_min = -150;//OsqpEigen::INFTY;//-150;
//     d_constraints.a_max =  200;
//     d_constraints.a_min = -200;//OsqpEigen::INFTY;//-200;
//     d_constraints.j_max =  400;
//     d_constraints.j_min = -400;//OsqpEigen::INFTY;//-400;

//     alan_traj::bezier_info b_info;
//     alan_traj::bezier_constraints b_constraints;

//     b_info.n_order = n_order;
//     b_info.m = m;
//     b_info.d_order = d_order;
//     b_info.s = s;

//     b_constraints.start = start;
//     b_constraints.end = end;
//     b_constraints.cube_list = cube_list;
//     b_constraints.d_constraints = d_constraints;

//     double t0 = ros::Time::now().toSec();

// 	alan_traj::traj_gen traj(b_info, b_constraints);

// 	traj.solve_opt();

//     double t1 = ros::Time::now().toSec();

//     cout<<"fps: "<<1/(t1-t0)<<endl;

// 	return 0;
// }


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    // while(ros::ok() && !current_state.connected){
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.2;

    //send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        std::cout<<"mode: "<<current_state.mode<<std::endl;
        
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }   
            last_request = ros::Time::now();
        } 
        else 
        {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}