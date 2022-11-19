

#include "include/planner_server.h"

planner_server::planner_server(ros::NodeHandle& _nh)
: nh(_nh), last_request(ros::Time::now().toSec())
{
    //subscribe
    uav_state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav/mavros/state", 1, &planner_server::uavStateCallback, this);
    
    uav_AlanPlannerMsg_sub = nh.subscribe<alan_landing_planning::AlanPlannerMsg>
            ("/AlanPlannerMsg/uav/data", 1, &planner_server::uavAlanMsgCallback, this);

    ugv_AlanPlannerMsg_sub = nh.subscribe<alan_landing_planning::AlanPlannerMsg>
            ("/AlanPlannerMsg/ugv/data", 1, &planner_server::ugvAlanMsgCallback, this);


    //publish
    pub_fsm = nh.advertise<alan_landing_planning::StateMachine>
            ("/alan_fsm", 1);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav/mavros/setpoint_position/local", 1);
        //   /mavros/setpoint_position/local
    local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("/uav/mavros/setpoint_velocity/cmd_vel_unstamped", 5);
    
    //client
    uav_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav/mavros/cmd/arming");

    uav_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav/mavros/set_mode");
    
    uav_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    alan_fsm_object.finite_state_machine = IDLE;

    uav_traj_pose_desired.pose.position.x = takeoff_hover_pt.x;
    uav_traj_pose_desired.pose.position.y = takeoff_hover_pt.y;
    uav_traj_pose_desired.pose.position.z = takeoff_hover_pt.z;

}

planner_server::~planner_server()
{

}

void planner_server::uavStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    uav_current_state = *msg;
}

void planner_server::uavAlanMsgCallback(const alan_landing_planning::AlanPlannerMsg::ConstPtr& msg)
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

    uav_traj_pose(0) = uav_current_AlanPlannerMsg.position.x;
    uav_traj_pose(1) = uav_current_AlanPlannerMsg.position.y;
    uav_traj_pose(2) = uav_current_AlanPlannerMsg.position.z;

    Eigen::Quaterniond q(
        uav_current_AlanPlannerMsg.orientation.ow,
        uav_current_AlanPlannerMsg.orientation.ox,
        uav_current_AlanPlannerMsg.orientation.oy,
        uav_current_AlanPlannerMsg.orientation.oz
        );
    
    // uav_traj_pose(3) = q.toRotationMatrix().eulerAngles(2);

    double yaw = atan2(q.toRotationMatrix()(1,0), q.toRotationMatrix()(0,0));

    uav_traj_pose(3) = yaw;

    // cout<<"yaw: "<<uav_traj_pose(3)<<endl;

    // cout<<euler(0)<<endl;;
    // cout<<euler(1)<<endl;;
    // cout<<euler(2)<<endl<<endl;;;

    // cout<<uavOdomPose.matrix()<<endl;

    // cout<<uav_current_AlanPlannerMsg.frame<<endl;

    // cout<<"im here"<<uav_current_AlanPlannerMsg.position.x<<endl;
}

void planner_server::ugvAlanMsgCallback(const alan_landing_planning::AlanPlannerMsg::ConstPtr& msg)
{
    ugv_current_AlanPlannerMsg = *msg;
    // ugv_current_AlanPlannerMsg.position.x;

    Eigen::Translation3d t_(
        ugv_current_AlanPlannerMsg.position.x,
        ugv_current_AlanPlannerMsg.position.y,
        ugv_current_AlanPlannerMsg.position.z
        );

    Eigen::Quaterniond q_(
        ugv_current_AlanPlannerMsg.orientation.ow,
        ugv_current_AlanPlannerMsg.orientation.ox,
        ugv_current_AlanPlannerMsg.orientation.oy,
        ugv_current_AlanPlannerMsg.orientation.oz
        );
    
    ugvOdomPose = t_ * q_;

    ugv_traj_pose(0) = ugv_current_AlanPlannerMsg.position.x;
    ugv_traj_pose(1) = ugv_current_AlanPlannerMsg.position.y;
    ugv_traj_pose(2) = ugv_current_AlanPlannerMsg.position.z;    

    // cout<<target_traj_pose<<endl;

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
        if(ros::Time::now().toSec() - last_request > ros::Duration(2.0).toSec())
        {
            fsm_state = FOLLOW;
            print_or_not = true;
            last_request = ros::Time::now().toSec();
        }

    }
    else if(fsm_state == FOLLOW)
    {
        if(print_or_not)
        {
            ROS_GREEN_STREAM(FOLLOW);
            print_or_not = false;
        }
        if(go_to_rendezvous_pt_and_follow())
        {
            fsm_state = LAND;
            print_or_not = true;
            last_request = ros::Time::now().toSec();
        }

    }
    else if(fsm_state == LAND)
    {
        if(print_or_not)
        {
            ROS_CYAN_STREAM(LAND);
            print_or_not = false;
        }

        if(land())
        {
            fsm_state = SHUTDOWN;
            print_or_not = true;
            last_request = ros::Time::now().toSec();
        }

    }
    else if(fsm_state == SHUTDOWN)
    {
        if(print_or_not)
        {
            ROS_GREEN_STREAM(SHUTDOWN);
            print_or_not = false;
        }
        if(shutdown())
        {
            
        }
    }
    else
    {
        ROS_ERROR("Please Check System...");
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

bool planner_server::go_to_rendezvous_pt_and_follow()
{
    uav_traj_pose_desired.pose.position.x = 2;
    uav_traj_pose_desired.pose.position.y = 2;
    uav_traj_pose_desired.pose.position.z = takeoff_hover_pt.z;


    target_traj_pose(0) = 2;
    target_traj_pose(1) = 2;
    target_traj_pose(2) = 2.5;
    target_traj_pose(3) = M_PI /2;
    //enter ugv and uav rendezvous point here

    Eigen::Vector4d twist_result = pid_controller(uav_traj_pose, target_traj_pose);

    // Eigen::Quaterniond q = yaw2q(twist_result(3) / 2);

    // uav_traj_pose_desired.pose.orientation.w = q.w();
    // uav_traj_pose_desired.pose.orientation.x = q.x();
    // uav_traj_pose_desired.pose.orientation.y = q.y();
    // uav_traj_pose_desired.pose.orientation.z = q.z();

    uav_traj_twist_desired.linear.x = twist_result(0);
    uav_traj_twist_desired.linear.y = twist_result(1);
    uav_traj_twist_desired.linear.z = twist_result(2);
    uav_traj_twist_desired.angular.z = twist_result(3);

    if(ros::Time::now().toSec() - last_request > ros::Duration(10.0).toSec())
        return true;
    else 
        return false;

}

bool planner_server::land()
{
    return false;
}

bool planner_server::shutdown()
{
    return false;
}

void planner_server::planner_pub()
{
    // cout<<uav_traj_desired.pose.position.x<<endl;

    if(fsm_state == FOLLOW || fsm_state == LAND)
        local_vel_pub.publish(uav_traj_twist_desired);
    else
        local_pos_pub.publish(uav_traj_pose_desired);

    alan_fsm_object.finite_state_machine = fsm_state;
    pub_fsm.publish(alan_fsm_object);

}

Eigen::Vector4d planner_server::pid_controller(Eigen::Vector4d pose, Eigen::Vector4d setpoint)
{
    Eigen::Vector4d error, u_p, u_i, u_d, output, derivative;
    // error[0] = 1;
    double iteration_time = ros::Time::now().toSec() - pid_last_request;

    // cout<<"time: "<<iteration_time<<endl<<endl;

    if(iteration_time > 1)
    {
        pid_last_request = ros::Time::now().toSec();
        return Eigen::Vector4d(0, 0, 0, 0);
    }
        

    Eigen::Vector4d K_p(1.2, 1.2, 1.5, 1);
    Eigen::Vector4d K_i(0.05, 0.05, 0.05, 0.05);
    Eigen::Vector4d K_d(0, 0, 0, 0);

    error = setpoint - pose;

    // cout<<"error:\n"<<error<<endl;;

    if (error[3] >= M_PI)
    {
        error[3] -= 2 * M_PI;
    }

    if (error[3] <= -M_PI)
    {
        error[3] += 2  *M_PI;
    }

    for (int i = 0; i < 4; i++)
    { 
        //i = x,y,z
        integral[i] += (error[i] * iteration_time);

        if(integral[i] >  1)
        { 
            integral[i] = 1;
        }

        if(integral[i] < -1)
        { 
            integral[i] = -1;
        }

        derivative[i] = (error[i] - last_error[i]) / (iteration_time + 1e-10);

        u_p[i] = error[i] * K_p[i];        //P controller
        u_i[i] = integral[i] * K_i[i];     //I controller
        u_d[i] = derivative[i] * K_d[i];   //D controller

        output[i] = u_p[i] + u_i[i] + u_d[i];
        
    }

    // cout<<endl;

    // cout<<"output here:\n"<<output<<endl;

    for (int i = 0; i < 3; i++)
    {
        if(output[i] >  1)
            { 
                output[i] =  1;
            }

        if(output[i] < -1)
        { 
            output[i] = -1;
        }
    }

    // cout<<"output here:\n"<<output<<endl<<endl;;

    last_error = error;
    pid_last_request = ros::Time::now().toSec();

    return output;
}



// Eigen::AngleAxisd rollAngle(0.872 * noise, Eigen::Vector3d::UnitZ());
//     Eigen::AngleAxisd yawAngle(0.872 * noise, Eigen::Vector3d::UnitY());
//     Eigen::AngleAxisd pitchAngle(0.872 * noise, Eigen::Vector3d::UnitX());

//     Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
