#ifndef PLAN_H
#define PLAN_H
#include "include/planner.h"

void* alan::PlannerNodelet::PubMainLoop(void* tmp)
{
    PlannerNodelet* pub = (PlannerNodelet*) tmp;

    ros::Rate loop_rate(50);
    pub->offb_set_mode.request.custom_mode = "OFFBOARD";
    pub->arm_cmd.request.value = true;

    while (ros::ok()) 
    {
        // fsm_manager(state);
        pub->fsm_manager(pub->state);

        // cout<<"!!!!! uav state:"
        geometry_msgs::PoseStamped temp;
        temp.pose.position.x = 10;
    // pub_traj_pos.publish(temp);

        pub->pub_traj_pos.publish(temp);

        ros::spinOnce();
        loop_rate.sleep();
    }

    void* result;

    return result;
}

void alan::PlannerNodelet::uavStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    uav_current_state = *msg;

    std::cout<<"mode: "<<uav_current_state.mode<<std::endl;

    if(uavOdomInitiated && uavAccInitiated && uav_current_state.connected)
    {
        if( uav_current_state.mode != "OFFBOARD" &&
            (ros::Time::now().toSec() - last_request > ros::Duration(5.0).toSec()))
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }   
            last_request = ros::Time::now().toSec();
        } 
        else 
        {
            if( !uav_current_state.armed &&
                (ros::Time::now().toSec() - last_request > ros::Duration(5.0).toSec()))
            {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now().toSec();
            }
        }

    }





    
}

void alan::PlannerNodelet::uavOdometryCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
    uav_odom = *msg;
    // cout<<"odom: "<<uav_odom.pose.pose.orientation<<endl;
    Eigen::Translation3d t_(uav_odom.pose.pose.position.x, uav_odom.pose.pose.position.y, uav_odom.pose.pose.position.z);
    Eigen::Quaterniond q_(
        uav_odom.pose.pose.orientation.w,
        uav_odom.pose.pose.orientation.x,
        uav_odom.pose.pose.orientation.y,
        uav_odom.pose.pose.orientation.z
        );
    
    uavOdomPose = t_ * q_;

    uavOdomInitiated = true;
    uav_traj_desi.pose.position.x = 1;
    // pub_traj_pos.publish(uav_traj_desi);



    // Eigen::
    // msg.
}

void alan::PlannerNodelet::uavImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(uavOdomInitiated)
    {
        uav_imu = *msg;

        Eigen::Quaterniond quatGimbal(msg->orientation.w,
                                        msg->orientation.x,
                                        msg->orientation.y,
                                        msg->orientation.z);
        
        uavAcc = uavOdomPose.rotation() 
                    * quatGimbal.inverse() 
                    * Eigen::Vector3d(
                        uav_imu.linear_acceleration.x,
                        uav_imu.linear_acceleration.y,
                        uav_imu.linear_acceleration.z);

        uavAccInitiated = true;

    }

}

void alan::PlannerNodelet::fsm_manager(fsm& state)
{
    switch (state)
        {
        case IDLE:
            ready_uav(state);
            ROS_CYAN_STREAM("IDLE");

            break;

        case READY:
            arm_uav(state);
            ROS_GREEN_STREAM("READY");

            break;

        case TAKEOFF:
            ROS_CYAN_STREAM("TAKEOFF");

            break;

        case RENDEZVOUS:
            ROS_BLUE_STREAM("RENDEZVOUS");
            break;

        case FOLLOW:
            ROS_BLUE_STREAM("FOLLOW");
            break;

        case LAND:
            ROS_BLUE_STREAM("LAND");
            break;

        case REPLAN:
            ROS_RED_STREAM("REPLAN");
            break;
        
        case SHUTDOWN:
            ROS_GREEN_STREAM("SHUTDOWN");
            break;
        
        
        default:
            ROS_ERROR("CHECK LAUNCH FILES");
            break;
        }
}

void alan::PlannerNodelet::ready_uav(fsm& state)
{
    if(uavOdomInitiated && uavAccInitiated && uav_current_state.connected)
        state = READY;
    else
        state = IDLE;

}

void alan::PlannerNodelet::arm_uav(fsm& state)
{
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    cout<<"mode:"<<uav_current_state.mode<<endl;


    if( uav_current_state.mode != "OFFBOARD" && (ros::Time::now().toSec() - last_request > ros::Duration(1.0).toSec()))
    {
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
            cout<<"why"<<endl;
            ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now().toSec();
    }
    else
    {
        if( !uav_current_state.armed && (ros::Time::now().toSec() - last_request > ros::Duration(1.0).toSec()))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
                state = TAKEOFF;
            }
            last_request = ros::Time::now().toSec();
        }
    }
    geometry_msgs::PoseStamped temp;
    temp.pose.position.x = 10;
    pub_traj_pos.publish(temp);
}

void alan::PlannerNodelet::takeoff_uav(fsm& state)
{

    // alan_traj::

}

void alan::PlannerNodelet::traj_setup(
    alan_traj::endpt start_pt,
    alan_traj::endpt end_pt,
    vector<alan_traj::corridor> cube_list
    )
{
    start = start_pt;
    end = end_pt;

    double dis = 
          pow(end_pt.posi.x - start_pt.posi.x, 2)
        + pow(end_pt.posi.y - start_pt.posi.y, 2)
        + pow(end_pt.posi.z - start_pt.posi.z, 2);

    dis = sqrt(dis);

    double t = (dis / vel_avg) / m;

    trajseg_t.clear();

    for(int i = 0; i < m; i++)
    {
        trajseg_t.push_back(t);
    }


}


#endif