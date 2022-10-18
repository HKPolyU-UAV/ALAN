#include "include/plan.h"

void* alan::PlannerNodelet::PubMainLoop(void* tmp)
{
    PlannerNodelet* pub = (PlannerNodelet*) tmp;

    ros::Rate loop_rate(50);
    while (ros::ok()) 
    {
        pub->fsm_manager(pub->state);

        // cout<<"!!!!! uav state:"

        pub->pub_traj_pos.publish(pub->uav_traj_desi);

        ros::spinOnce();
        loop_rate.sleep();
    }

    void* result;

    return result;
}

void alan::PlannerNodelet::uavOdometryCallback(const nav_msgs::Odometry & msg)
{
    uav_odom = msg;
    // msg.
}

void alan::PlannerNodelet::uavStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void alan::PlannerNodelet::fsm_manager(fsm state)
{
    switch (state)
        {
        case IDLE:
            arm_uav();
            ROS_GREEN_STREAM("IDLE");

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



void alan::PlannerNodelet::arm_uav()
{
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    if( current_state.mode != "OFFBOARD" && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()))
    {
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
            ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now().toSec();
    }
    else
    {
        if( !current_state.armed && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
                state = TAKEOFF;
            }
            last_request = ros::Time::now().toSec();
        }
    }
}

void alan::PlannerNodelet::takeoff()
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


