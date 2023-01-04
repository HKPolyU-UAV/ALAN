#include "include/planner_server.h"

planner_server::planner_server(ros::NodeHandle& _nh, int pub_freq)
: nh(_nh), last_request(ros::Time::now().toSec()), _pub_freq(pub_freq)
{
    nh.getParam("/alan_master_planner_node/final_landing_x", final_landing_x);
    nh.getParam("/alan_master_planner_node/landing_velocity", uav_landing_velocity);
    nh.getParam("/alan_master_planner_node/take_off_height", take_off_height);
    nh.getParam("/alan_master_planner_node/ugv_height", ugv_height);
    cout<<"ugv_height: "<<ugv_height<<endl;
    nh.getParam("/alan_master_planner_node/v_max", v_max);
    nh.getParam("/alan_master_planner_node/a_max", a_max);

    cout<<"v_max: "<<v_max<<endl;
    cout<<"a_max: "<<a_max<<endl;

    nh.getParam("/alan_master/final_corridor_height", final_corridor_height);
    nh.getParam("/alan_master/final_corridor_length", final_corridor_length);


    //subscribe
    uav_state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav/mavros/state", 1, &planner_server::uavStateCallback, this);
    
    uav_AlanPlannerMsg_sub = nh.subscribe<alan_landing_planning::AlanPlannerMsg>
            ("/AlanPlannerMsg/uav/data", 1, &planner_server::uavAlanMsgCallback, this);

    ugv_AlanPlannerMsg_sub = nh.subscribe<alan_landing_planning::AlanPlannerMsg>
            ("/AlanPlannerMsg/ugv/data", 1, &planner_server::ugvAlanMsgCallback, this);

    sfc_sub = nh.subscribe<alan_visualization::PolyhedronArray>
            ("/alan/sfc/all_corridors", 1, &planner_server::sfcMsgCallback, this);


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

    cout<<"final_landing_x: "<<final_landing_x<<endl;

    uav_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
    alan_fsm_object.finite_state_machine = IDLE;

    set_btraj_info();
    set_btraj_inequality_dynamic();

    //block traj temp
    set_block_traj = true;
}

planner_server::~planner_server()
{

}

void planner_server::mainserver()
{
    ros::Rate rate(_pub_freq);

    while(ros::ok())
    {
        if(
            uav_current_state_inititaed     &&
            uav_traj_pose_initiated         &&
            ugv_traj_pose_initiated         &&
            land_traj_constraint_initiated
        )
        {        
            fsm_manager();
            planner_pub();            
        }
        
        ros::spinOnce();
        rate.sleep();
    }
}

void planner_server::uavStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    uav_current_state = *msg;
    uav_current_state_inititaed = true;
}

void planner_server::uavAlanMsgCallback(const alan_landing_planning::AlanPlannerMsg::ConstPtr& msg)
{
    uav_current_AlanPlannerMsg = *msg;

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

    uav_traj_pose.head<3>() = t_.translation();
    uav_traj_pose(3) = atan2(q_.toRotationMatrix()(1,0), q_.toRotationMatrix()(0,0));

    uav_traj_pose_initiated = true;
}

void planner_server::ugvAlanMsgCallback(const alan_landing_planning::AlanPlannerMsg::ConstPtr& msg)
{
    ugv_current_AlanPlannerMsg = *msg;

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

    ugv_traj_pose.head<3>() = t_.translation();
    ugv_traj_pose(3) = atan2(q_.toRotationMatrix()(1,0), q_.toRotationMatrix()(0,0));

    ugv_traj_pose_initiated = true;
}

void planner_server::sfcMsgCallback(const alan_visualization::PolyhedronArray::ConstPtr& msg)
{
    land_traj_constraint.a_series_of_Corridor.clear();
    land_traj_constraint = *msg;

    land_traj_constraint_initiated = true;
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

            takeoff_hover_pt.x = uav_traj_pose(0);
            takeoff_hover_pt.y = uav_traj_pose(1);
            takeoff_hover_pt.z = take_off_height + ugv_height;

            target_traj_pose(0) = takeoff_hover_pt.x;
            target_traj_pose(1) = takeoff_hover_pt.y;
            target_traj_pose(2) = takeoff_hover_pt.z;
            target_traj_pose(3) = uav_traj_pose(3);

            cout<<"target takeoff position\n"<<target_traj_pose<<endl<<endl;
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

            plan_traj = true;
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
    //follow dynamic
    target_traj_pose = set_following_target_pose();
    
    //perform block traj
    // target_traj_pose = set_uav_block_pose();

    //should be time and following quality
    if(
        uav_current_AlanPlannerMsg.good2fly && 
        (ros::Time::now().toSec() - last_request) > ros::Duration(2.0).toSec()
    )
    {
        // cout<<"good to fly"<<endl;
        return true;
    }
    else
    {
        // cout<<"not good to fly"<<endl;
        return false;
    }
}

bool planner_server::hover()
{
    target_traj_pose(0) = 0.0;
    target_traj_pose(1) = 0.0;
    target_traj_pose(2) = 1.5;
    target_traj_pose(3) = M_PI;

    return false;
}

bool planner_server::land()
{    
    
    if(plan_traj)
    {        
        set_alan_b_traj();
        plan_traj = false;
    }



    return false;
}

bool planner_server::shutdown()
{
    return false;
}

void planner_server::planner_pub()
{
    // cout<<uav_traj_desired.pose.position.x<<endl;

    Eigen::Vector4d twist_result = pid_controller(uav_traj_pose, target_traj_pose);

    uav_traj_twist_desired.linear.x = twist_result(0);
    uav_traj_twist_desired.linear.y = twist_result(1);
    uav_traj_twist_desired.linear.z = twist_result(2);
    uav_traj_twist_desired.angular.z = twist_result(3);

    // cout<<target_traj_pose<<endl;
    // cout<<twist_result<<endl<<endl;

    local_vel_pub.publish(uav_traj_twist_desired);

    // if(fsm_state == FOLLOW || fsm_state == LAND)
    //     local_vel_pub.publish(uav_traj_twist_desired);
    // else
    //     local_pos_pub.publish(uav_traj_pose_desired);

    alan_fsm_object.finite_state_machine = fsm_state;
    pub_fsm.publish(alan_fsm_object);

}

Eigen::Vector4d planner_server::pid_controller(Eigen::Vector4d pose, Eigen::Vector4d setpoint)
{
    Eigen::Vector4d error, u_p, u_i, u_d, output, derivative;
    // error[0] = 1;
    double iteration_time = ros::Time::now().toSec() - pid_last_request;

    if(iteration_time > 1)
    {
        pid_last_request = ros::Time::now().toSec();
        return Eigen::Vector4d(0, 0, 0, 0);
    }
        
    Eigen::Vector4d K_p(2.0, 2.0, 1.5, 1);
    Eigen::Vector4d K_i(0.05, 0.05, 0.05, 0.05);
    Eigen::Vector4d K_d(0, 0, 0, 0);

    error = setpoint - pose;


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

    last_error = error;
    pid_last_request = ros::Time::now().toSec();

    return output;
}

Eigen::Vector4d planner_server::set_following_target_pose()
{
    Eigen::Vector3d uav_following_pt = Eigen::Vector3d(-1.6, 0.0, take_off_height);    
    uav_following_pt =  ugvOdomPose.rotation() * uav_following_pt 
        + Eigen::Vector3d(
            ugvOdomPose.translation().x(),
            ugvOdomPose.translation().y(),
            ugvOdomPose.translation().z()
        );

    // uav_following_pt.x() = ugv_traj_pose(0);
    // uav_following_pt.y() = ugv_traj_pose(1);
    // uav_following_pt.z() = ugv_traj_pose(2) + take_off_height;

    Eigen::Vector4d following_target_pose;
    following_target_pose(0) = uav_following_pt(0);
    following_target_pose(1) = uav_following_pt(1);
    following_target_pose(2) = uav_following_pt(2);
    following_target_pose(3) = ugv_traj_pose(3);

    return following_target_pose;
}

Eigen::Vector4d planner_server::set_uav_block_pose()
{    
    if(set_block_traj)
    { 
        double vel = 0.2;

        cout<<"take_off_height: "<<take_off_height<<endl;

        Eigen::Vector3d v1 = Eigen::Vector3d(-0.0,  0.2, take_off_height);
        Eigen::Vector3d v2 = Eigen::Vector3d(-0.0, -0.2, take_off_height);
        Eigen::Vector3d v3 = Eigen::Vector3d(-2.0, -0.2, take_off_height);
        Eigen::Vector3d v4 = Eigen::Vector3d(-2.0,  0.2, take_off_height);

        vector<Eigen::Vector3d> traj_per_edge;
        
        //v1-v2
        double delta_distance = (v1 - v2).norm();
        int total_time_steps = delta_distance / vel * _pub_freq;

        for(int i = 0; i < total_time_steps; i++)
        {
            traj_per_edge.emplace_back(Eigen::Vector3d(
                v1.x(),
                v1.y() + (v2.y() - v1.y()) / total_time_steps * (i + 1),
                v1.z()
            ));
        }
        block_traj_pts.emplace_back(traj_per_edge);

        //v2-v3
        traj_per_edge.clear();
        delta_distance = (v2 - v3).norm();
        total_time_steps = delta_distance / vel * _pub_freq;

        for(int i = 0; i < total_time_steps; i++)
        {
            traj_per_edge.emplace_back(Eigen::Vector3d(
                v2.x() + (v3.x() - v2.x()) / total_time_steps * (i + 1),
                v2.y(),
                v2.z()
            ));
        }
        block_traj_pts.emplace_back(traj_per_edge);

        //v3-v4
        traj_per_edge.clear();
        delta_distance = (v3 - v4).norm();
        total_time_steps = delta_distance / vel * _pub_freq;

        for(int i = 0; i < total_time_steps; i++)
        {
            traj_per_edge.emplace_back(Eigen::Vector3d(
                v3.x(),
                v3.y() + (v4.y() - v3.y()) / total_time_steps * (i + 1),
                v3.z()
            ));
        }
        block_traj_pts.emplace_back(traj_per_edge);

        //v4-v1
        traj_per_edge.clear();
        delta_distance = (v1 - v4).norm();
        total_time_steps = delta_distance / vel * _pub_freq;

        for(int i = 0; i < total_time_steps; i++)
        {
            traj_per_edge.emplace_back(Eigen::Vector3d(
                v4.x() + (v1.x() - v4.x()) / total_time_steps * (i + 1),
                v4.y(),
                v4.z()
            ));
        }
        block_traj_pts.emplace_back(traj_per_edge);

        cout<<"total edge:..."<<block_traj_pts.size()<<endl;
        cout<<"v1-v2:........"<<block_traj_pts[0].size()<<endl;
        cout<<"v2-v3:........"<<block_traj_pts[1].size()<<endl;
        cout<<"v3-v4:........"<<block_traj_pts[2].size()<<endl;
        cout<<"v4-v1:........"<<block_traj_pts[3].size()<<endl;


        //repeat 4 times        

        for(int i = 0; i < 4; i++)
        {
            block_traj_pts.emplace_back(block_traj_pts[i]);
        }

        for(int i = 0; i < 4; i++)
        {
            block_traj_pts.emplace_back(block_traj_pts[i]);
        }

        for(int i = 0; i < 4; i++)
        {
            block_traj_pts.emplace_back(block_traj_pts[i]);
        }
        
        cout<<"total edge:..."<<block_traj_pts.size()<<endl;

        set_block_traj = false;
        wp_counter_i = 0;
        traj_counter_j = 0;
        // cout<<block_traj_pts[0][0].x()<<endl;
        // cout<<block_traj_pts[0][0].y()<<endl;
        // cout<<block_traj_pts[0][0].z()<<endl;
    }

    Eigen::Vector4d following_target_pose;
    // cout<<traj_counter_j<<endl;
    // cout<<wp_counter_i<<endl<<endl;

    if(traj_counter_j == block_traj_pts[wp_counter_i].size())
    {
        wp_counter_i++;
        traj_counter_j = 0;
        landing_hover_pt.x = uav_traj_pose(0);
        landing_hover_pt.y = uav_traj_pose(1);
        landing_hover_pt.z = uav_traj_pose(2);
        landing_hover_pt.yaw = uav_traj_pose(3);
    }
        
    
    if(wp_counter_i == block_traj_pts.size())
    {
        following_target_pose(0) = landing_hover_pt.x;
        following_target_pose(1) = landing_hover_pt.y;
        following_target_pose(2) = landing_hover_pt.z;
        following_target_pose(3) = landing_hover_pt.yaw;

        return following_target_pose;
    }
    else 
    {
        Eigen::Vector3d uav_following_pt;
        uav_following_pt.x() = block_traj_pts[wp_counter_i][traj_counter_j].x();
        uav_following_pt.y() = block_traj_pts[wp_counter_i][traj_counter_j].y();
        uav_following_pt.z() = block_traj_pts[wp_counter_i][traj_counter_j].z();

        uav_following_pt =  ugvOdomPose.rotation() * uav_following_pt 
        + Eigen::Vector3d(
            ugvOdomPose.translation().x(),
            ugvOdomPose.translation().y(),
            ugvOdomPose.translation().z()
        );

        traj_counter_j ++;
        // cout<<traj_counter_j<<endl;
        // cout<<"here"<<endl;
        following_target_pose(0) = uav_following_pt.x();
        following_target_pose(1) = uav_following_pt.y();
        following_target_pose(2) = uav_following_pt.z();
        following_target_pose(3) = ugv_traj_pose(3);

        // cout<<following_target_pose<<endl<<endl;


        return following_target_pose;
        
    }

}

void planner_server::set_alan_b_traj()
{
    cout<<1<<endl;            
    set_traj_time();
    cout<<2<<endl;
    set_btraj_equality_constraint();
    cout<<3<<endl;
    set_btraj_inequality_kinematic();
    cout<<4<<endl;
    set_btraj_inequality_dynamic();
    cout<<5<<endl;
    
    planner_display();
        
    alan_traj::traj_gen alan_btraj(btraj_info, btraj_constraints, _pub_freq);
    alan_btraj.solve_opt(_pub_freq);
    alan_optiTraj = alan_btraj.getOptiTraj();

    ros::shutdown();

}

void planner_server::set_btraj_info()
{
    btraj_info.axis_dim = 3;
    btraj_info.n_order = 7;
    btraj_info.m = 2;
    btraj_info.d_order = 3;

}

void planner_server::set_btraj_equality_constraint()
{
    start_3d.posi(0) = uav_current_AlanPlannerMsg.position.x;
    start_3d.velo(0) = uav_current_AlanPlannerMsg.velocity.x;
    start_3d.accl(0) = uav_current_AlanPlannerMsg.acceleration.x;

    start_3d.posi(1) = uav_current_AlanPlannerMsg.position.y;
    start_3d.velo(1) = uav_current_AlanPlannerMsg.velocity.y;
    start_3d.accl(1) = uav_current_AlanPlannerMsg.acceleration.y;

    start_3d.posi(2) = uav_current_AlanPlannerMsg.position.z;
    start_3d.velo(2) = uav_current_AlanPlannerMsg.velocity.z;
    start_3d.accl(2) = uav_current_AlanPlannerMsg.acceleration.z;

    
    ////
    end_3d.posi(0) = ugv_current_AlanPlannerMsg.position.x 
        + ugv_current_AlanPlannerMsg.velocity.x * landing_time_total;
    end_3d.velo(0) = ugv_current_AlanPlannerMsg.velocity.x; 
    end_3d.accl(0) = ugv_current_AlanPlannerMsg.acceleration.x;
    
    end_3d.posi(1) = ugv_current_AlanPlannerMsg.position.y 
        + ugv_current_AlanPlannerMsg.velocity.y * landing_time_total;
    end_3d.velo(1) = ugv_current_AlanPlannerMsg.velocity.y;
    end_3d.accl(1) = ugv_current_AlanPlannerMsg.acceleration.y;
    
    end_3d.posi(2) = ugv_current_AlanPlannerMsg.position.z;
        + ugv_current_AlanPlannerMsg.velocity.x * landing_time_total;
    end_3d.velo(2) = ugv_current_AlanPlannerMsg.velocity.z;
    end_3d.accl(2) = ugv_current_AlanPlannerMsg.acceleration.z;

    btraj_constraints.start = start_3d;
    btraj_constraints.end = end_3d;

}

void planner_server::set_btraj_inequality_kinematic()
{
    int temp_size_i = 0;

    cout<<"we got this many corridor:..."<<land_traj_constraint.a_series_of_Corridor.size()<<endl;
    
    corridors.clear();

    for(int i = 0; i < land_traj_constraint.a_series_of_Corridor.size(); i++)
    {
        temp_size_i = land_traj_constraint.a_series_of_Corridor[i].PolyhedronTangentArray.size();
        
        temp_poly.PolyhedronTangentArray.clear();

        for(int j = 0; j < temp_size_i; j++)
        {
            temp_poly.PolyhedronTangentArray.emplace_back(
                land_traj_constraint.a_series_of_Corridor[i].PolyhedronTangentArray[j]
            );
        }

        corridors.emplace_back(temp_poly);
    }

    btraj_constraints.sfc_list = corridors;
    btraj_constraints.corridor_type = "POLYH";

}

void planner_server::set_btraj_inequality_dynamic()
{
    
    btraj_dconstraints.v_max(0) = v_max;
    btraj_dconstraints.v_min(0) = -v_max;
    btraj_dconstraints.a_max(0) = a_max;
    btraj_dconstraints.a_min(0) = -a_max;

    btraj_dconstraints.v_max(1) = v_max;
    btraj_dconstraints.v_min(1) = -v_max;
    btraj_dconstraints.a_max(1) = a_max;
    btraj_dconstraints.a_min(1) = -a_max;

    btraj_dconstraints.v_max(2) = v_max;
    btraj_dconstraints.v_min(2) = -v_max;
    btraj_dconstraints.a_max(2) = a_max;
    btraj_dconstraints.a_min(2) = -a_max;

    btraj_constraints.d_constraints = btraj_dconstraints;    

}

void planner_server::set_traj_time()
{
    btraj_info.s.clear();

    double distance_uav_ugv = sqrt(
        pow(
            uav_current_AlanPlannerMsg.position.x - ugv_current_AlanPlannerMsg.position.x,
            2
        )
        +
        pow(
            uav_current_AlanPlannerMsg.position.y - ugv_current_AlanPlannerMsg.position.y,
            2
        )
    );

    double diff_velo_uav_ugv = uav_landing_velocity - 
        sqrt(
            pow(
                ugv_current_AlanPlannerMsg.velocity.x,
                2
            ) 
            +
            pow(
                ugv_current_AlanPlannerMsg.velocity.y,
                2
            )
        );

    landing_time_total = distance_uav_ugv / diff_velo_uav_ugv;

    btraj_info.s.emplace_back(
        landing_time_total * 1.0 - (final_corridor_length / distance_uav_ugv)
        );
    btraj_info.s.emplace_back(
        landing_time_total * final_corridor_length / distance_uav_ugv
        );

}

void planner_server::planner_display()
{
    cout<<"\n\n\naxis_dim:\n";
    cout<<btraj_info.axis_dim<<endl<<endl;

    cout<<"n_order:\n";
    cout<<btraj_info.n_order<<endl<<endl;

    cout<<"m:\n";
    cout<<btraj_info.m<<endl<<endl;

    cout<<"d_order:\n";
    cout<<btraj_info.d_order<<endl<<endl;

    cout<<"s allocation:\n";
    for(int i = 0; i < btraj_info.s.size(); i++)
        cout<<btraj_info.s[i]<<" ";
    cout<<endl<<endl;

    cout<<"starting state:\n";
    cout<<"posi:\n";
    cout<<btraj_constraints.start.posi<<endl;
    cout<<"velo :\n";
    cout<<btraj_constraints.start.velo<<endl;
    cout<<"accl:\n";
    cout<<btraj_constraints.start.accl<<endl;
    cout<<"\nending state\n:";
    cout<<"posi:\n";
    cout<<btraj_constraints.end.posi<<endl;
    cout<<"velo:\n";
    cout<<btraj_constraints.end.velo<<endl;
    cout<<"accl:\n";
    cout<<btraj_constraints.end.accl<<endl;
    cout<<endl;

}