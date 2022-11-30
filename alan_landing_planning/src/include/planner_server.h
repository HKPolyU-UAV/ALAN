#ifndef PLANNER_SERVER_H
#define PLANNER_SERVER_H

#include "tools/essential.h"
#include "bezier_lib/traj_gen.h"

// #include "alan/AlanPlanner.h"

#include "alan_landing_planning/AlanPlannerMsg.h"
#include "alan_landing_planning/StateMachine.h"
#include "alan_visualization/PolyhedronArray.h"


#define IDLE "IDLE"
#define ARMED "ARMED"
#define TOOKOFF "TOOKOFF"
#define RENDEZVOUS "RENDEZVOUS"
#define FOLLOW "FOLLOW"
#define LAND "LAND"
#define SHUTDOWN "SHUTDOWN"


// #define REPLAN "IDLE"

class planner_server
{
    typedef struct waypts
    {
        double x;
        double y;
        double z;
        double yaw;
    }waypts;

private:
    ros::NodeHandle nh;

    //subscriber
    ros::Subscriber uav_state_sub;
    ros::Subscriber uav_AlanPlannerMsg_sub, ugv_AlanPlannerMsg_sub;
    ros::Subscriber sfc_sub;

    //publisher
    ros::Publisher local_pos_pub;
    ros::Publisher local_vel_pub;
    ros::Publisher pub_fsm;
    

    //service
    ros::ServiceClient uav_arming_client;
    ros::ServiceClient uav_set_mode_client;

    //callback functions
    void uavStateCallback(const mavros_msgs::State::ConstPtr& msg);

    void uavAlanMsgCallback(const alan_landing_planning::AlanPlannerMsg::ConstPtr& msg);

    void ugvAlanMsgCallback(const alan_landing_planning::AlanPlannerMsg::ConstPtr& msg);


    void sfcMsgCallback(const alan_visualization::PolyhedronArray::ConstPtr& msg);


    //fsm
    void fsm_manager();

    bool get_ready();

    bool taking_off();

    bool go_to_rendezvous_pt_and_follow();

    bool land();

    bool shutdown();
    
    //block trajectory (for data collection)
    Eigen::Vector4d set_following_target_pose();
    Eigen::Vector4d set_uav_block_pose();
    bool set_block_traj = false;
    vector<vector<Eigen::Vector3d>> block_traj_pts;
    int wp_counter_i = 0;
    int traj_counter_j = 0;


    //trajectory
    vector<alan_landing_planning::AlanPlannerMsg> traj_optimized;

    //other functions
    void planner_pub();     

    //server

    //private variables

    string fsm_state = IDLE;
    waypts takeoff_hover_pt = {0,0,1.2,0};
    waypts landing_hover_pt = {0,0,1.2,0};
    
    Eigen::Isometry3d uavOdomPose, ugvOdomPose;

    mavros_msgs::SetMode uav_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::State uav_current_state;

    alan_visualization::PolyhedronArray land_traj_constraint;

    // sensor_msgs::Imu cam_pose;
    // Eigen::Quaterniond cam_pose;

    alan_landing_planning::AlanPlannerMsg uav_current_AlanPlannerMsg, ugv_current_AlanPlannerMsg;
    geometry_msgs::PoseStamped cam_current_PoseMsg;

    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped uav_traj_pose_desired;
    geometry_msgs::Twist uav_traj_twist_desired;
    alan_landing_planning::StateMachine alan_fsm_object;

    Eigen::Vector4d pid_controller(Eigen::Vector4d pose, Eigen::Vector4d setpoint);    

    Eigen::Vector4d uav_traj_pose, ugv_traj_pose, target_traj_pose, ugv_target_traj_pose;
    
    double last_request;

    double pid_last_request = 0;

    bool print_or_not = true;

    // double kp, ki, kd;
    
    Eigen::Vector4d last_error, integral;

    double uav_v_max = 1.5;

    int _pub_freq = 0;

    double final_landing_x = 0; 

    
    //set btraj
    double uav_landing_velocity = 0;

    void set_alan_b_traj();

    void set_btraj_info();

    void set_btraj_equality_constraint();

    void set_btraj_inequality_kinematic();

    void set_btraj_inequality_dynamic();

    void set_traj_time();

    double final_corridor_height = 0;
    double final_corridor_length = 0;
    double take_off_height = 0;
    double ugv_height = 0.0;

    alan_traj::bezier_info btraj_info;

    alan_traj::endpt start_3d;
    alan_traj::endpt end_3d;

    alan_traj::dynamic_constraints btraj_dconstraints;

    alan_traj::bezier_constraints btraj_constraints;

    bool plan_traj = false;

    int traj_i = 0;

    double landing_time_total = 0;

    alan_visualization::Polyhedron temp_poly;
    vector<alan_visualization::Polyhedron> corridors;


    double v_max, a_max;

    alan_landing_planning::Traj alan_optiTraj;

    //rotation function
    Eigen::Vector3d q2rpy(Eigen::Quaterniond q) {
        return q.toRotationMatrix().eulerAngles(2,1,0);
    };

    Eigen::Quaterniond rpy2q(Eigen::Vector3d rpy){
        Eigen::AngleAxisd rollAngle(rpy(0), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(rpy(1), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(rpy(2), Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

        return q;

    };

    Eigen::Vector3d q_rotate_vector(Eigen::Quaterniond q, Eigen::Vector3d v){
        return q * v;
    }


public:
    planner_server(ros::NodeHandle& _nh, int pub_freq);
    ~planner_server();

    void mainserver();

};


#endif