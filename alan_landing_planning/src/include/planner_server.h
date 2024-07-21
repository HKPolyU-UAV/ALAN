#ifndef PLANNER_SERVER_H
#define PLANNER_SERVER_H

#include "tools/essential.h"
#include "bezier_lib/traj_gen.h"
#include "bezier_lib/traj_sampling.h"

#include "alan_landing_planning/AlanPlannerMsg.h"
#include "alan_landing_planning/StateMachine.h"
#include "alan_visualization/PolyhedronArray.h"

#include <thread>
#include <tf/tf.h>

#include "airo_message/ReferencePreview.h"

#define IDLE "IDLE"
#define ARMED "ARMED"
#define TOOKOFF "TOOKOFF"
#define FOLLOW "FOLLOW"
#define RENDEZVOUS "RENDEZVOUS"
#define LAND "LAND"
#define SHUTDOWN "SHUTDOWN"
#define MISSION_COMPLETE "MISSION_COMPLETE"

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

//ros related
    //subscriber
    ros::Subscriber uav_state_sub;
    ros::Subscriber uav_AlanPlannerMsg_sub, ugv_AlanPlannerMsg_sub;
    ros::Subscriber ugv_pose_predict_sub;
    ros::Subscriber sfc_sub;

    //publisher
    ros::Publisher local_pos_pub;
    ros::Publisher local_vel_pub;
    ros::Publisher pub_fsm;
    ros::Publisher traj_pub;
    ros::Publisher trajArray_pub;
    ros::Publisher ctrl_pt_pub;
    ros::Publisher kill_attitude_target_pub;
    ros::Publisher airo_setpt_pub;
    
    //service
    ros::ServiceClient uav_arming_client;
    ros::ServiceClient uav_set_mode_client;

    //callback functions
    void uavStateCallback(const mavros_msgs::State::ConstPtr& msg);
    void uavAlanMsgCallback(const alan_landing_planning::AlanPlannerMsg::ConstPtr& msg);
    void ugvAlanMsgCallback(const alan_landing_planning::AlanPlannerMsg::ConstPtr& msg);
    void ugvPosePredictedMsgCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void sfcMsgCallback(const alan_visualization::PolyhedronArray::ConstPtr& msg);


//main brain
    //fsm
    void fsm_manager();
    bool get_ready();
    bool taking_off();
    bool go_to_rendezvous_pt_and_follow();
    bool rendezvous();
    bool hover();
    bool land();
    bool shutdown();
    
//parameters, states and desired states callback
    std::string fsm_state = IDLE;
    waypts takeoff_hover_pt = {0,0,1.2,0};
    waypts landing_hover_pt = {0,0,1.2,0};
    Eigen::Isometry3d uavOdomPose, ugvOdomPose, ugvOdomPose_predicted;
    mavros_msgs::SetMode uav_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::State uav_current_state;
    mavros_msgs::AttitudeTarget attitude_target_for_kill;
    bool uav_current_state_inititaed = false;
    bool kill_or_not = false;
    alan_landing_planning::AlanPlannerMsg uav_current_AlanPlannerMsg, ugv_current_AlanPlannerMsg;
    geometry_msgs::PoseStamped cam_current_PoseMsg;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped uav_traj_pose_desired;
    geometry_msgs::Twist uav_traj_twist_desired;
    alan_landing_planning::StateMachine alan_fsm_object;
    
//position controller related
    Eigen::Vector4d pid_controller(Eigen::Vector4d pose, Eigen::Vector4d setpoint);
    void planner_pub();     
    Eigen::Vector4d uav_traj_pose, ugv_traj_pose, ugv_traj_predict_pose, target_traj_pose, ugv_target_traj_pose;
    bool uav_traj_pose_initiated = false;
    bool ugv_traj_pose_initiated = false;
    double last_request;
    double pid_last_request = 0;
    bool print_or_not = true;
    double following_norm = 0;
    int land_fix_count = 0;

    XmlRpc::XmlRpcValue pid_gain_list;
    Eigen::Vector4d kp, ki, kd;
    Eigen::Vector4d last_error, integral;
    int _pub_freq = 0;

    
//set btraj
    alan_landing_planning::Traj traj_execute_final_in_B;
    //b_traj-related
    int axis_dim = 0;
    int n_order = 0;
    int m = 0;
    int d_order = 0;
    alan_traj::bezier_info btraj_info;
    alan_traj::endpt start_3d;
    alan_traj::endpt end_3d;
    alan_traj::dynamic_constraints btraj_dconstraints;
    // alan_traj::traj_gen alan_traj()
    
    void set_alan_b_traj_prerequisite();
    void set_btraj_inequality_kinematic();
    void set_alan_b_traj_online();
    
    //corridors-related
    double final_corridor_height = 0;
    double final_corridor_length = 0;
    double take_off_height = 0;
    double landing_horizontal = 0;
    double touch_down_height = 0;
    double touch_down_offset = 0;
    double ugv_height = 0;
    double landing_time_duration_max = 0;
    double landing_time_duration_min = 0;
    bool landornot = false;
    alan_traj::bezier_constraints btraj_constraints;
    alan_visualization::Polyhedron temp_poly;
    std::vector<alan_visualization::Polyhedron> corridors;
    alan_visualization::PolyhedronArray land_traj_constraint;
    bool land_traj_constraint_initiated = false;

    alan_traj::traj_gen* alan_btraj; //= alan_traj::traj_gen(btraj_info, btraj_constraints, _pub_freq, log_path);
    alan_traj::traj_sampling* alan_btraj_sample;
    alan_traj::optimal_traj optimal_traj_info_obj;
    
    bool plan_traj = false;
    int traj_i = 0;
    double landing_time_total = 0;
    Eigen::Vector3d uav_in_ugv_frame_posi;
    void setRelativePose();
    
    //dynamic-related
    double v_max, a_max;
    double uav_landing_velocity = 0;
    alan_landing_planning::Traj alan_optiTraj;

    bool prerequisite_set = false;
    int sample_square_root = 0;

    // rebuttle backstepping controller
    airo_message::Reference airo_setpt;
    double meetup_thres;


//rotation function
    Eigen::Vector3d q2rpy(Eigen::Quaterniond q) {
        tfScalar yaw, pitch, roll;
        tf::Quaternion q_tf;
        q_tf.setW(q.w());
        q_tf.setX(q.x());
        q_tf.setY(q.y());
        q_tf.setZ(q.z());

        tf::Matrix3x3 mat(q_tf);
        mat.getEulerYPR(yaw, pitch, roll);

        return Eigen::Vector3d(roll, pitch, yaw);
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

//other 
    std::string log_path;
    void config(ros::NodeHandle& _nh);

    //block trajectory (for data collection)
    Eigen::Vector4d set_following_target_pose();
    double set_yaw();
    Eigen::Vector4d set_uav_block_pose();
    bool set_block_traj = true;
    std::vector<std::vector<Eigen::Vector3d>> block_traj_pts;
    int wp_counter_i = 0;
    int traj_counter_j = 0;

public:
    planner_server(ros::NodeHandle& _nh, int pub_freq);
    ~planner_server();

    void mainserver();

};

#endif