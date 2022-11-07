#include "tools/essential.h"
#include "bezier_lib/traj_gen.h"

// #include "alan/AlanPlanner.h"
#include "alan/StateMachine.h"
#include "alan/AlanPlannerMsg.h"


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
    }waypts;

private:
    ros::NodeHandle nh;

    //subscriber
    ros::Subscriber uav_state_sub;
    ros::Subscriber uav_AlanPlannerMsg_sub, ugv_AlanPlannerMsg_sub;

    //publisher
    ros::Publisher local_pos_pub;
    ros::Publisher local_vel_pub;
    ros::Publisher pub_fsm;

    //service
    ros::ServiceClient uav_arming_client;
    ros::ServiceClient uav_set_mode_client;

    //callback functions
    void uavStateCallback(const mavros_msgs::State::ConstPtr& msg);

    void uavAlanMsgCallback(const alan::AlanPlannerMsg::ConstPtr& msg);

    void ugvAlanMsgCallback(const alan::AlanPlannerMsg::ConstPtr& msg);

    //fsm
    void fsm_manager();

    bool get_ready();

    bool taking_off();

    bool go_to_rendezvous_pt_and_follow();

    bool land();

    bool shutdown();

    //other functions
    void planner_pub();

     


    // void 

    //server


    //private variables

    string fsm_state = IDLE;
    waypts takeoff_hover_pt = {0,0,1.2};
    
    Eigen::Isometry3d uavOdomPose, ugvOdomPose;

    mavros_msgs::SetMode uav_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::State uav_current_state;

    alan::AlanPlannerMsg uav_current_AlanPlannerMsg, ugv_current_AlanPlannerMsg;

    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped uav_traj_pose_desired;
    geometry_msgs::Twist uav_traj_twist_desired;
    alan::StateMachine alan_fsm_object;

    Eigen::Vector4d pid_controller(Eigen::Vector4d pose, Eigen::Vector4d setpoint);

    Eigen::Vector4d uav_traj_pose, ugv_traj_pose, target_traj_pose, ugv_target_traj_pose;
    
    double last_request;

    double pid_last_request = 0;

    bool print_or_not = true;

    double kp, ki, kd;
    
    Eigen::Vector4d last_error, integral;

    double uav_v_max = 1.5;

    

public:
    planner_server(ros::NodeHandle& _nh);
    ~planner_server();

    void mainserver();

};