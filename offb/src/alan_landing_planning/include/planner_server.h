#include "tools/essential.h"
#include "bezier_lib/traj_gen.h"

#include "alan/AlanPlanner.h"
#include "alan/StateMachine.h"
#include "alan/AlanPlannerMsg.h"


#define IDLE "IDLE"
#define READY "READY"
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
    ros::Subscriber uav_AlanPlannerMsg_sub;

    //publisher
    ros::Publisher local_pos_pub;
    ros::Publisher pub_fsm;

    //service
    ros::ServiceClient uav_arming_client;
    ros::ServiceClient uav_set_mode_client;

    //callback functions
    void uavStateCallback(const mavros_msgs::State::ConstPtr& msg);

    void uavAlanMsgCallback(const alan::AlanPlannerMsg::ConstPtr& msg);

    //fsm
    void fsm_manager();


    bool get_ready();

    bool taking_off();


    //other functions
    void planner_pub();

    // void 

    //server


    //private variables

    string fsm_state = IDLE;
    waypts takeoff_hover_pt = {0,0,1.2};
    mavros_msgs::SetMode uav_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::State uav_current_state;

    alan::AlanPlannerMsg uav_current_AlanPlannerMsg;

    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped uav_traj_desired;
    alan::StateMachine alan_fsm_object;

    double last_request;
    

public:
    planner_server(ros::NodeHandle& _nh);
    ~planner_server();

    void mainserver();

};


