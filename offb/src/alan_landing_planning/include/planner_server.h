#include "essential.h"

class planner_server
{
private:
    ros::NodeHandle nh;

    //subscriber
    ros::Subscriber uav_state_sub;

    //publisher
    ros::Publisher local_pos_pub;

    //service
    ros::ServiceClient uav_arming_client;
    ros::ServiceClient uav_set_mode_client;

    //callback functions
    void uavStateCallback(const mavros_msgs::State::ConstPtr& msg);

    //server

    //private variables
    mavros_msgs::SetMode uav_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::State uav_current_state;

    geometry_msgs::PoseStamped pose;



    double last_request;
    

public:
    planner_server(ros::NodeHandle& _nh);
    ~planner_server();

    void mainserver();

};


