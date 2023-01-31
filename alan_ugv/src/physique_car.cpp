/*
    physique_car for simulation in RVIZ
    do not launch this in VICON!
*/

#include "./include/essential.h"
#include "./include/RosTopicConfigs.h"
#include "./include/ugvpath.hpp"
#include <cmath>
#include <random>
#include <gazebo_msgs/ModelStates.h>
#include <type_traits>

using namespace std;

#define TURN 0
#define GOST 1

static Eigen::VectorXd current_traj_wp;

static nav_msgs::Odometry physique_car_odom;

static geometry_msgs::PoseStamped physique_car_pose;
static geometry_msgs::TwistStamped physique_car_twist;
static sensor_msgs::Imu physique_car_imu;
static geometry_msgs::PoseStamped virual_pose;

static vector<vector<string>> ugvpaths;

static int global_wp_counter = 0;
static int global_traj_counter = 0;

static double max_vel_lin = 0;
static double max_vel_ang = 0;
static int pub_freq = 0;
static std::string environs;
static std::string GAZEBO_ = "GAZEBO";

static int fsm = TURN;

static bool cal_new_traj = false;

static vector<double> yaw_traj;
static vector<Eigen::Vector2d> lin_traj;

static bool notyetfinish = false;


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

void test_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    msg->name.size();
    msg->pose[2].position.x;
    msg->pose[2].orientation.w;
    
}


static Eigen::Vector3d physique_car_state; //x y yaw

static int indi = 0;
static int state_i = 0;


template<typename T>
void physique_car_state_callback_impl(const T& msg, std::true_type)
{
    gazebo_msgs::ModelStates state = *msg;

    if(indi == 0)
    {
        for(int i = 0; i < state.name.size(); i++)
        {
            if(state.name[i] == "scout/")
            {
                state_i = i;
                indi++;
            }
        }
    }

    physique_car_state(0) = state.pose[state_i].position.x;
    physique_car_state(1) = state.pose[state_i].position.y;

    physique_car_state(2) = q2rpy(Eigen::Quaterniond(
            state.pose[state_i].orientation.w,
            state.pose[state_i].orientation.x,
            state.pose[state_i].orientation.y,
            state.pose[state_i].orientation.z
        )
    ).x();

    std::cout<<physique_car_state(2)<<std::endl;
    
    std::cout<<"gazebo"<<std::endl;
    
}

template<typename T>
void physique_car_state_callback_impl(const T& msg, std::false_type)
{
    geometry_msgs::PoseStamped state = *msg;

    physique_car_state(0) = state.pose.position.x;
    physique_car_state(1) = state.pose.position.y;

    physique_car_state(2) = q2rpy(Eigen::Quaterniond(
            state.pose.orientation.w,
            state.pose.orientation.x,
            state.pose.orientation.y,
            state.pose.orientation.z
        )
    ).x();

    std::cout<<physique_car_state(2)<<std::endl;
    
    std::cout<<"not gazebo"<<std::endl;
        
}

template <typename T>
void physique_car_state_callback(const T& msg)
{
    physique_car_state_callback_impl(msg, std::is_same<T, gazebo_msgs::ModelStates::ConstPtr>());        
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "physique_car");
    ros::NodeHandle nh;

    nh.getParam("/physique_car/max_vel_lin", max_vel_lin);
    nh.getParam("/physique_car/max_vel_ang", max_vel_ang);
    nh.getParam("/physique_car/pub_freq", pub_freq);
    nh.getParam("/physique_car/environs", environs);
    
    // ros::Subscriber 

 
    const char* temp1 = environs.c_str();
    const char* temp2 = GAZEBO_.c_str();

    std::cout<<temp1<<std::endl;
    std::cout<<temp2<<std::endl;
    std::cout<<std::strcmp(temp1, temp2)<<std::endl;
    
    ros::Subscriber physique_car_state_sub;

    if(environs == GAZEBO_)
    {
        physique_car_state_sub = 
            nh.subscribe<gazebo_msgs::ModelStates>(
                "/gazebo/model_states", 
                1, 
                physique_car_state_callback<gazebo_msgs::ModelStates::ConstPtr>
            );

    }
    else
    {
        physique_car_state_sub = 
            nh.subscribe<geometry_msgs::PoseStamped>(
                "/uav/mavros/vision_pose/pose", 
                1, 
                physique_car_state_callback<geometry_msgs::PoseStamped::ConstPtr>
            );

    }
    
    // ros::Publisher physique_car_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    //                     (configs.getTopicName(POSE_PUB_TOPIC_A), 1, true);

    // ros::Publisher physique_car_twist_pub = nh.advertise<geometry_msgs::TwistStamped>
    //                     (configs.getTopicName(TWIST_PUB_TOPIC_A), 1, true);
    
    // ros::Publisher physique_car_imu_pub = nh.advertise<sensor_msgs::Imu>
    //                     (configs.getTopicName(IMU_PUB_TOPIC_A), 1, true);
    

    ros::Rate physique_car_rate(pub_freq);

    ugv::ugv_traj_info_circle circle_traj_info;

    ugv::ugvpath<ugv::ugv_traj_info_circle> circle_traj(circle_traj_info);
    

    cal_new_traj = true;
    current_traj_wp.resize(6);

    current_traj_wp(0) = 0.0;
    current_traj_wp(1) = 0.0;
    current_traj_wp(2) = 0.0;

    current_traj_wp(3) = 0.0;
    current_traj_wp(4) = 0.0;
    current_traj_wp(5) = 0.0;


    while (ros::ok())    
    {         
        // set_current_traj_wp(current_traj_wp);
        // set_virtual_pose_twist_imu(current_traj_wp);

        // physique_car_pose_pub.publish(physique_car_pose);
        // physique_car_twist_pub.publish(physique_car_twist);
        // physique_car_imu_pub.publish(physique_car_imu);
                  
        ros::spinOnce();
        physique_car_rate.sleep();
        /* code */
    }

    return 0;    
}