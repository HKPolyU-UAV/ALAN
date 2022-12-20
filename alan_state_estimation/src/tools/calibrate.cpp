#include "../include/tools/essential.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

static geometry_msgs::PoseStamped uav_pose;
static geometry_msgs::PoseStamped led_pose;
static geometry_msgs::PoseStamped ledfront_pose;
static geometry_msgs::PoseStamped cam_pose;
static geometry_msgs::PoseStamped car_pose;

static vector<double> later_deltas;
static vector<double> later_delta2;
bool cal = false;

void uav_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uav_pose = *pose;
}

void led_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    led_pose = *pose;
    cal = true;

    double lateral_delta = ledfront_pose.pose.position.z - led_pose.pose.position.z;

    double vertical_delta = ledfront_pose.pose.position.x - led_pose.pose.position.x;
}

void ledfront_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    ledfront_pose = *pose;
}

void car_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    car_pose = *pose;
    cal = true;

}

void cam_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    cam_pose = *pose;

}

void calculate()
{
    
    double vertical_delta = ledfront_pose.pose.position.x - led_pose.pose.position.x;

    // cout<<lateral_delta<<endl;

    
    
    double later_avg;

    if(cal)
    {
        // double lateral_delta = ledfront_pose.pose.position.z - led_pose.pose.position.z;
        // cout<<lateral_delta<<endl;
        // later_deltas.emplace_back(lateral_delta);
        // cout<<later_deltas.size()<<endl;
        // later_avg = accumulate(later_deltas.begin(), later_deltas.end(), 0.0) / later_deltas.size();
        // cout<<"final average: "<<later_avg<<endl;

    }

    if(cal)
    {
        // double lateral_delta = led_pose.pose.position.z - uav_pose.pose.position.z;

        // cout<<lateral_delta<<endl;
        // later_delta2.emplace_back(lateral_delta);
        // cout<<later_delta2.size()<<endl;
        // later_avg = accumulate(later_delta2.begin(), later_delta2.end(), 0.0) / later_delta2.size();
        // cout<<"final average: "<<later_avg<<endl;

    }

    if(cal)
    {
        // double lateral_delta = ledfront_pose.pose.position.x - led_pose.pose.position.x;

        // cout<<lateral_delta<<endl;
        // later_delta2.emplace_back(lateral_delta);
        // cout<<later_delta2.size()<<endl;
        // later_avg = accumulate(later_delta2.begin(), later_delta2.end(), 0.0) / later_delta2.size();
        // cout<<"final average: "<<later_avg<<endl;

    }

    if(cal)
    {
        // double lateral_delta = led_pose.pose.position.x - uav_pose.pose.position.x;

        // cout<<lateral_delta<<endl;
        // later_delta2.emplace_back(lateral_delta);
        // cout<<later_delta2.size()<<endl;
        // later_avg = accumulate(later_delta2.begin(), later_delta2.end(), 0.0) / later_delta2.size();
        // cout<<"final average: "<<later_avg<<endl;

    }

    if(cal)
    {
        // double lateral_delta = led_pose.pose.position.x - uav_pose.pose.position.x;

        // cout<<lateral_delta<<endl;
        // later_delta2.emplace_back(lateral_delta);
        // cout<<later_delta2.size()<<endl;
        // later_avg = accumulate(later_delta2.begin(), later_delta2.end(), 0.0) / later_delta2.size();
        // cout<<"final average: "<<later_avg<<endl;

    }

    if(cal)
    {
        double lateral_delta = car_pose.pose.position.z - cam_pose.pose.position.z;

        cout<<lateral_delta<<endl;
        later_delta2.emplace_back(lateral_delta);
        cout<<later_delta2.size()<<endl;
        later_avg = accumulate(later_delta2.begin(), later_delta2.end(), 0.0) / later_delta2.size();
        cout<<"final average: "<<later_avg<<endl;
    }


    

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;

    ros::Subscriber uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/vision_pose/pose", 1, &uav_pose_callback);

    ros::Subscriber led_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/vrpn_client_node/gh034_nano_led/pose", 1, &led_pose_callback);

    ros::Subscriber ledfront_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/vrpn_client_node/gh034_nano_led_front/pose", 1, &ledfront_pose_callback);
    
    ros::Subscriber cam_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/vrpn_client_node/gh034_cam/pose", 1, &cam_pose_callback);
    
    ros::Subscriber car_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/vrpn_client_node/gh034_car/pose", 1, &car_pose_callback);

    later_deltas.clear();
    ros::Rate cal_rate(300.0);

    while (ros::ok())
    {   
        // cout<<later_deltas.size()<<endl;

        calculate();  
        cal = false;   

        ros::spinOnce();
        cal_rate.sleep();
    }
    
    

    return 0;
}