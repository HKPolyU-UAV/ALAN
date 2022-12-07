#include "../include/tools/essential.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

static geometry_msgs::PoseStamped uav_pose;
static geometry_msgs::PoseStamped led_pose;
static geometry_msgs::PoseStamped ledfront_pose;

static vector<double> later_deltas;
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

void calculate()
{
    double lateral_delta = ledfront_pose.pose.position.z - led_pose.pose.position.z;

    double vertical_delta = ledfront_pose.pose.position.x - led_pose.pose.position.x;

    // cout<<lateral_delta<<endl;

    later_deltas.emplace_back(lateral_delta);
    
    double later_avg;

    if(later_deltas.size() != 0 && cal)
    {
        cout<<later_deltas.size()<<endl;
        later_avg = accumulate(later_deltas.begin(), later_deltas.end(), 0.0) / later_deltas.size();
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

    later_deltas.clear();

    while (ros::ok())
    {        
        ros::spinOnce();
        calculate();
        cal = false;
    }
    
    

    return 0;
}