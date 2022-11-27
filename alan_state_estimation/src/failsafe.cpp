/*
    A CPP file for
    failsafing ALan System.
    Created on 27/11/2022
    (c) pattylo
    from the RCUAS of Hong Kong Polytechnic University
*/

/**
 * \file failsafe.cpp
 * \brief a failsafe gate for ALan System
 */

#include "./include/tools/essential.h"

static nav_msgs::Odometry uav_final_odom;

static geometry_msgs::PoseStamped uav_vrpn_pose;
static geometry_msgs::Twist uav_vrpn_twist;

static geometry_msgs::PoseStamped uav_led_pose;
static geometry_msgs::Twist uav_led_twist;


static bool uav_vrpn_pose_initiated = false;
static bool uav_vrpn_twist_initiated = false;
static bool uav_led_pose_initiated = false;

static double failsafe_threshold = 0;


void uav_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uav_vrpn_pose = *pose;  
    uav_led_pose_initiated = true;
}

void uav_twist_callback(const geometry_msgs::Twist::ConstPtr& twist)
{
    uav_vrpn_twist = *twist;
    uav_vrpn_twist_initiated = true;
}

void led_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    uav_led_pose = *pose;
}

void led_twist_callback(const geometry_msgs::Twist::ConstPtr &pose)
{
    uav_led_twist = *pose;
}

void set_uav_final_odom()
{
    double deltax = uav_vrpn_pose.pose.position.x - uav_led_pose.pose.position.x;
    double deltay = uav_vrpn_pose.pose.position.y - uav_led_pose.pose.position.y;
    double deltaz = uav_vrpn_pose.pose.position.z - uav_led_pose.pose.position.z;

    double delta_total = sqrt(
        pow(deltax, 2) +
        pow(deltay, 2) +
        pow(deltaz, 2)  
    );

    if(delta_total > failsafe_threshold)
    {
        uav_final_odom.pose.pose.position.x = uav_vrpn_pose.pose.position.x;
        uav_final_odom.pose.pose.position.y = uav_vrpn_pose.pose.position.y;
        uav_final_odom.pose.pose.position.z = uav_vrpn_pose.pose.position.z;

        uav_final_odom.pose.pose.orientation.w = uav_vrpn_pose.pose.orientation.w;
        uav_final_odom.pose.pose.orientation.x = uav_vrpn_pose.pose.orientation.x;
        uav_final_odom.pose.pose.orientation.y = uav_vrpn_pose.pose.orientation.y;
        uav_final_odom.pose.pose.orientation.z = uav_vrpn_pose.pose.orientation.z;        

        uav_final_odom.twist.twist.linear.x = uav_vrpn_twist.linear.x;
        uav_final_odom.twist.twist.linear.y = uav_vrpn_twist.linear.y;
        uav_final_odom.twist.twist.linear.z = uav_vrpn_twist.linear.z;

        uav_final_odom.twist.twist.angular.x = uav_vrpn_twist.angular.x;
        uav_final_odom.twist.twist.angular.y = uav_vrpn_twist.angular.y;
        uav_final_odom.twist.twist.angular.z = uav_vrpn_twist.angular.z;
        
    }
    else
    {
        uav_final_odom.pose.pose.position.x = uav_led_pose.pose.position.x;
        uav_final_odom.pose.pose.position.y = uav_led_pose.pose.position.y;
        uav_final_odom.pose.pose.position.z = uav_led_pose.pose.position.z;

        uav_final_odom.pose.pose.orientation.w = uav_led_pose.pose.orientation.w;
        uav_final_odom.pose.pose.orientation.x = uav_led_pose.pose.orientation.x;
        uav_final_odom.pose.pose.orientation.y = uav_led_pose.pose.orientation.y;
        uav_final_odom.pose.pose.orientation.z = uav_led_pose.pose.orientation.z;        

        uav_final_odom.twist.twist.linear.x = uav_led_twist.linear.x;
        uav_final_odom.twist.twist.linear.y = uav_led_twist.linear.y;
        uav_final_odom.twist.twist.linear.z = uav_led_twist.linear.z;

        uav_final_odom.twist.twist.angular.x = uav_led_twist.angular.x;
        uav_final_odom.twist.twist.angular.y = uav_led_twist.angular.y;
        uav_final_odom.twist.twist.angular.z = uav_led_twist.angular.z;
        
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "failsafe");
    ros::NodeHandle nh;

    nh.getParam("/alan_master/failsafe_threshold", failsafe_threshold);

    ros::Subscriber vrpn_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("/vrpn_client_node/gh034_nano/pose", 1, uav_pose_callback);

    ros::Subscriber vrpn_twist_sub = nh.subscribe<geometry_msgs::Twist>
                    ("/vrpn_client_node/gh034_nano/twist", 1, uav_twist_callback);
    
    ros::Subscriber led_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("/alan_state_estimation/LED/pose", 1, led_pose_callback);
    
    ros::Subscriber led_twist_sub = nh.subscribe<geometry_msgs::Twist>
                    ("/alan_state_estimation/LED/twist", 1, led_twist_callback);

    

    ros::Publisher uav_pose_final_pub = nh.advertise<nav_msgs::Odometry>
                    ("/uav/alan_estimation/final_odom", 1, true);
    

    ros::Rate failsafeRate(60.0);

    while (ros::ok())
    {

        if(uav_vrpn_pose_initiated && uav_vrpn_twist_initiated)
            set_uav_final_odom();

        uav_pose_final_pub.publish(uav_final_odom);

        failsafeRate.sleep();        
    }
    


    return 0;
}