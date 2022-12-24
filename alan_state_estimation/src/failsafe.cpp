#include "./include/failsafe.h"

void alan::FailSafeNodelet::led_odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
    led_odom = *odom;
    led_odom_initiated = true;


    if(failsafe_on && uav_vrpn_pose_initiated && uav_vrpn_twist_initiated)
    {
        double delta = 
                pow(
                    led_odom.pose.pose.position.x - uav_vrpn_pose.pose.position.x, 2
                )
              + pow(
                    led_odom.pose.pose.position.y - uav_vrpn_pose.pose.position.y, 2
                )
              + pow(
                    led_odom.pose.pose.position.z - uav_vrpn_pose.pose.position.z, 2
                );

        delta = sqrt(delta);
        
        if(delta < 0.14)
        {
            uav_odom_final_pub.publish(led_odom);
        }
        else
        {
            uav_final_odom.header = uav_vrpn_pose.header;
            uav_final_odom.pose.pose = uav_vrpn_pose.pose;
            uav_final_odom.twist.twist = uav_vrpn_twist.twist;            

            uav_odom_final_pub.publish(uav_final_odom);
        }
            
    }
    else
    {
        uav_odom_final_pub.publish(led_odom);
    }    
}

void alan::FailSafeNodelet::uav_vrpn_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uav_vrpn_pose = *pose;
    uav_vrpn_pose_initiated = true;
}

void alan::FailSafeNodelet::uav_vrpn_twist_callback(const geometry_msgs::TwistStamped::ConstPtr& twist)
{
    uav_vrpn_twist = *twist;
    uav_vrpn_twist_initiated = true;
}

void alan::FailSafeNodelet::ugv_vrpn_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    ugv_vrpn_pose = *pose;
    ugv_vrpn_pose_initiated = true;

    if(ugv_vrpn_pose_initiated && ugv_vrpn_twist_initiated)
    {
        ugv_final_odom.header = ugv_vrpn_pose.header;
        ugv_final_odom.pose.pose = ugv_vrpn_pose.pose; 
        ugv_final_odom.twist.twist = ugv_vrpn_twist.twist;

        ugv_odom_final_pub.publish(ugv_final_odom);
    }
}

void alan::FailSafeNodelet::ugv_vrpn_twist_callback(const geometry_msgs::TwistStamped::ConstPtr& twist)
{
    ugv_vrpn_twist = *twist;
    ugv_vrpn_twist_initiated = true;
}


