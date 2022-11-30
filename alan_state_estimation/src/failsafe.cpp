#include "./include/failsafe.h"

void alan::FailSafeNodelet::uav_msg_callback(const geometry_msgs::PoseStamped::ConstPtr& pose, const geometry_msgs::TwistStamped::ConstPtr& twist)
{
    // ROS_INFO("uav_msg_callback");
    uav_final_odom.header.stamp = ros::Time::now();
    // uav_final_odom.header.stamp.nsec = ros::Time::now().toNSec() - ros::Time::now().toSec() * 1e9;
    
    uav_final_odom.pose.pose.position.x = pose->pose.position.x;
    uav_final_odom.pose.pose.position.y = pose->pose.position.y;
    uav_final_odom.pose.pose.position.z = pose->pose.position.z;
    
    uav_final_odom.pose.pose.orientation.w = pose->pose.orientation.w;
    uav_final_odom.pose.pose.orientation.x = pose->pose.orientation.x;
    uav_final_odom.pose.pose.orientation.y = pose->pose.orientation.y;
    uav_final_odom.pose.pose.orientation.z = pose->pose.orientation.z;

    uav_final_odom.twist.twist.linear.x = twist->twist.linear.x;
    uav_final_odom.twist.twist.linear.y = twist->twist.linear.y;
    uav_final_odom.twist.twist.linear.z = twist->twist.linear.z;

    uav_final_odom.twist.twist.angular.x = twist->twist.angular.x;
    uav_final_odom.twist.twist.angular.y = twist->twist.angular.y;
    uav_final_odom.twist.twist.angular.z = twist->twist.angular.z;

    uav_odom_final_pub.publish(uav_final_odom);

    // ros::Duration(2.0).sleep();
}

void alan::FailSafeNodelet::ugv_msg_callback(const geometry_msgs::PoseStamped::ConstPtr& pose, const geometry_msgs::TwistStamped::ConstPtr& twist)
{
    // ROS_INFO("ugv_msg_callback");
    ugv_final_odom.header.stamp = ros::Time::now();
    // ugv_final_odom.header.stamp.nsec = ros::Time::now().toNSec();
    
    ugv_final_odom.pose.pose.position.x = pose->pose.position.x;
    ugv_final_odom.pose.pose.position.y = pose->pose.position.y;
    ugv_final_odom.pose.pose.position.z = pose->pose.position.z;
    
    ugv_final_odom.pose.pose.orientation.w = pose->pose.orientation.w;
    ugv_final_odom.pose.pose.orientation.x = pose->pose.orientation.x;
    ugv_final_odom.pose.pose.orientation.y = pose->pose.orientation.y;
    ugv_final_odom.pose.pose.orientation.z = pose->pose.orientation.z;

    ugv_final_odom.twist.twist.linear.x = twist->twist.linear.x;
    ugv_final_odom.twist.twist.linear.y = twist->twist.linear.y;
    ugv_final_odom.twist.twist.linear.z = twist->twist.linear.z;

    ugv_final_odom.twist.twist.angular.x = twist->twist.angular.x;
    ugv_final_odom.twist.twist.angular.y = twist->twist.angular.y;
    ugv_final_odom.twist.twist.angular.z = twist->twist.angular.z;

    ugv_odom_final_pub.publish(ugv_final_odom);
}

