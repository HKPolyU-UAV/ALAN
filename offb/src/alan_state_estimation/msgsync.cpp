#ifndef IMU_H
#define IMU_H

#include "include/msgsync.h"

void alan::MsgSyncNodelet::uav_msg_callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::Imu::ConstPtr& imu)
{
    uav_odom = *odom;
    uav_imu = *imu;

    uav_alan_msg.position.x = uav_odom.pose.pose.position.x;
    uav_alan_msg.position.y = uav_odom.pose.pose.position.y;
    uav_alan_msg.position.z = uav_odom.pose.pose.position.z;

    uav_alan_msg.velocity.x = uav_odom.twist.twist.linear.x;
    uav_alan_msg.velocity.y = uav_odom.twist.twist.linear.y;
    uav_alan_msg.velocity.z = uav_odom.twist.twist.linear.z;

    Eigen::Translation3d t_(
        uav_odom.pose.pose.position.x, 
        uav_odom.pose.pose.position.y, 
        uav_odom.pose.pose.position.z
        );

    Eigen::Quaterniond q_(
        uav_odom.pose.pose.orientation.w,
        uav_odom.pose.pose.orientation.x,
        uav_odom.pose.pose.orientation.y,
        uav_odom.pose.pose.orientation.z
        );
    
    uavOdomPose = t_ * q_;

    uav_odom_initiated = true;
    
    acc_body(0) = uav_imu.linear_acceleration.x;
    acc_body(1) = uav_imu.linear_acceleration.y;
    acc_body(2) = uav_imu.linear_acceleration.z;

    acc_world = uavOdomPose.rotation() * acc_body;
    uav_alan_msg.acceleration.x = acc_body(0);
    uav_alan_msg.acceleration.y = acc_body(1);
    uav_alan_msg.acceleration.z = acc_body(2);

    uav_alan_msg.orientation.ow = uav_odom.pose.pose.orientation.w;
    uav_alan_msg.orientation.ox = uav_odom.pose.pose.orientation.x;
    uav_alan_msg.orientation.oy = uav_odom.pose.pose.orientation.y;
    uav_alan_msg.orientation.oz = uav_odom.pose.pose.orientation.z;

    // uav_alan_msg
    uav_alan_msg.frame = "world";

    pub_AlanPlannerMsg.publish(uav_alan_msg);

}

void alan::MsgSyncNodelet::ugv_msg_callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::Imu::ConstPtr& imu)
{
    uav_odom = *odom;
    uav_imu = *imu;

    uav_alan_msg.position.x = uav_odom.pose.pose.position.x;
    uav_alan_msg.position.y = uav_odom.pose.pose.position.y;
    uav_alan_msg.position.z = uav_odom.pose.pose.position.z;

    uav_alan_msg.velocity.x = uav_odom.twist.twist.linear.x;
    uav_alan_msg.velocity.y = uav_odom.twist.twist.linear.y;
    uav_alan_msg.velocity.z = uav_odom.twist.twist.linear.z;

    Eigen::Translation3d t_(
        uav_odom.pose.pose.position.x, 
        uav_odom.pose.pose.position.y, 
        uav_odom.pose.pose.position.z
        );

    Eigen::Quaterniond q_(
        uav_odom.pose.pose.orientation.w,
        uav_odom.pose.pose.orientation.x,
        uav_odom.pose.pose.orientation.y,
        uav_odom.pose.pose.orientation.z
        );
    
    uavOdomPose = t_ * q_;

    uav_odom_initiated = true;
    
    acc_body(0) = uav_imu.linear_acceleration.x;
    acc_body(1) = uav_imu.linear_acceleration.y;
    acc_body(2) = uav_imu.linear_acceleration.z;

    acc_world = uavOdomPose.rotation() * acc_body;
    uav_alan_msg.acceleration.x = acc_body(0);
    uav_alan_msg.acceleration.y = acc_body(1);
    uav_alan_msg.acceleration.z = acc_body(2);

    uav_alan_msg.orientation.ow = uav_odom.pose.pose.orientation.w;
    uav_alan_msg.orientation.ox = uav_odom.pose.pose.orientation.x;
    uav_alan_msg.orientation.oy = uav_odom.pose.pose.orientation.y;
    uav_alan_msg.orientation.oz = uav_odom.pose.pose.orientation.z;

    // uav_alan_msg
    uav_alan_msg.frame = "world";

    pub_AlanPlannerMsg.publish(uav_alan_msg);

}
#endif