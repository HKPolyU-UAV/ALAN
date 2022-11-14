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
    // cout<<uavOdomPose.data()<<endl;

    uav_odom_initiated = true;
    
    uav_acc_body(0) = uav_imu.linear_acceleration.x;
    uav_acc_body(1) = uav_imu.linear_acceleration.y;
    uav_acc_body(2) = uav_imu.linear_acceleration.z;

    uav_acc_world = uavOdomPose.rotation() * uav_acc_body;
    uav_alan_msg.acceleration.x = uav_acc_body(0);
    uav_alan_msg.acceleration.y = uav_acc_body(1);
    uav_alan_msg.acceleration.z = uav_acc_body(2) - 9.8066;

    uav_alan_msg.orientation.ow = uav_odom.pose.pose.orientation.w;
    uav_alan_msg.orientation.ox = uav_odom.pose.pose.orientation.x;
    uav_alan_msg.orientation.oy = uav_odom.pose.pose.orientation.y;
    uav_alan_msg.orientation.oz = uav_odom.pose.pose.orientation.z;

    // uav_alan_msg
    uav_alan_msg.frame = "world";

    uav_pub_AlanPlannerMsg.publish(uav_alan_msg);

}

void alan::MsgSyncNodelet::ugv_msg_callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::Imu::ConstPtr& imu)
{
    ugv_odom = *odom;
    ugv_imu = *imu;

    ugv_alan_msg.position.x = ugv_odom.pose.pose.position.x;
    ugv_alan_msg.position.y = ugv_odom.pose.pose.position.y;
    ugv_alan_msg.position.z = ugv_odom.pose.pose.position.z;

    ugv_alan_msg.velocity.x = ugv_odom.twist.twist.linear.x;
    ugv_alan_msg.velocity.y = ugv_odom.twist.twist.linear.y;
    ugv_alan_msg.velocity.z = ugv_odom.twist.twist.linear.z;

    Eigen::Translation3d t_(
        ugv_odom.pose.pose.position.x, 
        ugv_odom.pose.pose.position.y, 
        ugv_odom.pose.pose.position.z
        );

    Eigen::Quaterniond q_(
        ugv_odom.pose.pose.orientation.w,
        ugv_odom.pose.pose.orientation.x,
        ugv_odom.pose.pose.orientation.y,
        ugv_odom.pose.pose.orientation.z
        );
    
    ugvOdomPose = t_ * q_;

    ugv_odom_initiated = true;
    
    ugv_acc_body(0) = ugv_imu.linear_acceleration.x;
    ugv_acc_body(1) = ugv_imu.linear_acceleration.y;
    ugv_acc_body(2) = ugv_imu.linear_acceleration.z;

    ugv_acc_world = ugvOdomPose.rotation() * ugv_acc_body;
    ugv_alan_msg.acceleration.x = ugv_acc_body(0);
    ugv_alan_msg.acceleration.y = ugv_acc_body(1);
    ugv_alan_msg.acceleration.z = ugv_acc_body(2) - 9.8066;

    ugv_alan_msg.orientation.ow = ugv_odom.pose.pose.orientation.w;
    ugv_alan_msg.orientation.ox = ugv_odom.pose.pose.orientation.x;
    ugv_alan_msg.orientation.oy = ugv_odom.pose.pose.orientation.y;
    ugv_alan_msg.orientation.oz = ugv_odom.pose.pose.orientation.z;

    // ugv_alan_msg
    ugv_alan_msg.frame = "world";

    ugv_pub_AlanPlannerMsg.publish(ugv_alan_msg);

}

void alan::MsgSyncNodelet::cam_msg_callback(const sensor_msgs::Imu::ConstPtr& imu)
{
    
}
