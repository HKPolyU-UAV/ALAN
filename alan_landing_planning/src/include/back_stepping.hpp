#ifndef BACKSTEPPING_H
#define BACKSTEPPING_H

#include <iostream>
#include "tools/essential.h"
#include <ros_utilities/ros_utilities.h>
#include "airo_message/ReferencePreview.h"

class BACKSTEPPING : private RosUtilities{
    private:
        double hover_thrust;
        double k_x1,k_x2,k_y1,k_y2,k_z1,k_z2;
        double e_x1,e_x2,e_y1,e_y2,e_z1,e_z2,u_x,u_y,a_z;
        Eigen::Vector3d ref_euler,current_euler,target_euler;

        mavros_msgs::AttitudeTarget attitude_target;


    public:
        BACKSTEPPING::BACKSTEPPING(ros::NodeHandle& nh);
        void backstepping_config();

        mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::ReferenceStamped&, const sensor_msgs::BatteryState&);
        double get_hover_thrust();
};

BACKSTEPPING::BACKSTEPPING(ros::NodeHandle& nh){
    
    // Get Parameters
    nh.getParam("airo_control_node/backstepping/hover_thrust",hover_thrust);
    nh.getParam("airo_control_node/backstepping/k_x1",k_x1);
    nh.getParam("airo_control_node/backstepping/k_x2",k_x2);
    nh.getParam("airo_control_node/backstepping/k_y1",k_y1);  
    nh.getParam("airo_control_node/backstepping/k_y2",k_y2);
    nh.getParam("airo_control_node/backstepping/k_z1",k_z1);
    nh.getParam("airo_control_node/backstepping/k_z2",k_z2);
} 

mavros_msgs::AttitudeTarget BACKSTEPPING::solve(
    const geometry_msgs::PoseStamped& current_pose, 
    const geometry_msgs::TwistStamped& current_twist, 
    const geometry_msgs::AccelStamped& current_accel, 
    const airo_message::Reference& ref
)
{  
    // current_pose.pose.orientation

    current_euler = q2rpy(
        poseRmsg_to_Q(
            current_pose.pose.orientation
        )
    );

    // ref.ref.pose.orientation
    ref_euler = q2rpy(
        poseRmsg_to_Q(
            // ref.ref.pose.orientation
            ref.pose.orientation
        )
    );

    // Altitude Control
    e_z1 = ref.ref.pose.position.z - current_pose.pose.position.z;
    e_z2 = current_twist.twist.linear.z - ref.ref.twist.linear.z - param.k_z1*e_z1;
    attitude_target.thrust = param.hover_thrust/(g*cos(current_euler.x())*cos(current_euler.y())) * (e_z1+g+ref.ref.accel.linear.z-param.k_z1*(e_z2 + param.k_z1*e_z1)-param.k_z2*e_z2);
    if (attitude_target.thrust > 1.0){
        attitude_target.thrust = 1.0;
    }
    else if (attitude_target.thrust < 0.0){
        attitude_target.thrust = 0.0;
    }

    // X Translation Control
    e_x1 = ref.ref.pose.position.x - current_pose.pose.position.x;
    e_x2 = current_twist.twist.linear.x - ref.ref.twist.linear.x - param.k_x1*e_x1;
    u_x = param.hover_thrust/(attitude_target.thrust*g)*(e_x1+ref.ref.accel.linear.x-param.k_x1*(e_x2+param.k_x1*e_x1)-param.k_x2*e_x2);

    // Y Translation Control
    e_y1 = ref.ref.pose.position.y - current_pose.pose.position.y;
    e_y2 = current_twist.twist.linear.y - ref.ref.twist.linear.y - param.k_y1*e_y1;
    u_y = param.hover_thrust/(attitude_target.thrust*g)*(e_y1+ref.ref.accel.linear.y-param.k_y1*(e_y2+param.k_y1*e_y1)-param.k_y2*e_y2);

    // Calculate Targer Eulers
    target_euler.x() = asin(u_x*sin(ref_euler.z()) - u_y*cos(ref_euler.z()));
    target_euler.y() = asin((u_x*cos(ref_euler.z()) + u_y*sin(ref_euler.z()))/cos(target_euler.x()));
    target_euler.z() = ref_euler.z();
    attitude_target.orientation = rpy2q(target_euler);

    if (param.pub_debug){
        pub_debug();
    }

    return attitude_target;
}

double BACKSTEPPING::get_hover_thrust(){
    return param.hover_thrust;
}

#endif