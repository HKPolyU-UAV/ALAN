/*
    A header HPP file for
    RViz tool package for UAV, UGV, UUV (under constrution) and camera.
    Created on 23/11/2022
    by Patty LO 
    from the RCUAS of Hong Kong Polytechnic University
*/

#ifndef RVIZ_VEHICLE_H
#define RVIZ_VEHICLE_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <iostream>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#define UGV "UGV"
#define UAV "UAV"

class rviz_vehicle
{

private:
    ros::NodeHandle nh;
    std::string _robot_type;
    bool _got_camera;
    Eigen::Isometry3d _c2b;

    ros::Publisher vehicle_marker_pub;
    
    visualization_msgs::Marker robot_marker_temp;
    visualization_msgs::MarkerArray robot_marker_array;

    geometry_msgs::Point posi_temp;

    inline Eigen::Vector3d q2rpy(Eigen::Quaterniond q) {
        return q.toRotationMatrix().eulerAngles(2,1,0);
    };

    inline Eigen::Quaterniond rpy2q(Eigen::Vector3d rpy){
        Eigen::AngleAxisd rollAngle(rpy(0), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(rpy(1), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(rpy(2), Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

        return q;

    };

    inline Eigen::Vector3d q_rotate_vector(Eigen::Quaterniond q, Eigen::Vector3d v){
        return q * v;
    }

    Eigen::Quaterniond q_temp;
    Eigen::Quaterniond q045 = rpy2q(Eigen::Vector3d(0,0,M_PI * 1 / 4));
    Eigen::Quaterniond q135 = rpy2q(Eigen::Vector3d(0,0,M_PI * 3 / 4));
    Eigen::Quaterniond q225 = rpy2q(Eigen::Vector3d(0,0,M_PI * 5 / 4));
    Eigen::Quaterniond q315 = rpy2q(Eigen::Vector3d(0,0,M_PI * 7 / 4));

    Eigen::Quaterniond q_robot;
    Eigen::Translation3d t_robot;

    Eigen::Vector3d p_temp;
    
    

public:
    rviz_vehicle(ros::NodeHandle& _nh, std::string robot_type, bool got_camera, Eigen::VectorXd c2b);
    rviz_vehicle(ros::NodeHandle& _nh, std::string robot_type, bool got_camera);
    rviz_vehicle();

    void rviz_pub_vehicle(geometry_msgs::PoseStamped robot_pose);


    
    ~rviz_vehicle();
};

rviz_vehicle::rviz_vehicle(
    ros::NodeHandle& _nh,
    std::string robot_type,
    bool got_camera,
    Eigen::VectorXd c2b
) : nh(_nh), _robot_type(robot_type), _got_camera(got_camera), _c2b(c2b)
{   
    std::string topicname = "/rviz_vehicle/" + _robot_type;
    vehicle_marker_pub = nh.advertise<visualization_msgs::MarkerArray>(topicname, 1, true);

    robot_marker_temp.header.frame_id = "map";
    // robot_marker_temp.
    robot_marker_temp.header.stamp = ros::Time::now();

    robot_marker_temp.ns = robot_type;    
}

rviz_vehicle::rviz_vehicle(
    ros::NodeHandle& _nh,
    std::string robot_type,
    bool got_camera
): nh(_nh), _robot_type(robot_type), _got_camera(got_camera)
{   
    std::string topicname = "/rviz_vehicle/" + robot_type;
    vehicle_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>(topicname, 1, true);

    robot_marker_temp.header.frame_id = "map";
    
    robot_marker_temp.ns = robot_type;

}

void rviz_vehicle::rviz_pub_vehicle(geometry_msgs::PoseStamped robot_pose)
{
    robot_marker_array.markers.clear();

    q_robot.w() = robot_pose.pose.orientation.w;
    q_robot.x() = robot_pose.pose.orientation.x;
    q_robot.y() = robot_pose.pose.orientation.y;
    q_robot.z() = robot_pose.pose.orientation.z;

    t_robot.x() = robot_pose.pose.position.x;
    t_robot.y() = robot_pose.pose.position.y;
    t_robot.z() = robot_pose.pose.position.z;


    if(_robot_type == UAV)
    {
        robot_marker_temp.header.stamp = ros::Time::now();

        //uav base        
        robot_marker_temp.action = visualization_msgs::Marker::ADD;
        robot_marker_temp.id = 0;

        robot_marker_temp.pose.orientation.w = q_robot.w();
        robot_marker_temp.pose.orientation.x = q_robot.x();
        robot_marker_temp.pose.orientation.y = q_robot.y();
        robot_marker_temp.pose.orientation.z = q_robot.z();

        robot_marker_temp.pose.position.x = t_robot.x();
        robot_marker_temp.pose.position.y = t_robot.y();
        robot_marker_temp.pose.position.z = t_robot.z();

        robot_marker_temp.type = visualization_msgs::Marker::CUBE;

        robot_marker_temp.scale.x = 0.2;
        robot_marker_temp.scale.y = 0.2;
        robot_marker_temp.scale.z = 0.1;

        robot_marker_temp.color.a = 1;
        robot_marker_temp.color.r = 1;
        robot_marker_temp.color.b = 0;
        robot_marker_temp.color.g = 1;

        robot_marker_array.markers.emplace_back(robot_marker_temp);




        //uav frame
        robot_marker_temp.action = visualization_msgs::Marker::ADD;

        //045 dg
        q_temp = q_robot * q045;
        robot_marker_temp.id = 1;

        p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(0.1,0.1,0));

        robot_marker_temp.pose.orientation.w = q_temp.w();
        robot_marker_temp.pose.orientation.x = q_temp.x();
        robot_marker_temp.pose.orientation.y = q_temp.y();
        robot_marker_temp.pose.orientation.z = q_temp.z();
        
        robot_marker_temp.pose.position.x = robot_pose.pose.position.x + p_temp(0);
        robot_marker_temp.pose.position.y = robot_pose.pose.position.y + p_temp(1);
        robot_marker_temp.pose.position.z = robot_pose.pose.position.z + p_temp(2);

        robot_marker_temp.type = visualization_msgs::Marker::CUBE;

        robot_marker_temp.scale.x = 0.3;
        robot_marker_temp.scale.y = 0.05;
        robot_marker_temp.scale.z = 0.02;

        robot_marker_temp.color.a = 1;
        robot_marker_temp.color.r = 0;
        robot_marker_temp.color.b = 1;
        robot_marker_temp.color.g = 0;

        robot_marker_array.markers.emplace_back(robot_marker_temp);



        //135 dg
        q_temp = q_robot * q135;
        robot_marker_temp.id = 2;

        p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(-0.1,0.1,0));

        robot_marker_temp.pose.orientation.w = q_temp.w();
        robot_marker_temp.pose.orientation.x = q_temp.x();
        robot_marker_temp.pose.orientation.y = q_temp.y();
        robot_marker_temp.pose.orientation.z = q_temp.z();
        
        robot_marker_temp.pose.position.x = robot_pose.pose.position.x + p_temp(0);
        robot_marker_temp.pose.position.y = robot_pose.pose.position.y + p_temp(1);
        robot_marker_temp.pose.position.z = robot_pose.pose.position.z + p_temp(2);

        robot_marker_temp.type = visualization_msgs::Marker::CUBE;

        robot_marker_temp.scale.x = 0.3;
        robot_marker_temp.scale.y = 0.05;
        robot_marker_temp.scale.z = 0.02;

        robot_marker_temp.color.a = 1;
        robot_marker_temp.color.r = 0;
        robot_marker_temp.color.b = 1;
        robot_marker_temp.color.g = 0;

        robot_marker_array.markers.emplace_back(robot_marker_temp);



        //225 dg
        q_temp = q_robot * q225;
        robot_marker_temp.id = 3;

        p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(-0.1,-0.1,0));

        robot_marker_temp.pose.orientation.w = q_temp.w();
        robot_marker_temp.pose.orientation.x = q_temp.x();
        robot_marker_temp.pose.orientation.y = q_temp.y();
        robot_marker_temp.pose.orientation.z = q_temp.z();
        
        robot_marker_temp.pose.position.x = robot_pose.pose.position.x + p_temp(0);
        robot_marker_temp.pose.position.y = robot_pose.pose.position.y + p_temp(1);
        robot_marker_temp.pose.position.z = robot_pose.pose.position.z + p_temp(2);

        robot_marker_temp.type = visualization_msgs::Marker::CUBE;

        robot_marker_temp.scale.x = 0.3;
        robot_marker_temp.scale.y = 0.05;
        robot_marker_temp.scale.z = 0.02;

        robot_marker_temp.color.a = 1;
        robot_marker_temp.color.r = 0;
        robot_marker_temp.color.b = 1;
        robot_marker_temp.color.g = 0;

        robot_marker_array.markers.emplace_back(robot_marker_temp);



        //315 dg
        q_temp = q_robot * q315;
        robot_marker_temp.id = 4;

        p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(0.1,-0.1,0));

        robot_marker_temp.pose.orientation.w = q_temp.w();
        robot_marker_temp.pose.orientation.x = q_temp.x();
        robot_marker_temp.pose.orientation.y = q_temp.y();
        robot_marker_temp.pose.orientation.z = q_temp.z();
        
        robot_marker_temp.pose.position.x = robot_pose.pose.position.x + p_temp(0);
        robot_marker_temp.pose.position.y = robot_pose.pose.position.y + p_temp(1);
        robot_marker_temp.pose.position.z = robot_pose.pose.position.z + p_temp(2);

        robot_marker_temp.type = visualization_msgs::Marker::CUBE;

        robot_marker_temp.scale.x = 0.3;
        robot_marker_temp.scale.y = 0.05;
        robot_marker_temp.scale.z = 0.02;

        robot_marker_temp.color.a = 1;
        robot_marker_temp.color.r = 0;
        robot_marker_temp.color.b = 1;
        robot_marker_temp.color.g = 0;

        robot_marker_array.markers.emplace_back(robot_marker_temp);




        //propeller
        robot_marker_temp.id = 5;

        p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(0.2,0.2,0));

        robot_marker_temp.pose.orientation.w = q_robot.w();
        robot_marker_temp.pose.orientation.x = q_robot.x();
        robot_marker_temp.pose.orientation.y = q_robot.y();
        robot_marker_temp.pose.orientation.z = q_robot.z();
        
        robot_marker_temp.pose.position.x = robot_pose.pose.position.x + p_temp(0);
        robot_marker_temp.pose.position.y = robot_pose.pose.position.y + p_temp(1);
        robot_marker_temp.pose.position.z = robot_pose.pose.position.z + p_temp(2);

        robot_marker_temp.type = visualization_msgs::Marker::CYLINDER;

        robot_marker_temp.scale.x = 0.32;
        robot_marker_temp.scale.y = 0.32;
        robot_marker_temp.scale.z = 0.01;

        robot_marker_temp.color.a = 0.64;
        robot_marker_temp.color.r = 0;
        robot_marker_temp.color.b = 0;
        robot_marker_temp.color.g = 1;

        robot_marker_array.markers.emplace_back(robot_marker_temp);


        robot_marker_temp.id = 6;

        p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(-0.2,0.2,0));

        robot_marker_temp.pose.orientation.w = q_robot.w();
        robot_marker_temp.pose.orientation.x = q_robot.x();
        robot_marker_temp.pose.orientation.y = q_robot.y();
        robot_marker_temp.pose.orientation.z = q_robot.z();
        
        robot_marker_temp.pose.position.x = robot_pose.pose.position.x + p_temp(0);
        robot_marker_temp.pose.position.y = robot_pose.pose.position.y + p_temp(1);
        robot_marker_temp.pose.position.z = robot_pose.pose.position.z + p_temp(2);

        robot_marker_temp.type = visualization_msgs::Marker::CYLINDER;

        robot_marker_temp.scale.x = 0.32;
        robot_marker_temp.scale.y = 0.32;
        robot_marker_temp.scale.z = 0.01;

        robot_marker_temp.color.a = 0.64;
        robot_marker_temp.color.r = 0;
        robot_marker_temp.color.b = 0;
        robot_marker_temp.color.g = 1;

        robot_marker_array.markers.emplace_back(robot_marker_temp);

        robot_marker_temp.id = 7;

        p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(-0.2,-0.2,0));

        robot_marker_temp.pose.orientation.w = q_robot.w();
        robot_marker_temp.pose.orientation.x = q_robot.x();
        robot_marker_temp.pose.orientation.y = q_robot.y();
        robot_marker_temp.pose.orientation.z = q_robot.z();
        
        robot_marker_temp.pose.position.x = robot_pose.pose.position.x + p_temp(0);
        robot_marker_temp.pose.position.y = robot_pose.pose.position.y + p_temp(1);
        robot_marker_temp.pose.position.z = robot_pose.pose.position.z + p_temp(2);

        robot_marker_temp.type = visualization_msgs::Marker::CYLINDER;

        robot_marker_temp.scale.x = 0.32;
        robot_marker_temp.scale.y = 0.32;
        robot_marker_temp.scale.z = 0.01;

        robot_marker_temp.color.a = 0.64;
        robot_marker_temp.color.r = 0;
        robot_marker_temp.color.b = 0;
        robot_marker_temp.color.g = 1;

        robot_marker_array.markers.emplace_back(robot_marker_temp);

        robot_marker_temp.id = 8;

        p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(0.2,-0.2,0));

        robot_marker_temp.pose.orientation.w = q_robot.w();
        robot_marker_temp.pose.orientation.x = q_robot.x();
        robot_marker_temp.pose.orientation.y = q_robot.y();
        robot_marker_temp.pose.orientation.z = q_robot.z();
        
        robot_marker_temp.pose.position.x = robot_pose.pose.position.x + p_temp(0);
        robot_marker_temp.pose.position.y = robot_pose.pose.position.y + p_temp(1);
        robot_marker_temp.pose.position.z = robot_pose.pose.position.z + p_temp(2);

        robot_marker_temp.type = visualization_msgs::Marker::CYLINDER;

        robot_marker_temp.scale.x = 0.32;
        robot_marker_temp.scale.y = 0.32;
        robot_marker_temp.scale.z = 0.01;

        robot_marker_temp.color.a = 0.64;
        robot_marker_temp.color.r = 0;
        robot_marker_temp.color.b = 0;
        robot_marker_temp.color.g = 1;

        robot_marker_array.markers.emplace_back(robot_marker_temp);







        //final publish
        // cout<<robot_marker_array.markers.size()<<endl;
        vehicle_marker_pub.publish(robot_marker_array);

    }
    else if(_robot_type == UGV)
    {

    }
    else
    {
        ROS_ERROR("PLEASE CHECK ROBOT TYPE...");
    }

    if(_got_camera)
    {
        
    }
    
    
    

}

rviz_vehicle::rviz_vehicle()
{

}

rviz_vehicle::~rviz_vehicle()
{

}




#endif