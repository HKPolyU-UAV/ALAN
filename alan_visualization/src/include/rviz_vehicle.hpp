/*
    This file is part of ALan - the non-robocentric dynamic landing system for quadrotor

    ALan is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ALan is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ALan.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file rviz_vehicle.hpp
 * \date 16/02/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes of a simple RViz robot visualization package of UAV, UGV, UUV
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
    Eigen::VectorXd _c2b;

    ros::Publisher vehicle_marker_pub;
    ros::Publisher vehicle_pose_pub;
    ros::Publisher vehicle_cam_pose_pub;
    
    visualization_msgs::Marker robot_marker_temp;
    visualization_msgs::MarkerArray robot_marker_array;
    visualization_msgs::Marker camera_pyramid;
    visualization_msgs::Marker camera_pyramid_temp;

    Eigen::Vector3d cam_body_pyramidPoint[5]; 
           
    // inline Eigen::Vector3d

    void setupUAV();
    void setupUGV();

    Eigen::Quaterniond q_temp;
    Eigen::Quaterniond q045_yaw = rpy2q(Eigen::Vector3d(0,0,M_PI * 1 / 4));
    Eigen::Quaterniond q135_yaw = rpy2q(Eigen::Vector3d(0,0,M_PI * 3 / 4));
    Eigen::Quaterniond q225_yaw = rpy2q(Eigen::Vector3d(0,0,M_PI * 5 / 4));
    Eigen::Quaterniond q315_yaw = rpy2q(Eigen::Vector3d(0,0,M_PI * 7 / 4));

    Eigen::Quaterniond q090_rol = rpy2q(Eigen::Vector3d(M_PI * 1 / 2,0,0));

    

    Eigen::Quaterniond q_robot;
    Eigen::Translation3d t_robot;

    Eigen::Quaterniond q_c2b;
    Eigen::Vector3d t_c2b;

    Eigen::Vector3d p_temp;

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

    topicname = topicname + "/pose";
    vehicle_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(topicname, 1, true);

    topicname = "/rviz_vehicle/" + _robot_type + "/camera";
    vehicle_cam_pose_pub = nh.advertise<visualization_msgs::Marker>(topicname, 1, true);


    robot_marker_temp.header.frame_id = "world";
    // robot_marker_temp.
    robot_marker_temp.header.stamp = ros::Time::now();

    robot_marker_temp.ns = robot_type;

    if(_got_camera)
    {
        
        cam_body_pyramidPoint[0] = Eigen::Vector3d(0.07, 0.1, 0.07);
        cam_body_pyramidPoint[1] = Eigen::Vector3d(0.07, 0.1, -0.07);
        cam_body_pyramidPoint[2] = Eigen::Vector3d(0.07, -0.1, -0.07);
        cam_body_pyramidPoint[3] = Eigen::Vector3d(0.07, -0.1, 0.07);
        cam_body_pyramidPoint[4] = Eigen::Vector3d(0.0,0.0,0.0);

        q_c2b = rpy2q(
            Eigen::Vector3d(
                _c2b(3),
                _c2b(4),
                _c2b(5)                
            )
        );

        t_c2b = Eigen::Vector3d(
            _c2b(0),
            _c2b(1),
            _c2b(2)
        );

        for(int i = 0; i < 5; i++)
        {
            cam_body_pyramidPoint[i] = 
                q_rotate_vector(q_c2b, cam_body_pyramidPoint[i]) + t_c2b;            

        }

    }
}

rviz_vehicle::rviz_vehicle(
    ros::NodeHandle& _nh,
    std::string robot_type,
    bool got_camera
): nh(_nh), _robot_type(robot_type), _got_camera(got_camera)
{   
    std::string topicname = "/rviz_vehicle/" + robot_type;
    vehicle_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>(topicname, 1, true);

    topicname = topicname + "/pose";
    vehicle_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(topicname, 1, true);

    topicname = "/rviz_vehicle/" + _robot_type + "/camera";
    vehicle_cam_pose_pub = nh.advertise<visualization_msgs::Marker>(topicname, 1, true);

    robot_marker_temp.header.frame_id = "world";
    
    robot_marker_temp.ns = robot_type;

    if(_got_camera)
    {
        
        cam_body_pyramidPoint[0] = Eigen::Vector3d(0.07, 0.1, 0.07);
        cam_body_pyramidPoint[1] = Eigen::Vector3d(0.07, 0.1, -0.07);
        cam_body_pyramidPoint[2] = Eigen::Vector3d(0.07, -0.1, -0.07);
        cam_body_pyramidPoint[3] = Eigen::Vector3d(0.07, -0.1, 0.07);
        cam_body_pyramidPoint[4] = Eigen::Vector3d(0.0,0.0,0.0);

        q_c2b = rpy2q(
            Eigen::Vector3d(
                _c2b(3),
                _c2b(4),
                _c2b(5)                
            )
        );

        t_c2b = Eigen::Vector3d(
            _c2b(0),
            _c2b(1),
            _c2b(2)
        );

        for(int i = 0; i < 5; i++)
        {
            cam_body_pyramidPoint[i] = 
                q_rotate_vector(q_c2b, cam_body_pyramidPoint[i]) + t_c2b;

        }

    }
        
}

void rviz_vehicle::rviz_pub_vehicle(geometry_msgs::PoseStamped robot_pose)
{
    robot_marker_array.markers.clear();
    camera_pyramid.points.clear();

    q_robot.w() = robot_pose.pose.orientation.w;
    q_robot.x() = robot_pose.pose.orientation.x;
    q_robot.y() = robot_pose.pose.orientation.y;
    q_robot.z() = robot_pose.pose.orientation.z;

    t_robot.x() = robot_pose.pose.position.x;
    t_robot.y() = robot_pose.pose.position.y;
    t_robot.z() = robot_pose.pose.position.z;


    if(_robot_type == UAV)
    {
        setupUAV();        
        //final publish
        vehicle_marker_pub.publish(robot_marker_array);
        robot_pose.header.frame_id = "world";
        vehicle_pose_pub.publish(robot_pose);

    }
    else if(_robot_type == UGV)
    {
        setupUGV();
        //final publish
        vehicle_marker_pub.publish(robot_marker_array);
        robot_pose.header.frame_id = "world";
        vehicle_pose_pub.publish(robot_pose);
    }
    else
    {
        ROS_ERROR("PLEASE CHECK ROBOT TYPE...");
    }


    if(_got_camera)
    {                
        camera_pyramid.header.frame_id = "world";
        camera_pyramid.header.stamp = ros::Time::now();

        camera_pyramid.ns = "camera_pyramid";
        camera_pyramid.action = visualization_msgs::Marker::ADD;
        camera_pyramid.id = 0;
        

        camera_pyramid.type = visualization_msgs::Marker::LINE_LIST;

        camera_pyramid.pose.orientation.w = 1;
        
        camera_pyramid.color.r = 0.3f;
        camera_pyramid.color.b = 0.5f;
        camera_pyramid.color.a = 1.0;
        camera_pyramid.scale.x = 0.02;

        Eigen::Vector3d cam_world_pyramidPoint[5];

        Eigen::Vector3d cam_world_pyramidPointOrigin 
            = q_rotate_vector(q_robot, cam_body_pyramidPoint[4]);

        geometry_msgs::Point pyramidPointDraw[5];            


        for(int i = 0; i < 5; i++)
        {
            cam_world_pyramidPoint[i] 
                = q_rotate_vector(q_robot, cam_body_pyramidPoint[i]);

            pyramidPointDraw[i].x = cam_world_pyramidPoint[i].x() + t_robot.x();
            pyramidPointDraw[i].y = cam_world_pyramidPoint[i].y() + t_robot.y();
            pyramidPointDraw[i].z = cam_world_pyramidPoint[i].z() + t_robot.z();

            // camera_pyramid.points.push_back(pyramidPointDrawOrigin);
            // camera_pyramid.points.push_back(pyramidPointDraw[i]);


        }

        camera_pyramid.points.push_back(pyramidPointDraw[4]);
        camera_pyramid.points.push_back(pyramidPointDraw[0]);

        camera_pyramid.points.push_back(pyramidPointDraw[4]);
        camera_pyramid.points.push_back(pyramidPointDraw[1]);

        camera_pyramid.points.push_back(pyramidPointDraw[4]);
        camera_pyramid.points.push_back(pyramidPointDraw[2]);

        camera_pyramid.points.push_back(pyramidPointDraw[4]);
        camera_pyramid.points.push_back(pyramidPointDraw[3]);

        camera_pyramid.points.push_back(pyramidPointDraw[0]);
        camera_pyramid.points.push_back(pyramidPointDraw[1]);
        camera_pyramid.points.push_back(pyramidPointDraw[1]);
        camera_pyramid.points.push_back(pyramidPointDraw[2]);
        camera_pyramid.points.push_back(pyramidPointDraw[2]);
        camera_pyramid.points.push_back(pyramidPointDraw[3]);
        camera_pyramid.points.push_back(pyramidPointDraw[3]);
        camera_pyramid.points.push_back(pyramidPointDraw[0]);


        vehicle_cam_pose_pub.publish(camera_pyramid);        
    }


}



void rviz_vehicle::setupUAV()
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

    robot_marker_temp.scale.x = 0.1;
    robot_marker_temp.scale.y = 0.1;
    robot_marker_temp.scale.z = 0.05;

    robot_marker_temp.color.a = 1;
    robot_marker_temp.color.r = 1;
    robot_marker_temp.color.b = 0;
    robot_marker_temp.color.g = 1;

    robot_marker_array.markers.emplace_back(robot_marker_temp);




    //uav frame
    robot_marker_temp.action = visualization_msgs::Marker::ADD;

    //045 dg
    q_temp = q_robot * q045_yaw;
    robot_marker_temp.id = 1;

    p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(0.06,0.06,0));

    robot_marker_temp.pose.orientation.w = q_temp.w();
    robot_marker_temp.pose.orientation.x = q_temp.x();
    robot_marker_temp.pose.orientation.y = q_temp.y();
    robot_marker_temp.pose.orientation.z = q_temp.z();
    
    robot_marker_temp.pose.position.x = t_robot.x() + p_temp(0);
    robot_marker_temp.pose.position.y = t_robot.y() + p_temp(1);
    robot_marker_temp.pose.position.z = t_robot.z() + p_temp(2);

    robot_marker_temp.type = visualization_msgs::Marker::CUBE;

    robot_marker_temp.scale.x = 0.15;
    robot_marker_temp.scale.y = 0.025;
    robot_marker_temp.scale.z = 0.01;

    robot_marker_temp.color.a = 1;
    robot_marker_temp.color.r = 0;
    robot_marker_temp.color.b = 1;
    robot_marker_temp.color.g = 0;

    robot_marker_array.markers.emplace_back(robot_marker_temp);




    //135 dg
    q_temp = q_robot * q135_yaw;
    robot_marker_temp.id = 2;

    p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(-0.06,0.06,0));

    robot_marker_temp.pose.orientation.w = q_temp.w();
    robot_marker_temp.pose.orientation.x = q_temp.x();
    robot_marker_temp.pose.orientation.y = q_temp.y();
    robot_marker_temp.pose.orientation.z = q_temp.z();
    
    robot_marker_temp.pose.position.x = t_robot.x() + p_temp(0);
    robot_marker_temp.pose.position.y = t_robot.y() + p_temp(1);
    robot_marker_temp.pose.position.z = t_robot.z() + p_temp(2);

    robot_marker_temp.type = visualization_msgs::Marker::CUBE;

    robot_marker_temp.scale.x = 0.15;
    robot_marker_temp.scale.y = 0.025;
    robot_marker_temp.scale.z = 0.01;

    robot_marker_temp.color.a = 1;
    robot_marker_temp.color.r = 0;
    robot_marker_temp.color.b = 1;
    robot_marker_temp.color.g = 0;

    robot_marker_array.markers.emplace_back(robot_marker_temp);




    //225 dg
    q_temp = q_robot * q225_yaw;
    robot_marker_temp.id = 3;

    p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(-0.06,-0.06,0));

    robot_marker_temp.pose.orientation.w = q_temp.w();
    robot_marker_temp.pose.orientation.x = q_temp.x();
    robot_marker_temp.pose.orientation.y = q_temp.y();
    robot_marker_temp.pose.orientation.z = q_temp.z();
    
    robot_marker_temp.pose.position.x = t_robot.x() + p_temp(0);
    robot_marker_temp.pose.position.y = t_robot.y() + p_temp(1);
    robot_marker_temp.pose.position.z = t_robot.z() + p_temp(2);

    robot_marker_temp.type = visualization_msgs::Marker::CUBE;

    robot_marker_temp.scale.x = 0.15;
    robot_marker_temp.scale.y = 0.025;
    robot_marker_temp.scale.z = 0.01;

    robot_marker_temp.color.a = 1;
    robot_marker_temp.color.r = 0;
    robot_marker_temp.color.b = 1;
    robot_marker_temp.color.g = 0;

    robot_marker_array.markers.emplace_back(robot_marker_temp);




    //315 dg
    q_temp = q_robot * q315_yaw;
    robot_marker_temp.id = 4;

    p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(0.06,-0.06,0));

    robot_marker_temp.pose.orientation.w = q_temp.w();
    robot_marker_temp.pose.orientation.x = q_temp.x();
    robot_marker_temp.pose.orientation.y = q_temp.y();
    robot_marker_temp.pose.orientation.z = q_temp.z();
    
    robot_marker_temp.pose.position.x = t_robot.x() + p_temp(0);
    robot_marker_temp.pose.position.y = t_robot.y() + p_temp(1);
    robot_marker_temp.pose.position.z = t_robot.z() + p_temp(2);

    robot_marker_temp.type = visualization_msgs::Marker::CUBE;

    robot_marker_temp.scale.x = 0.15;
    robot_marker_temp.scale.y = 0.025;
    robot_marker_temp.scale.z = 0.01;

    robot_marker_temp.color.a = 1;
    robot_marker_temp.color.r = 0;
    robot_marker_temp.color.b = 1;
    robot_marker_temp.color.g = 0;

    robot_marker_array.markers.emplace_back(robot_marker_temp);




    //propeller 1
    robot_marker_temp.id = 5;

    p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(0.12,0.12,0));

    robot_marker_temp.pose.orientation.w = q_robot.w();
    robot_marker_temp.pose.orientation.x = q_robot.x();
    robot_marker_temp.pose.orientation.y = q_robot.y();
    robot_marker_temp.pose.orientation.z = q_robot.z();
    
    robot_marker_temp.pose.position.x = t_robot.x() + p_temp(0);
    robot_marker_temp.pose.position.y = t_robot.y() + p_temp(1);
    robot_marker_temp.pose.position.z = t_robot.z() + p_temp(2);

    robot_marker_temp.type = visualization_msgs::Marker::CYLINDER;

    robot_marker_temp.scale.x = 0.16;
    robot_marker_temp.scale.y = 0.16;
    robot_marker_temp.scale.z = 0.005;

    robot_marker_temp.color.a = 0.64;
    robot_marker_temp.color.r = 0;
    robot_marker_temp.color.b = 0;
    robot_marker_temp.color.g = 1;

    robot_marker_array.markers.emplace_back(robot_marker_temp);




    //propeller 2
    robot_marker_temp.id = 6;

    p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(-0.12,0.12,0));

    robot_marker_temp.pose.orientation.w = q_robot.w();
    robot_marker_temp.pose.orientation.x = q_robot.x();
    robot_marker_temp.pose.orientation.y = q_robot.y();
    robot_marker_temp.pose.orientation.z = q_robot.z();
    
    robot_marker_temp.pose.position.x = t_robot.x() + p_temp(0);
    robot_marker_temp.pose.position.y = t_robot.y() + p_temp(1);
    robot_marker_temp.pose.position.z = t_robot.z() + p_temp(2);

    robot_marker_temp.type = visualization_msgs::Marker::CYLINDER;

    robot_marker_temp.scale.x = 0.16;
    robot_marker_temp.scale.y = 0.16;
    robot_marker_temp.scale.z = 0.005;

    robot_marker_temp.color.a = 0.64;
    robot_marker_temp.color.r = 0;
    robot_marker_temp.color.b = 0;
    robot_marker_temp.color.g = 1;

    robot_marker_array.markers.emplace_back(robot_marker_temp);




    //propeller 3
    robot_marker_temp.id = 7;

    p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(-0.12,-0.12,0));

    robot_marker_temp.pose.orientation.w = q_robot.w();
    robot_marker_temp.pose.orientation.x = q_robot.x();
    robot_marker_temp.pose.orientation.y = q_robot.y();
    robot_marker_temp.pose.orientation.z = q_robot.z();
    
    robot_marker_temp.pose.position.x = t_robot.x() + p_temp(0);
    robot_marker_temp.pose.position.y = t_robot.y() + p_temp(1);
    robot_marker_temp.pose.position.z = t_robot.z() + p_temp(2);

    robot_marker_temp.type = visualization_msgs::Marker::CYLINDER;

    robot_marker_temp.scale.x = 0.16;
    robot_marker_temp.scale.y = 0.16;
    robot_marker_temp.scale.z = 0.005;

    robot_marker_temp.color.a = 0.64;
    robot_marker_temp.color.r = 0;
    robot_marker_temp.color.b = 0;
    robot_marker_temp.color.g = 1;

    robot_marker_array.markers.emplace_back(robot_marker_temp);




    //propeller 4
    robot_marker_temp.id = 8;

    p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(0.12,-0.12,0));

    robot_marker_temp.pose.orientation.w = q_robot.w();
    robot_marker_temp.pose.orientation.x = q_robot.x();
    robot_marker_temp.pose.orientation.y = q_robot.y();
    robot_marker_temp.pose.orientation.z = q_robot.z();
    
    robot_marker_temp.pose.position.x = t_robot.x() + p_temp(0);
    robot_marker_temp.pose.position.y = t_robot.y() + p_temp(1);
    robot_marker_temp.pose.position.z = t_robot.z() + p_temp(2);

    robot_marker_temp.type = visualization_msgs::Marker::CYLINDER;

    robot_marker_temp.scale.x = 0.16;
    robot_marker_temp.scale.y = 0.16;
    robot_marker_temp.scale.z = 0.005;

    robot_marker_temp.color.a = 0.64;
    robot_marker_temp.color.r = 0;
    robot_marker_temp.color.b = 0;
    robot_marker_temp.color.g = 1;

    robot_marker_array.markers.emplace_back(robot_marker_temp);
}

void rviz_vehicle::setupUGV()
{
    robot_marker_temp.header.stamp = ros::Time::now();

    //ugv base        
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

    robot_marker_temp.scale.x = 0.5;
    robot_marker_temp.scale.y = 0.5;
    robot_marker_temp.scale.z = 0.20;

    robot_marker_temp.color.a = 0.64;
    robot_marker_temp.color.r = 1;
    robot_marker_temp.color.b = 1;
    robot_marker_temp.color.g = 1;

    robot_marker_array.markers.emplace_back(robot_marker_temp);




    //wheels 1
    q_temp = q_robot * q090_rol;
    robot_marker_temp.action = visualization_msgs::Marker::ADD;
    robot_marker_temp.id = 1;

    p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(0.25,0.25,-0.12));

    robot_marker_temp.pose.orientation.w = q_temp.w();
    robot_marker_temp.pose.orientation.x = q_temp.x();
    robot_marker_temp.pose.orientation.y = q_temp.y();
    robot_marker_temp.pose.orientation.z = q_temp.z();

    robot_marker_temp.pose.position.x = t_robot.x() + p_temp(0);
    robot_marker_temp.pose.position.y = t_robot.y() + p_temp(1);
    robot_marker_temp.pose.position.z = t_robot.z() + p_temp(2);

    robot_marker_temp.type = visualization_msgs::Marker::CYLINDER;

    robot_marker_temp.scale.x = 0.2;
    robot_marker_temp.scale.y = 0.2;
    robot_marker_temp.scale.z = 0.02;

    robot_marker_temp.color.a = 0.8;
    robot_marker_temp.color.r = 0;
    robot_marker_temp.color.b = 0;
    robot_marker_temp.color.g = 0;

    robot_marker_array.markers.emplace_back(robot_marker_temp);


    //wheels 2
    q_temp = q_robot * q090_rol;
    robot_marker_temp.action = visualization_msgs::Marker::ADD;
    robot_marker_temp.id = 2;

    p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(-0.25,0.25,-0.12));

    robot_marker_temp.pose.orientation.w = q_temp.w();
    robot_marker_temp.pose.orientation.x = q_temp.x();
    robot_marker_temp.pose.orientation.y = q_temp.y();
    robot_marker_temp.pose.orientation.z = q_temp.z();

    robot_marker_temp.pose.position.x = t_robot.x() + p_temp(0);
    robot_marker_temp.pose.position.y = t_robot.y() + p_temp(1);
    robot_marker_temp.pose.position.z = t_robot.z() + p_temp(2);

    robot_marker_temp.type = visualization_msgs::Marker::CYLINDER;

    robot_marker_temp.scale.x = 0.2;
    robot_marker_temp.scale.y = 0.2;
    robot_marker_temp.scale.z = 0.02;

    robot_marker_temp.color.a = 0.8;
    robot_marker_temp.color.r = 0;
    robot_marker_temp.color.b = 0;
    robot_marker_temp.color.g = 0;

    robot_marker_array.markers.emplace_back(robot_marker_temp);

    
    //wheels 3
    q_temp = q_robot * q090_rol;
    robot_marker_temp.action = visualization_msgs::Marker::ADD;
    robot_marker_temp.id = 3;

    p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(-0.25,-0.25,-0.12));

    robot_marker_temp.pose.orientation.w = q_temp.w();
    robot_marker_temp.pose.orientation.x = q_temp.x();
    robot_marker_temp.pose.orientation.y = q_temp.y();
    robot_marker_temp.pose.orientation.z = q_temp.z();

    robot_marker_temp.pose.position.x = t_robot.x() + p_temp(0);
    robot_marker_temp.pose.position.y = t_robot.y() + p_temp(1);
    robot_marker_temp.pose.position.z = t_robot.z() + p_temp(2);

    robot_marker_temp.type = visualization_msgs::Marker::CYLINDER;

    robot_marker_temp.scale.x = 0.2;
    robot_marker_temp.scale.y = 0.2;
    robot_marker_temp.scale.z = 0.02;

    robot_marker_temp.color.a = 0.8;
    robot_marker_temp.color.r = 0;
    robot_marker_temp.color.b = 0;
    robot_marker_temp.color.g = 0;

    robot_marker_array.markers.emplace_back(robot_marker_temp);



    //wheels 4
    q_temp = q_robot * q090_rol;
    robot_marker_temp.action = visualization_msgs::Marker::ADD;
    robot_marker_temp.id = 4;

    p_temp = q_rotate_vector(q_robot, Eigen::Vector3d(0.25,-0.25,-0.12));

    robot_marker_temp.pose.orientation.w = q_temp.w();
    robot_marker_temp.pose.orientation.x = q_temp.x();
    robot_marker_temp.pose.orientation.y = q_temp.y();
    robot_marker_temp.pose.orientation.z = q_temp.z();

    robot_marker_temp.pose.position.x = t_robot.x() + p_temp(0);
    robot_marker_temp.pose.position.y = t_robot.y() + p_temp(1);
    robot_marker_temp.pose.position.z = t_robot.z() + p_temp(2);

    robot_marker_temp.type = visualization_msgs::Marker::CYLINDER;

    robot_marker_temp.scale.x = 0.2;
    robot_marker_temp.scale.y = 0.2;
    robot_marker_temp.scale.z = 0.02;

    robot_marker_temp.color.a = 0.8;
    robot_marker_temp.color.r = 0;
    robot_marker_temp.color.b = 0;
    robot_marker_temp.color.g = 0;

    robot_marker_array.markers.emplace_back(robot_marker_temp);
}

rviz_vehicle::rviz_vehicle()
{

}

rviz_vehicle::~rviz_vehicle()
{

}

#endif