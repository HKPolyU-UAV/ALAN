










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
 * \file calibrate_led_rela.cpp
 * \date 22/Sep/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief alan -> config relative positions within LED constellation
 */

#include "../include/tools/essential.h"
#include "ros_vicon_sdk/PointArray.h"
#include "alan_state_estimation/temp.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <sophus/se3.hpp>

#include "../include/cameraModel.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;

static Eigen::MatrixXd led_position;
static Eigen::MatrixXd led_position_previous;
static double iir_gain = 0.4;
static bool start_iir = false;

static ros::Publisher temp_pub;
static ros_vicon_sdk::PointArray temp_pub_obj;

// static ros::Publisher temp_pub;
// static alan_state_estimation::temp temp_pub_obj;

static ros::Publisher temp_pub_uav;
static geometry_msgs::PoseStamped temp_pub_uav_obj;

void led_posi_callback(const alan_state_estimation::temp::ConstPtr& msg)
{
    for(auto& what : msg->PointArray)
    {
        std::cout<<what.x<<std::endl;
        std::cout<<what.y<<std::endl;
        std::cout<<what.z<<std::endl;
        std::cout<<"=========="<<std::endl;
    }

    for(int i = 0; i < msg->PointArray.size(); i++)
    {
        led_position.block<1,3>(i,0) = Eigen::Vector3d(
            msg->PointArray[i].x,
            msg->PointArray[i].x,
            msg->PointArray[i].z
        );
    }

    if(start_iir)
    {
        led_position = iir_gain * led_position + (1 - iir_gain) * led_position_previous;
    }

    led_position_previous = led_position;
    start_iir = true;


    temp_pub_obj.header = msg->header;
    temp_pub_obj.PointArray = msg->PointArray;

    temp_pub.publish(temp_pub_obj);
}

void uav_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    temp_pub_uav_obj = *msg;

    temp_pub_uav.publish(temp_pub_uav_obj);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;

    led_position.resize(6,3);
    led_position_previous.resize(6,3);

    ros::Subscriber led_position_sub = nh.subscribe
        <alan_state_estimation::temp>("/leds", 1, &led_posi_callback);

    ros::Subscriber uav_pose_sub = nh.subscribe
        <geometry_msgs::PoseStamped>("/uav", 1, &uav_pose_callback);
    
    temp_pub = nh.advertise
        <ros_vicon_sdk::PointArray>("/led_detailed_positions",1);

    temp_pub_uav = nh.advertise
        <geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_nano/pose",1);
    //     <ros_vicon_sdk::PointArray>("/led_detailed_positions",1);

    ros::spin();

    // std::cout<<std::endl;
    // std::cout<<"WRITING CALIBRATION DATA TO TXT..."<<std::endl;

    // std::remove("/home/patty/alan_ws/src/alan/alan_state_estimation/src/tools/calib_data/led_rela.txt");
    // std::ofstream save(
    //     "/home/patty/alan_ws/src/alan/alan_state_estimation/src/tools/calib_data/led_rela.txt",
    //     std::ios::app
    // );

    // save<<"led position:"<<std::endl;
    // save << led_position;
    // save<<std::endl<<std::endl;

    // save.close();


    

    return 0;
}