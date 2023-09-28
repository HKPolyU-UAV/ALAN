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
 * \file calibrate_led2uav.cpp
 * \date 22/Sep/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief alan -> config relative pose between led and uav
 */

#include "../include/tools/essential.h"
#include "ros_vicon_sdk/PointArray.h"
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

static Sophus::SE3d ledPose;
static Sophus::SE3d uavPose;
static Sophus::SE3d led_in_uav_pose;
static vision::cameraModel tool;


void callback(
    const geometry_msgs::PoseStamped::ConstPtr& led_pose, 
    const geometry_msgs::PoseStamped::ConstPtr& uav_pose
)
{
    ledPose = Sophus::SE3d(
        Eigen::Quaterniond(
            led_pose->pose.orientation.w,
            led_pose->pose.orientation.x,
            led_pose->pose.orientation.y,
            led_pose->pose.orientation.z
        ).toRotationMatrix(),
        Eigen::Vector3d(
            led_pose->pose.position.x,
            led_pose->pose.position.y,
            led_pose->pose.position.z
        )
    );

    uavPose = Sophus::SE3d(
        Eigen::Quaterniond(
            uav_pose->pose.orientation.w,
            uav_pose->pose.orientation.x,
            uav_pose->pose.orientation.y,
            uav_pose->pose.orientation.z
        ).toRotationMatrix(),
        Eigen::Vector3d(
            uav_pose->pose.position.x,
            uav_pose->pose.position.y,
            uav_pose->pose.position.z
        )
    );

    // global * what = led
    // what = global.inv * led
    led_in_uav_pose = uavPose.inverse() * ledPose;

    std::cout<<led_in_uav_pose.translation()<<std::endl<<std::endl;;
    std::cout<<tool.q2rpy(
        Eigen::Quaterniond(
            led_in_uav_pose.rotationMatrix()
        )
    )/ M_PI * 180<<std::endl<<std::endl;
    std::cout<<"========"<<std::endl;


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;

    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_led_pose(nh, "/vrpn_client_node/gh034_led/pose",1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_uav_pose(nh, "/vrpn_client_node/gh034_nano/pose",1);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_led_pose, sub_uav_pose);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    

    return 0;
}