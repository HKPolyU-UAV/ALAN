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

static Sophus::SE3d camPose;
static Sophus::SE3d ugvPose;
static Sophus::SE3d cam_in_ugv_Pose;
static vision::cameraModel tool;

void led_posi_callback(const ros_vicon_sdk::PointArray::ConstPtr& msg)
{
    for(auto& what : msg->PointArray)
    {
        std::cout<<what.x<<std::endl;
        std::cout<<what.y<<std::endl;
        std::cout<<what.z<<std::endl;
        std::cout<<"=========="<<std::endl;
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;

    ros::Subscriber led_position_sub = nh.subscribe
        <ros_vicon_sdk::PointArray>("/led_detailed_positions", 1, &led_posi_callback);

    ros::spin();

    

    return 0;
}