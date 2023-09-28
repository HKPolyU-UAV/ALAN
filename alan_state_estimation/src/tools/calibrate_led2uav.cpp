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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;

    rosbag::Bag bag;
    bag.open(
        "/home/patty/alan_ws/misc/exp_09_calibration/0915_calib_led_2_uav.bag", 
        rosbag::bagmode::Read
    );

    std::vector<std::string> topics;
    topics.push_back(std::string("/vrpn_client_node/gh034_led/pose"));
    topics.push_back(std::string("/vrpn_client_node/gh034_nano/pose"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    cout<<"hiiii"<<endl;

    foreach(rosbag::MessageInstance const m, view)
    {
        cout<<"lala"<<endl;
        std::cout<<m.getTopic()<<std::endl;
        std::cout<<m.getTime()<<std::endl;
        geometry_msgs::PoseStamped::ConstPtr p1  = m.instantiate<geometry_msgs::PoseStamped>();
        if (p1 != NULL)
        {
            std::cout << p1->pose.position.x << std::endl;
        }
        else
            cout<<"what what"<<endl;

        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        if (i != NULL)
            std::cout << i->data << std::endl;
    }

    bag.close();

    

    return 0;
}