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

static Eigen::MatrixXd led_position;
static Eigen::MatrixXd led_position_previous;
static Sophus::SE3d uavPose;
static double iir_gain = 0.4;
static bool start_iir = false;

void callback(
    const geometry_msgs::PoseStamped::ConstPtr& uav_pose,
    const ros_vicon_sdk::PointArray::ConstPtr& led_posi
)
{
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

    std::cout<<"here uav:"<<std::endl;

    std::cout<<uavPose.translation()<<std::endl<<std::endl;

    

    std::cout<<"lala"<<std::endl;
    for(int i = 0; i < led_posi->PointArray.size(); i++)
    {
        Eigen::Vector4d led_temp = Eigen::Vector4d(
            led_posi->PointArray[i].x,
            led_posi->PointArray[i].y,
            led_posi->PointArray[i].z,
            1
        );

        led_temp.head(3) = led_temp.head(3) * 0.001;

        std::cout<<led_temp<<std::endl;

        std::cout<<"============"<<std::endl;

        led_position.block<1,3>(i,0) = (uavPose.matrix().inverse() * led_temp).head(3);
    }
    std::cout<<std::endl<<std::endl;

    if(start_iir)
    {
        led_position = iir_gain * led_position + (1 - iir_gain) * led_position_previous;
    }

    led_position_previous = led_position;
    start_iir = true;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;

    led_position.resize(6,3);
    led_position_previous.resize(6,3);

    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_uav_pose(nh, "/vrpn_client_node/gh034_nano/pose",1);
    message_filters::Subscriber<ros_vicon_sdk::PointArray> sub_led_posi(nh, "/led_detailed_positions",1);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, ros_vicon_sdk::PointArray> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_uav_pose, sub_led_posi);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    std::cout<<std::endl;
    std::cout<<"WRITING CALIBRATION DATA TO TXT..."<<std::endl;

    std::remove("/home/patty/alan_ws/src/alan/alan_state_estimation/src/tools/calib_data/led_rela.txt");
    std::ofstream save(
        "/home/patty/alan_ws/src/alan/alan_state_estimation/src/tools/calib_data/led_rela.txt",
        std::ios::app
    );

    save<<"led position:"<<std::endl;
    save << led_position;
    save<<std::endl<<std::endl;

    save.close();


    return 0;
}