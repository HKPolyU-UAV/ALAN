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
 * \file calibrate_cam2ugv.cpp
 * \date 22/Sep/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief alan -> config relative pose between cam and ugv
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
static Sophus::SE3d cam_in_ugv_Pose_previous;
static vision::cameraModel tool;
static double iir_gain = 0.4;
static bool start_iir = false;


void callback(
    const geometry_msgs::PoseStamped::ConstPtr& cam_pose, 
    const geometry_msgs::PoseStamped::ConstPtr& ugv_pose
)
{
    camPose = Sophus::SE3d(
        Eigen::Quaterniond(
            cam_pose->pose.orientation.w,
            cam_pose->pose.orientation.x,
            cam_pose->pose.orientation.y,
            cam_pose->pose.orientation.z
        ).toRotationMatrix(),
        Eigen::Vector3d(
            cam_pose->pose.position.x,
            cam_pose->pose.position.y,
            cam_pose->pose.position.z
        )
    );

    ugvPose = Sophus::SE3d(
        Eigen::Quaterniond(
            ugv_pose->pose.orientation.w,
            ugv_pose->pose.orientation.x,
            ugv_pose->pose.orientation.y,
            ugv_pose->pose.orientation.z
        ).toRotationMatrix(),
        Eigen::Vector3d(
            ugv_pose->pose.position.x,
            ugv_pose->pose.position.y,
            ugv_pose->pose.position.z
        )
    );

    // global * what = cam
    // what = global.inv * cam
    cam_in_ugv_Pose = ugvPose.inverse() * camPose;

    std::cout<<cam_in_ugv_Pose.translation()<<std::endl<<std::endl;;
    std::cout<<tool.q2rpy(
        Eigen::Quaterniond(
            cam_in_ugv_Pose.rotationMatrix()
        )
    )/ M_PI * 180<<std::endl<<std::endl;
    std::cout<<"========"<<std::endl;

    if(start_iir)
    {
        cam_in_ugv_Pose = Sophus::SE3d::exp(
            (iir_gain * cam_in_ugv_Pose.log() 
                + 
            (1 - iir_gain) * cam_in_ugv_Pose_previous.log())
        );
    } 

    cam_in_ugv_Pose_previous = cam_in_ugv_Pose;
    start_iir = true;


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;

    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_cam_pose(nh, "/vrpn_client_node/gh034_d455/pose",1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_ugv_pose(nh, "/vrpn_client_node/gh034_car/pose",1);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_cam_pose, sub_ugv_pose);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    std::cout<<std::endl;
    std::cout<<"WRITING CALIBRATION DATA TO TXT..."<<std::endl;

    std::remove("/home/patty/alan_ws/src/alan/alan_state_estimation/src/tools/calib_data/cam2ugv.txt");
    std::ofstream save(
        "/home/patty/alan_ws/src/alan/alan_state_estimation/src/tools/calib_data/cam2ugv.txt",
        std::ios::app
    );

    save<<"SE(3):"<<std::endl;
    save << cam_in_ugv_Pose.matrix();
    save<<std::endl<<std::endl;
    
    save<<"translation:"<<std::endl;
    save<<cam_in_ugv_Pose.translation();
    save<<std::endl<<std::endl;

    save<<"rotation:"<<std::endl;
    save<<tool.q2rpy(
        Eigen::Quaterniond(
            cam_in_ugv_Pose.rotationMatrix()
        )
    )/ M_PI * 180<<std::endl;

    save.close();

    

    return 0;
}