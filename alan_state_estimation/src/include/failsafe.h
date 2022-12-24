/*
    A ROS Nodelet H + CPP file for
    vision-based relative localization (6D) for UAV and UGV.
    Created on 15/06/2022
    (c) pattylo
    from the RCUAS of Hong Kong Polytechnic University
*/

/**
 * \file aruco.h + aruco.cpp
 * \brief classes for vision-based relative localization for UAV and UGV based on ARUCOs
 * \remark //this aruco library is used for baseline validations as well as trial and error of algorithms
 */

#ifndef FAILSAFE_H
#define FAILSAFE_H

#include "tools/essential.h"

#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <sophus/se3.hpp>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>


#include <pthread.h>

#include "tools/RosTopicConfigs.h"
#define COLOR_SUB_TOPIC CAMERA_SUB_TOPIC_A
#define DEPTH_SUB_TOPIC CAMERA_SUB_TOPIC_B
#define UAV_POSE_SUB_TOPIC POSE_SUB_TOPIC_A
#define UGV_POSE_SUB_TOPIC POSE_SUB_TOPIC_B

#define LED_POSE_PUB_TOPIC POSE_PUB_TOPIC_A
#define UGV_POSE_PUB_TOPIC POSE_PUB_TOPIC_B
#define UAV_POSE_PUB_TOPIC POSE_PUB_TOPIC_C
#define CAM_POSE_PUB_TOPIC POSE_PUB_TOPIC_D

#define LED_ODOM_PUB_TOPIC ODOM_PUB_TOPIC_A


namespace alan
{
    class FailSafeNodelet : public nodelet::Nodelet
    {
        private:
        //general 
            bool failsafe_on = false;

        //publisher
            ros::Publisher uav_odom_final_pub;
            ros::Publisher ugv_odom_final_pub;
            nav_msgs::Odometry uav_final_odom;
            nav_msgs::Odometry ugv_final_odom;                                    
            
        //subscriber
            //objects
            nav_msgs::Odometry led_odom;
            bool led_odom_initiated = false;
            
            geometry_msgs::PoseStamped uav_vrpn_pose;  
            bool uav_vrpn_pose_initiated = false;
            geometry_msgs::TwistStamped uav_vrpn_twist;
            bool uav_vrpn_twist_initiated = false;

            geometry_msgs::PoseStamped ugv_vrpn_pose;
            bool ugv_vrpn_pose_initiated = false;
            geometry_msgs::TwistStamped ugv_vrpn_twist;
            bool ugv_vrpn_twist_initiated = false;

            ros::Subscriber UavVrpnPoseSub, UavVrpnTwistSub;
            ros::Subscriber UgvVrpnPoseSub, UgvVrpnTwistSub;
            ros::Subscriber LedOdomSub;


            //functions            
            void uav_vrpn_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
            void uav_vrpn_twist_callback(const geometry_msgs::TwistStamped::ConstPtr& twist);

            void ugv_vrpn_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
            void ugv_vrpn_twist_callback(const geometry_msgs::TwistStamped::ConstPtr& twist);

            void led_odom_callback(const nav_msgs::Odometry::ConstPtr& odom);

            virtual void onInit() 
            {
                ros::NodeHandle& nh = getMTNodeHandle();
                ROS_INFO("FailSafe Nodelet Initiated...");

                nh.getParam("/alan_master/failsafe_on", failsafe_on);

                RosTopicConfigs configs(nh, "/alan_master");
                    
                UavVrpnPoseSub = nh.subscribe<geometry_msgs::PoseStamped>
                                ("/uav/mavros/vision_pose/pose", 1, &FailSafeNodelet::uav_vrpn_pose_callback, this);
                
                UavVrpnTwistSub = nh.subscribe<geometry_msgs::TwistStamped>
                                ("/vrpn_client_node", 1, &FailSafeNodelet::uav_vrpn_twist_callback, this);

                UgvVrpnPoseSub = nh.subscribe<geometry_msgs::PoseStamped>
                                ("", 1, &FailSafeNodelet::ugv_vrpn_pose_callback, this);

                // LedPoseSub = nh.subscribe<geometry_msgs::PoseSta
                LedOdomSub = nh.subscribe<nav_msgs::Odometry>
                                (configs.getTopicName(LED_ODOM_PUB_TOPIC), 1, &FailSafeNodelet::led_odom_callback, this);

                
                uav_odom_final_pub = nh.advertise<nav_msgs::Odometry>
                        ("/uav/alan_estimation/final_odom", 1, true);
                ugv_odom_final_pub = nh.advertise<nav_msgs::Odometry>
                        ("/ugv/alan_estimation/final_odom", 1, true);

            }     

    };
    
    PLUGINLIB_EXPORT_CLASS(alan::FailSafeNodelet, nodelet::Nodelet)
}


#endif