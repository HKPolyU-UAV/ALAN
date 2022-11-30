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

#ifndef ARUCO_H
#define ARUCO_H

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


namespace alan
{
    class FailSafeNodelet : public nodelet::Nodelet
    {
        private:

        //publisher
            ros::Publisher uav_odom_final_pub;
            ros::Publisher ugv_odom_final_pub;
            nav_msgs::Odometry uav_final_odom;
            nav_msgs::Odometry ugv_final_odom;
            
        //subscriber
            //objects
            geometry_msgs::PoseStamped uav_vrpn_pose;  
            bool uav_vrpn_pose_initiated = false;
            geometry_msgs::TwistStamped uav_vrpn_twist;
            bool uav_vrpn_twist_initiated = false;

            geometry_msgs::PoseStamped ugv_vrpn_pose;
            bool ugv_vrpn_pose_initiated = false;
            geometry_msgs::TwistStamped ugv_vrpn_twist;
            bool ugv_vrpn_twist_initiated = false;
            
            message_filters::Subscriber<geometry_msgs::PoseStamped>vrpn_uavpose_sub;
            message_filters::Subscriber<geometry_msgs::TwistStamped>vrpn_uavtwist_sub;
            message_filters::Subscriber<geometry_msgs::PoseStamped>vrpn_ugvpose_sub;
            message_filters::Subscriber<geometry_msgs::TwistStamped>vrpn_ugvtwist_sub;

            typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> uavMySyncPolicy;
            typedef message_filters::Synchronizer<uavMySyncPolicy> uavsync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<uavsync> uavsync_;     

            typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> ugvMySyncPolicy;
            typedef message_filters::Synchronizer<ugvMySyncPolicy> ugvsync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<ugvsync> ugvsync_;
            
            //functions
            void uav_msg_callback(const geometry_msgs::PoseStamped::ConstPtr& pose, const geometry_msgs::TwistStamped::ConstPtr& twist);

            void ugv_msg_callback(const geometry_msgs::PoseStamped::ConstPtr& pose, const geometry_msgs::TwistStamped::ConstPtr& twist);
            

            virtual void onInit() 
            {
                ros::NodeHandle& nh = getMTNodeHandle();
                    
                vrpn_uavpose_sub.subscribe(nh, "/uav/mavros/vision_pose/pose", 1);
                vrpn_uavtwist_sub.subscribe(nh, "/vrpn_client_node/gh034_nano/twist", 1);
                
                uavsync_.reset(new uavsync( uavMySyncPolicy(10), vrpn_uavpose_sub, vrpn_uavtwist_sub));            
                uavsync_->registerCallback(boost::bind(&FailSafeNodelet::uav_msg_callback, this, _1, _2));

                vrpn_ugvpose_sub.subscribe(nh, "/vrpn_client_node/gh034_car/pose", 1);
                vrpn_ugvtwist_sub.subscribe(nh, "/vrpn_client_node/gh034_car/twist", 1);

                ugvsync_.reset(new ugvsync( ugvMySyncPolicy(10), vrpn_ugvpose_sub, vrpn_ugvtwist_sub));            
                ugvsync_->registerCallback(boost::bind(&FailSafeNodelet::ugv_msg_callback, this, _1, _2));                


                uav_odom_final_pub = nh.advertise<nav_msgs::Odometry>
                        ("/uav/alan_estimation/final_odom", 1, true);
                ugv_odom_final_pub = nh.advertise<nav_msgs::Odometry>
                        ("/ugv/alan_estimation/final_odom", 1, true);

                ROS_INFO("FailSafe Nodelet Initiated...");
            }     

            public:
                static void* PubMainLoop(void* tmp);   

    };
    
    PLUGINLIB_EXPORT_CLASS(alan::FailSafeNodelet, nodelet::Nodelet)
}


#endif