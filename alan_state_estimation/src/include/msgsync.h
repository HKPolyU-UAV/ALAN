/*
    A ROS Nodelet H + CPP file for
    msg filtering all information (from failsafe, VIO, vrpn, sfc).
    Created on 01/07/2022
    (c) pattylo
    from the RCUAS of Hong Kong Polytechnic University
*/

/**
 * \file msgsync.h + msgsync.cpp
 * \brief classes for synchronization of messages
 */

#ifndef MSGSYNC_H
#define MSGSYNC_H

#include "tools/essential.h"

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include "alan_landing_planning/AlanPlannerMsg.h"
#include "alan_visualization/PolyhedronArray.h"

#include "tools/RosTopicConfigs.h"

#define UAV_VRPN_POSE_SUB_TOPIC POSE_SUB_TOPIC_A
#define UAV_VRPN_TWIST_SUB_TOPIC TWIST_SUB_TOPIC_A
#define UAV_IMU_SUB_TOPIC IMU_SUB_TOPIC_A

#define UGV_VRPN_POSE_SUB_TOPIC POSE_SUB_TOPIC_B
#define UGV_VRPN_TWIST_SUB_TOPIC TWIST_SUB_TOPIC_B
#define UGV_IMU_SUB_TOPIC IMU_SUB_TOPIC_B

#define LED_ODOM_SUB_TOPIC ODOM_SUB_TOPIC_A

namespace alan
{
    class MsgSyncNodelet : public nodelet::Nodelet
    {
        private:
            int failsafe_indicator = 0;
            bool failsafe_on = false;

        //publish
            //objects
            ros::Publisher uav_pub_AlanPlannerMsg;
            ros::Publisher ugv_pub_AlanPlannerMsg;
            ros::Publisher alan_sfc_pub;            
            ros::Publisher alan_all_sfc_pub;

        //subscriber
            //objects
            ros::Subscriber UavVrpnPoseSub, UavVrpnTwistSub, UavImuSub;
            ros::Subscriber UgvVrpnPoseSub, UgvVrpnTwistSub, UgvImuSub;
            ros::Subscriber LedOdomSub;      
            geometry_msgs::PoseStamped uav_vrpn_pose;
            bool uav_vrpn_pose_initiated = false;
            geometry_msgs::TwistStamped uav_vrpn_twist;
            bool uav_vrpn_twist_initiated = false;
            sensor_msgs::Imu uav_imu;
            bool uav_imu_initiated = false;
            
            geometry_msgs::PoseStamped ugv_vrpn_pose;
            bool ugv_vrpn_pose_initiated = false;
            geometry_msgs::TwistStamped ugv_vrpn_twist;
            bool ugv_vrpn_twist_initiated = false;
            sensor_msgs::Imu ugv_imu;     
            bool ugv_imu_initiated = false;

            nav_msgs::Odometry led_odom;             
            bool led_odom_inititated = false;

            //functions                    
            void uav_vrpn_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
            void uav_vrpn_twist_callback(const geometry_msgs::TwistStamped::ConstPtr& twist);
            void uav_imu_callback(const sensor_msgs::Imu::ConstPtr& imu);

            void ugv_vrpn_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
            void ugv_vrpn_twist_callback(const geometry_msgs::TwistStamped::ConstPtr& twist);
            void ugv_imu_callback(const sensor_msgs::Imu::ConstPtr& imu);

            void led_odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
        
             
            //private variables 
            // nav_msgs::Odometry uav_odom;
            // geometry_msgs::TwistStamped uav_imu;  

            // nav_msgs::Odometry ugv_odom;
            // sensor_msgs::Imu ugv_imu; 

            alan_landing_planning::AlanPlannerMsg uav_alan_msg;
            alan_landing_planning::AlanPlannerMsg ugv_alan_msg;

            Eigen::Isometry3d uavOdomPose;
            Eigen::Vector3d uav_pos_world;
            Eigen::Vector3d uav_acc_world;
            Eigen::Vector3d uav_acc_body;

            Eigen::Isometry3d ugvOdomPose;
            Eigen::Vector3d ugv_pos_world;
            Eigen::Vector3d ugv_acc_world;
            Eigen::Vector3d ugv_acc_body;

            geometry_msgs::PoseStamped cam_current_PoseMsg;
            Eigen::Isometry3d camPose;
            Eigen::Vector3d cam_pos_world;


        //SFC RVIZ
            //objects 
            double FOV_H = 0, FOV_V = 0;//fov horizontal & vertical
            double final_corridor_height = 0, final_corridor_length = 0;

            Eigen::Quaterniond q1, q2, q3, q4;
            Eigen::Vector3d cam_center_vector = Eigen::Vector3d(1,0,0),
                            cam_1axis_vector, 
                            cam_2axis_vector, 
                            cam_3axis_vector,
                            cam_4axis_vector;

            alan_visualization::Polyhedron polyh_total_bound;
            alan_visualization::PolyhedronArray polyh_array_pub_object;
            Eigen::VectorXd c2b_ugv;
            Eigen::Quaterniond q_c2b;
            Eigen::Translation3d t_c2b;
            Eigen::Isometry3d body_to_cam_Pose;

            //functions
            void setup_publish_SFC();
            Eigen::Vector3d q2rpy(Eigen::Quaterniond q);
            Eigen::Quaterniond rpy2q(Eigen::Vector3d rpy);
            Eigen::Vector3d q_rotate_vector(Eigen::Quaterniond q, Eigen::Vector3d v);

            void set_total_bound(Eigen::Translation3d t_current, Eigen::Quaterniond q_current);
            void set_all_sfc(Eigen::Translation3d t_current,Eigen::Quaterniond q_current);

            alan_visualization::Tangent construct_tangent_plane(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d pt);
            alan_visualization::Tangent set_plane_bound(Eigen::Vector3d v, Eigen::Vector3d pt);

            Eigen::Vector3d get_outer_product(Eigen::Vector3d v1, Eigen::Vector3d v2);   

            void setup_camera_config(ros::NodeHandle& nh);       
                    

            virtual void onInit() 
            {
                ros::NodeHandle& nh = getMTNodeHandle();    
                ROS_INFO("MSGSYNC Nodelet Initiated..."); 


                nh.getParam("/alan_master/failsafe_on", failsafe_indicator);
                if(failsafe_indicator == 1)
                    failsafe_on = true;
                else
                    failsafe_on = false;
                
                setup_camera_config(nh);

                RosTopicConfigs configs(nh, "/msgsync");

            //subscriber
                UavVrpnPoseSub = nh.subscribe<geometry_msgs::PoseStamped>
                        (configs.getTopicName(UAV_VRPN_POSE_SUB_TOPIC), 1, &MsgSyncNodelet::uav_vrpn_pose_callback, this);

                UavVrpnTwistSub = nh.subscribe<geometry_msgs::TwistStamped>
                        (configs.getTopicName(UAV_VRPN_TWIST_SUB_TOPIC), 1, &MsgSyncNodelet::uav_vrpn_twist_callback, this);

                UavImuSub = nh.subscribe<sensor_msgs::Imu>
                        (configs.getTopicName(UAV_IMU_SUB_TOPIC), 1, &MsgSyncNodelet::uav_imu_callback, this);

                UgvVrpnPoseSub = nh.subscribe<geometry_msgs::PoseStamped>
                        (configs.getTopicName(UGV_VRPN_POSE_SUB_TOPIC), 1, &MsgSyncNodelet::ugv_vrpn_pose_callback, this);

                UgvVrpnTwistSub = nh.subscribe<geometry_msgs::TwistStamped>
                        (configs.getTopicName(UGV_VRPN_TWIST_SUB_TOPIC), 1, &MsgSyncNodelet::ugv_vrpn_twist_callback, this);

                UgvImuSub = nh.subscribe<sensor_msgs::Imu>
                        (configs.getTopicName(UGV_IMU_SUB_TOPIC), 1, &MsgSyncNodelet::ugv_imu_callback, this);

                led_odom.pose.pose.position.x = -10000000;
                led_odom.pose.pose.position.y = -10000000;
                led_odom.pose.pose.position.z = -10000000;
                
                LedOdomSub = nh.subscribe<nav_msgs::Odometry>
                        (configs.getTopicName(LED_ODOM_SUB_TOPIC), 1, &MsgSyncNodelet::led_odom_callback, this);
                
                                           
            //publisher
                uav_pub_AlanPlannerMsg = nh.advertise<alan_landing_planning::AlanPlannerMsg>
                        ("/alan_state_estimation/msgsync/uav/alan_planner_msg", 1);

                ugv_pub_AlanPlannerMsg = nh.advertise<alan_landing_planning::AlanPlannerMsg>
                        ("/alan_state_estimation/msgsync/ugv/alan_planner_msg", 1);

                // alan_sfc_pub = nh.advertise<alan_visualization::Polyhedron>
                //                     ("/alan/sfc/total_bound", 1);

                alan_all_sfc_pub = nh.advertise<alan_visualization::PolyhedronArray>
                        ("/alan_state_estimation/msgsync/polyhedron_array", 1);   
            }      

    };
        
    PLUGINLIB_EXPORT_CLASS(alan::MsgSyncNodelet, nodelet::Nodelet)

}

#endif