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

namespace alan
{
    class MsgSyncNodelet : public nodelet::Nodelet
    {
        private:
            pthread_t tid;

        //publish
            //objects
            ros::Publisher uav_pub_AlanPlannerMsg;
            ros::Publisher ugv_pub_AlanPlannerMsg;
            ros::Publisher alan_sfc_pub;            
            ros::Publisher alan_all_sfc_pub;

        //subscriber
            //objects
            ros::Subscriber UavOdomSub, UavImuSub;
            ros::Subscriber UgvOdomSub, UgvImuSub;

            // message_filters::Subscriber<nav_msgs::Odometry> uav_sub_odom;
            // message_filters::Subscriber<geometry_msgs::TwistStamped> uav_sub_imu;

            // message_filters::Subscriber<nav_msgs::Odometry> ugv_sub_odom;
            // message_filters::Subscriber<sensor_msgs::Imu> ugv_sub_imu;
            ros::Subscriber cam_pose_sub;
            
            typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::TwistStamped> uavMySyncPolicy;
            typedef message_filters::Synchronizer<uavMySyncPolicy> uavsync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<uavsync> uavsync_;     

            typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> ugvMySyncPolicy;
            typedef message_filters::Synchronizer<ugvMySyncPolicy> ugvsync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<ugvsync> ugvsync_;
            
            //functions
            void uav_odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
            void uav_imu_callback(const sensor_msgs::Imu::ConstPtr& imu);

            void ugv_odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
            void ugv_imu_callback(const sensor_msgs::Imu::ConstPtr& imu);


             
            //private variables 
            nav_msgs::Odometry uav_odom;
            geometry_msgs::TwistStamped uav_imu;  

            nav_msgs::Odometry ugv_odom;
            sensor_msgs::Imu ugv_imu; 

            alan_landing_planning::AlanPlannerMsg uav_alan_msg;
            alan_landing_planning::AlanPlannerMsg ugv_alan_msg;


            bool uav_odom_initiated;
            bool uav_imu_initiated;

            bool ugv_odom_initiated;
            bool ugv_imu_initiated;

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
            Eigen::Vector3d q2rpy(Eigen::Quaterniond q);
            Eigen::Quaterniond rpy2q(Eigen::Vector3d rpy);

            Eigen::Vector3d q_rotate_vector(Eigen::Quaterniond q, Eigen::Vector3d v);

            void set_total_bound(Eigen::Translation3d t_current, Eigen::Quaterniond q_current);
            void set_all_sfc(Eigen::Translation3d t_current,Eigen::Quaterniond q_current);

            alan_visualization::Tangent construct_tangent_plane(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d pt);
            alan_visualization::Tangent set_plane_bound(Eigen::Vector3d v, Eigen::Vector3d pt);

            Eigen::Vector3d get_outer_product(Eigen::Vector3d v1, Eigen::Vector3d v2);          
                    

            virtual void onInit() 
            {
                ros::NodeHandle& nh = getMTNodeHandle();    
                ROS_INFO("MSGSYNC Nodelet Initiated...");

            //param
                nh.getParam("/alan_master/cam_FOV_H", FOV_H);     
                nh.getParam("/alan_master/cam_FOV_V", FOV_V);   

                nh.getParam("/alan_master/final_corridor_height", final_corridor_height);
                nh.getParam("/alan_master/final_corridor_length", final_corridor_length);                          
            
            //set sfc visualization
                FOV_H = FOV_H / 180 * M_PI * 0.75;
                FOV_V = FOV_V / 180 * M_PI * 0.75;        

                Eigen::Vector3d rpy_temp;

                rpy_temp = Eigen::Vector3d(0, -FOV_V/2, -FOV_H/2);                
                q1 = rpy2q(rpy_temp);
                cam_1axis_vector = q_rotate_vector(q1, cam_center_vector);

                rpy_temp = Eigen::Vector3d(0, -FOV_V/2, FOV_H/2);                
                q2 = rpy2q(rpy_temp);
                cam_2axis_vector = q_rotate_vector(q2, cam_center_vector);

                rpy_temp = Eigen::Vector3d(0, FOV_V/2, FOV_H/2);                
                q3 = rpy2q(rpy_temp); 
                cam_3axis_vector = q_rotate_vector(q3, cam_center_vector);

                rpy_temp = Eigen::Vector3d(0, FOV_V/2, -FOV_H/2);                
                q4 = rpy2q(rpy_temp);
                cam_4axis_vector = q_rotate_vector(q4, cam_center_vector);

                c2b_ugv.resize(6);
    
                c2b_ugv(0) = 0.38;
                c2b_ugv(1) = 0.0;
                c2b_ugv(2) = 0.12;

                c2b_ugv(3) = 0;//r
                c2b_ugv(4) = (-20.0) / 180.0 * M_PI;//p
                c2b_ugv(5) = M_PI;//y

                q_c2b = rpy2q(
                    Eigen::Vector3d(
                        c2b_ugv(3),
                        c2b_ugv(4),
                        c2b_ugv(5)
                    )
                );

                t_c2b = Eigen::Translation3d(
                    c2b_ugv(0),
                    c2b_ugv(1),
                    c2b_ugv(2)
                );

                body_to_cam_Pose = t_c2b * q_c2b;

            //subscriber
                UavOdomSub = nh.subscribe<nav_msgs::Odometry>
                                ("/uav/alan_estimation/final_odom", 1, &MsgSyncNodelet::uav_odom_callback, this);
                
                UavImuSub = nh.subscribe<sensor_msgs::Imu>
                                ("/uav/mavros/imu/data", 1, &MsgSyncNodelet::uav_imu_callback, this);


                UgvOdomSub = nh.subscribe<nav_msgs::Odometry>
                                ("/ugv/alan_estimation/final_odom", 1, &MsgSyncNodelet::ugv_odom_callback, this);
                
                UgvImuSub = nh.subscribe<sensor_msgs::Imu>
                                ("/ugv/mavros/imu/data", 1, &MsgSyncNodelet::ugv_imu_callback, this);
                                            
            //publisher
                uav_pub_AlanPlannerMsg = nh.advertise<alan_landing_planning::AlanPlannerMsg>
                                    ("/AlanPlannerMsg/uav/data", 1);

                ugv_pub_AlanPlannerMsg = nh.advertise<alan_landing_planning::AlanPlannerMsg>
                                    ("/AlanPlannerMsg/ugv/data", 1);

                // alan_sfc_pub = nh.advertise<alan_visualization::Polyhedron>
                //                     ("/alan/sfc/total_bound", 1);

                alan_all_sfc_pub = nh.advertise<alan_visualization::PolyhedronArray>
                                    ("/alan/sfc/all_corridors", 1);                                                    
                
            }      

    };
        
    PLUGINLIB_EXPORT_CLASS(alan::MsgSyncNodelet, nodelet::Nodelet)

}

#endif