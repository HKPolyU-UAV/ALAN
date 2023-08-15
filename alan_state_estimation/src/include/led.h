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
 * \file led.h
 * \date 28/07/2022
 * \author pattylo
 * \copyright (c) RCUAS of Hong Kong Polytechnic University
 * \brief classes for vision-based relative localization for UAV and UGV based on LED markers
 */

#ifndef LED_H
#define LED_H

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
#include <sensor_msgs/Image.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <sophus/se3.hpp>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pthread.h>
#include <tf/tf.h>


#include "tools/RosTopicConfigs.h"
#include "tools/KalmanFilter.hpp"
#include "alan_state_estimation/alan_log.h"

// map definition for convinience
#define COLOR_SUB_TOPIC CAMERA_SUB_TOPIC_A
#define DEPTH_SUB_TOPIC CAMERA_SUB_TOPIC_B
#define UAV_POSE_SUB_TOPIC POSE_SUB_TOPIC_A
#define UGV_POSE_SUB_TOPIC POSE_SUB_TOPIC_B

#define LED_POSE_PUB_TOPIC POSE_PUB_TOPIC_A
#define UGV_POSE_PUB_TOPIC POSE_PUB_TOPIC_B
#define UAV_POSE_PUB_TOPIC POSE_PUB_TOPIC_C
#define CAM_POSE_PUB_TOPIC POSE_PUB_TOPIC_D

#define LED_ODOM_PUB_TOPIC ODOM_PUB_TOPIC_A

namespace correspondence
{
    typedef struct matchid
        {
            int detected_indices; //
            bool detected_ornot = false;
            Eigen::Vector3d pts_3d_correspond;
            Eigen::Vector2d pts_2d_correspond;
        }matchid;
}

namespace alan
{
    class LedNodelet : public nodelet::Nodelet
    {
        //primary objects
            //frames
            cv::Mat frame, display, hsv;
            cv::Mat frame_input;
            cv::Mat im_with_keypoints;
            cv::Mat frame_initial_thresholded;
            
            //time related
            ros::Time led_pose_stamp, led_pose_stamp_previous;
            double last_request = 0;

            //camera related
            Eigen::MatrixXd cameraMat = Eigen::MatrixXd::Zero(3,3);
            Eigen::VectorXd cameraEX;
            Eigen::Quaterniond q_c2b;
            Eigen::Translation3d t_c2b;

            //LED config and correspondences
            std::vector<Eigen::Vector3d> pts_on_body_frame;
            std::vector<correspondence::matchid> corres_global;
            Eigen::VectorXd LEDEX;

            //poses
            Sophus::SE3d pose_global_sophus;
            Sophus::SE3d pose_epnp_sophus, pose_depth_sophus;
            geometry_msgs::PoseStamped ugv_pose_msg, uav_pose_msg;
            geometry_msgs::PoseStamped uav_stpt_msg;
            Eigen::Isometry3d ugv_pose;
            Eigen::Isometry3d cam_pose;
            Eigen::Isometry3d ugv_cam_pose, led_cambody_pose;
            Eigen::Isometry3d uav_pose;

            Eigen::Isometry3d led_pose;
            Eigen::Vector3d cam_origin_in_body_frame, cam_origin;
            Eigen::Quaterniond q_cam, q_led_cambody;
            Eigen::Vector3d t_cam, t_led_cambody;
            
            
        //secondary objects
            // double temp = 0;
            int i = 0;
            bool nodelet_activated = false;
            int detect_no = 0;
            double BA_error = 0;
            double depth_avg_of_all = 0;
            
            std::vector<cv::KeyPoint> blobs_for_initialize;
            int _width = 0, _height = 0;


        //subscribe                                    
            //objects
            message_filters::Subscriber<sensor_msgs::CompressedImage> subimage;
            message_filters::Subscriber<sensor_msgs::Image> subdepth;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
            typedef message_filters::Synchronizer<MySyncPolicy> sync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<sync> sync_;                    
            ros::Subscriber ugv_pose_sub, uav_pose_sub;
            ros::Subscriber uav_setpt_sub;
            //functions
            void camera_callback(const sensor_msgs::CompressedImage::ConstPtr & rgbimage, const sensor_msgs::Image::ConstPtr & depth);            
            void ugv_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
            void uav_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
            void uav_setpt_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
        
        //publisher 
            //objects
            ros::Publisher ledpose_pub, ledodom_pub, 
                           campose_pub, ugvpose_pub, uavpose_pub,
                           record_led_pub, record_uav_pub;
            image_transport::Publisher pubimage;
            image_transport::Publisher pubimage_input;
            //functions
        
        //solve pose & tools
            void solve_pose_w_LED(cv::Mat& frame, cv::Mat depth);             
            Eigen::Vector2d reproject_3D_2D(Eigen::Vector3d P, Sophus::SE3d pose);   
            double get_reprojection_error(std::vector<Eigen::Vector3d> pts_3d, std::vector<Eigen::Vector2d> pts_2d, Sophus::SE3d pose, bool draw_reproject);         

        //main process
            void recursive_filtering(cv::Mat& frame, cv::Mat depth);
            bool search_corres_and_pose_predict(std::vector<Eigen::Vector2d> pts_2d_detect);

        //pnp + BA
            void solve_pnp_initial_pose(std::vector<Eigen::Vector2d> pts_2d, std::vector<Eigen::Vector3d> body_frame_pts);
            void optimize(Sophus::SE3d& pose, std::vector<Eigen::Vector3d> pts_3d_exists, std::vector<Eigen::Vector2d> pts_2d_detected);
                //converge problem need to be solved //-> fuck you, your Jacobian was wrong
            void solveJacobian(Eigen::Matrix<double, 2, 6>& Jacob, Sophus::SE3d pose, Eigen::Vector3d point_3d);

        //LED extraction tool
            //objects
            double LANDING_DISTANCE = 0;
            int BINARY_THRES = 0;
            std::vector<Eigen::Vector2d> LED_extract_POI(cv::Mat& frame, cv::Mat depth);
            std::vector<Eigen::Vector3d> pointcloud_generate(std::vector<Eigen::Vector2d> pts_2d_detected, cv::Mat depthimage);
            bool get_final_POI(std::vector<Eigen::Vector2d>& pts_2d_detected);

        //initiation & correspondence 
            //objects
            bool LED_tracker_initiated_or_tracked = false;
            int LED_no;
            int LED_r_no;
            int LED_g_no;
            //functions       
            void correspondence_search_kmeans(std::vector<Eigen::Vector2d> pts_2d_detected);        
            bool LED_tracking_initialize(cv::Mat& frame, cv::Mat depth);
                      
        //outlier rejection 
            //objects
            cv::Point3f pcl_center_point_wo_outlier_previous;
            double MAD_dilate, MAD_max;
            double MAD_x_threshold = 0, MAD_y_threshold = 0, MAD_z_threshold = 0;
            double min_blob_size = 0;
            //functions
            void reject_outlier(std::vector<Eigen::Vector2d>& pts_2d_detect, cv::Mat depth);
            double calculate_MAD(std::vector<double> norm_of_points);
        
        //depth compensation
            //objects
            Eigen::Vector3d led_3d_posi_in_camera_frame_depth;

        //reinitialization
            bool reinitialization(std::vector<Eigen::Vector2d> pts_2d_detect, cv::Mat depth);
            
        //publish
            //objects
            int error_no = 0;
            int total_no = 0;
            geometry_msgs::PoseStamped led_pose_estimated;
            geometry_msgs::TwistStamped led_twist_estimated;
            nav_msgs::Odometry led_odom_estimated;            
            //functions
            void map_SE3_to_pose(Sophus::SE3d pose);
            void set_image_to_publish(
                double hz, 
                const sensor_msgs::CompressedImageConstPtr & rgbmsg
            );
            void terminal_msg_display(double hz);
            void log(double ms);
            //functions
            Eigen::Vector3d q2rpy(Eigen::Quaterniond q);
            Eigen::Quaterniond rpy2q(Eigen::Vector3d rpy);
            Eigen::Vector3d q_rotate_vector(Eigen::Quaterniond q, Eigen::Vector3d v);
            

        //below is the courtesy of UZH Faessler et al.
        //which was used for calculation of the twists in this project
            /*
                @inproceedings{faessler2014monocular,
                title={A monocular pose estimation system based on infrared leds},
                author={Faessler, Matthias and Mueggler, Elias and Schwabe, Karl and Scaramuzza, Davide},
                booktitle={2014 IEEE international conference on robotics and automation (ICRA)},
                pages={907--913},
                year={2014},
                organization={IEEE}
                }
            */

            //twist for correspondence search
            //objects
            Eigen::Matrix4d pose_previous;
            Eigen::Matrix4d pose_current;
            Eigen::Matrix4d pose_predicted;   
                /*------------------------*/
            Eigen::Isometry3d led_pose_previous;   
            Eigen::VectorXd led_twist_current;    
                /*------------------------*/
            double time_previous = 0;
            double time_current = 0;
            double time_predicted = 0;
            int global_counter = 0;

            //functions
            void set_pose_predict();
            void set_twist_estimate(Eigen::Isometry3d led_pose_current);

            Eigen::VectorXd logarithmMap(Eigen::Matrix4d trans);
            Eigen::Matrix4d exponentialMap(Eigen::VectorXd& twist);
            Eigen::Matrix3d skewSymmetricMatrix(Eigen::Vector3d w);

//---------------------------------------------------------------------------------------
            virtual void onInit()
            {                
                ros::NodeHandle& nh = getMTNodeHandle();
                ROS_INFO("LED Nodelet Initiated...");

                RosTopicConfigs configs(nh, "/led");
                                
            //load POI_extract config
                nh.getParam("/alan_master/LANDING_DISTANCE", LANDING_DISTANCE);     
                nh.getParam("/alan_master/BINARY_threshold", BINARY_THRES);     
                nh.getParam("/alan_master/frame_width", _width);
                nh.getParam("/alan_master/frame_height", _height);

                // #define CAR_POSE_TOPIC POSE_SUB_TOPIC_A
                // std::cout<<CAR_POSE_TOPIC<<std::endl;
                // nh.getParam("/alan_master/CAR_POSE_TOPIC", CAR_POSE_TOPIC);
                // nh.getParam("/alan_master/UAV_POSE_TOPIC", UAV_POSE_TOPIC);

                
            //load camera intrinsics
                Eigen::Vector4d intrinsics_value;
                XmlRpc::XmlRpcValue intrinsics_list;
                nh.getParam("/alan_master/cam_intrinsics_455", intrinsics_list);
                                                
                for(int i = 0; i < 4; i++)
                {
                    intrinsics_value[i] = intrinsics_list[i];
                }

                cameraMat <<    
                    intrinsics_value[0], 0, intrinsics_value[2], 
                    0, intrinsics_value[1], intrinsics_value[3],
                    0, 0,  1; 

                // std::cout<<cameraMat<<std::endl;

                cameraEX.resize(6);
                XmlRpc::XmlRpcValue extrinsics_list;
                
                nh.getParam("/alan_master/cam_ugv_extrinsics", extrinsics_list);                
                
                for(int i = 0; i < 6; i++)
                {                    
                    cameraEX(i) = extrinsics_list[i];                    
                }

                LEDEX.resize(6);
                XmlRpc::XmlRpcValue extrinsics_list_led;

                nh.getParam("/alan_master/led_uav_extrinsics", extrinsics_list_led);

                for(int i = 0; i < 6; i++)
                {
                    LEDEX(i) = extrinsics_list_led[i];
                }


                q_c2b = rpy2q(
                    Eigen::Vector3d(
                        cameraEX(3) / 180 * M_PI,
                        cameraEX(4) / 180 * M_PI,
                        cameraEX(5) / 180 * M_PI              
                    )
                );

                t_c2b.translation() = Eigen::Vector3d(
                    cameraEX(0),
                    cameraEX(1),
                    cameraEX(2)
                );

                q_c2b = q_c2b;
                t_c2b.translation() = Eigen::Vector3d(0,0,0) + t_c2b.translation();

                ugv_cam_pose = t_c2b * q_c2b;//cam_to_body

                Eigen::Matrix<double, 4, 4> cam_to_body;
                cam_to_body << 
                    0,0,1,0,
                    -1,0,0,0,
                    0,-1,0,0,
                    0,0,0,1;
                led_cambody_pose = Eigen::Isometry3d(cam_to_body);
                q_led_cambody = Eigen::Quaterniond(led_cambody_pose.rotation());


                led_twist_current.resize(6);
                // t_led_cambody = Eigen::
                // cam_origin_in_body_frame = ugv_cam_pose.rotation() * Eigen::Vector3d(0.0,0.0,0.0) + ugv_cam_pose.translation();
                // cam_origin = cam_origin_in_body_frame;
                // ugv_cam_pose = ugv_cam_pose.inverse();



            // Eigen::Matrix3d body_to_cam;//rpy = 0 -90 90
            // body_to_cam << 
            //     0.0000000,  -1.0000000,  0.0000000,
            //     0.0000000,  0.0000000,  -1.0000000,
            //     1.0000000, 0.0000000,  0.0000000;

            //load LED potisions in body frame
                XmlRpc::XmlRpcValue LED_list;
                nh.getParam("/alan_master/LED_positions", LED_list); 

                std::vector<double> norm_of_x_points, norm_of_y_points, norm_of_z_points;

                std::cout<<"\nPts on body frame (X Y Z):\n";
                for(int i = 0; i < LED_list.size(); i++)
                {
                    Eigen::Vector3d temp(LED_list[i]["x"], LED_list[i]["y"], LED_list[i]["z"]);
                    
                    norm_of_x_points.push_back(temp.x());
                    norm_of_y_points.push_back(temp.y());
                    norm_of_z_points.push_back(temp.z());                    
                    std::cout<<"-----"<<std::endl;
                    std::cout<<temp.x()<<" "<<temp.y()<<" "<<temp.z()<<" "<<std::endl;                    
                    pts_on_body_frame.push_back(temp);
                }   
                std::cout<<std::endl;

                LED_no = pts_on_body_frame.size();

                nh.getParam("/alan_master/LED_r_number", LED_r_no);
                nh.getParam("/alan_master/LED_g_number", LED_g_no);


            //load outlier rejection info
                nh.getParam("/alan_master/MAD_dilate", MAD_dilate);
                nh.getParam("/alan_master/MAD_max", MAD_max);

                MAD_x_threshold = (calculate_MAD(norm_of_x_points) * MAD_dilate > MAD_max ? MAD_max : calculate_MAD(norm_of_x_points) * MAD_dilate);
                MAD_y_threshold = (calculate_MAD(norm_of_y_points) * MAD_dilate > MAD_max ? MAD_max : calculate_MAD(norm_of_y_points) * MAD_dilate);
                MAD_z_threshold = (calculate_MAD(norm_of_z_points) * MAD_dilate > MAD_max ? MAD_max : calculate_MAD(norm_of_z_points) * MAD_dilate);

                // std::cout<<MAD_x_threshold<<std::endl;
                // std::cout<<MAD_y_threshold<<std::endl;
                // std::cout<<MAD_z_threshold<<std::endl;

                LED_no = pts_on_body_frame.size();
                                                                
            //subscribe
                
                subimage.subscribe(nh, configs.getTopicName(COLOR_SUB_TOPIC), 1);                
                subdepth.subscribe(nh, configs.getTopicName(DEPTH_SUB_TOPIC), 1);                
                sync_.reset(new sync( MySyncPolicy(10), subimage, subdepth));            
                sync_->registerCallback(boost::bind(&LedNodelet::camera_callback, this, _1, _2));                                

                
                uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                //only used for validation stage
                    (configs.getTopicName(UAV_POSE_SUB_TOPIC), 1, &LedNodelet::uav_pose_callback, this);
            
                ugv_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    (configs.getTopicName(UGV_POSE_SUB_TOPIC), 1, &LedNodelet::ugv_pose_callback, this);

                uav_setpt_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("/planner_server/traj/pose", 1, &LedNodelet::uav_setpt_callback, this);
            //publish
                image_transport::ImageTransport image_transport_(nh);

                pubimage = image_transport_.advertise("/processed_image",1);
                pubimage_input = image_transport_.advertise("/input_image", 1);

                
                ledpose_pub = nh.advertise<geometry_msgs::PoseStamped>
                //only used for validation stage
                                (configs.getTopicName(LED_POSE_PUB_TOPIC), 1, true);
                
                ledodom_pub = nh.advertise<nav_msgs::Odometry>
                                (configs.getTopicName(LED_ODOM_PUB_TOPIC),1 , true);
                
                ugvpose_pub = nh.advertise<geometry_msgs::PoseStamped>
                //only used for validation stage
                                (configs.getTopicName(UGV_POSE_PUB_TOPIC), 1, true); 
                
                
                uavpose_pub = nh.advertise<geometry_msgs::PoseStamped>
                //only used for validation stage
                                (configs.getTopicName(UAV_POSE_PUB_TOPIC), 1, true);

                
                campose_pub = nh.advertise<geometry_msgs::PoseStamped>
                //only used for validation stage
                                (configs.getTopicName(CAM_POSE_PUB_TOPIC), 1, true);    
                
                record_led_pub = nh.advertise<alan_state_estimation::alan_log>
                                ("/alan_state_estimation/led/led_log", 1);
                
                record_uav_pub = nh.advertise<alan_state_estimation::alan_log>
                                ("/alan_state_estimation/led/uav_log", 1);
            }


              
    };

    PLUGINLIB_EXPORT_CLASS(alan::LedNodelet, nodelet::Nodelet)
}

#endif