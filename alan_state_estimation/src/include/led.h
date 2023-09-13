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
#include "alan_state_estimation/alan_log.h"

#include "aiekf.hpp"
#include "cameraModel.hpp"

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
    class LedNodelet : public nodelet::Nodelet, private kf::aiekf
    {
        //primary objects
            //frames
            cv::Mat frame, display, hsv, frame_temp;
            cv::Mat frame_input;
            cv::Mat im_with_keypoints;
            cv::Mat frame_initial_thresholded;
            
            //time related
            std_msgs::Header led_pose_header, led_pose_header_previous;
            double last_request = 0;

            //camera related
            // Eigen::MatrixXd cameraMat = Eigen::MatrixXd::Zero(3,3);
            Eigen::VectorXd cameraEX;

            //LED config and correspondences
            std::vector<Eigen::Vector3d> pts_on_body_frame;

            // detect_no, correspondences(in order of 0->5)
            std::tuple<int, std::vector<correspondence::matchid>> corres_global_current;
            std::tuple<int, std::vector<correspondence::matchid>> corres_global_previous;

            Eigen::VectorXd LEDEX;

            //poses
            Sophus::SE3d pose_global_sophus;
            Sophus::SE3d pose_epnp_sophus, pose_depth_sophus;
            
            Sophus::SE3d pose_cam_inWorld_SE3;
            Sophus::SE3d pose_ugv_inWorld_SE3;
            Sophus::SE3d pose_uav_inWorld_SE3;
            Sophus::SE3d pose_led_inWorld_SE3;

            Sophus::SE3d pose_cam_inGeneralBodySE3;
            Sophus::SE3d pose_cam_inUgvBody_SE3;
            Sophus::SE3d pose_led_inUavBodyOffset_SE3;

            geometry_msgs::PoseStamped ugv_pose_msg, 
                                       uav_pose_msg,
                                       uav_stpt_msg;
            
            
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
            typedef message_filters::Synchronizer<MySyncPolicy> sync;// (MySyncPolicy(10), subimage, subdepth);
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
       
            double get_reprojection_error(
                std::vector<Eigen::Vector3d> pts_3d, 
                std::vector<Eigen::Vector2d> pts_2d, 
                Sophus::SE3d pose, 
                bool draw_reproject
            ) override
            {
                double e = 0;

                Eigen::Vector2d reproject, error;

                for(int i = 0; i < pts_3d.size(); i++)
                {
                    reproject = reproject_3D_2D(pts_3d[i], pose);
                    error = pts_2d[i] - reproject;
                    e = e + error.norm();

                    if(draw_reproject)
                        cv::circle(display, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(0,255,0),-1);
                    
                }

                return e;
            };         

        //main process & kf
            // kf::MEASUREMENT global_meas_at_k; 
            // Eigen::MatrixXd Q_init_;
            // Eigen::MatrixXd R_init_;
            // double Q_alpha_, R_beta_;

            void apiKF(int DOKF);
            void recursive_filtering(cv::Mat& frame, cv::Mat depth);        

        //pnp + BA
            void solve_pnp_initial_pose(std::vector<Eigen::Vector2d> pts_2d, std::vector<Eigen::Vector3d> body_frame_pts);
            // void optimize(Sophus::SE3d& pose, std::vector<Eigen::Vector3d> pts_3d_exists, std::vector<Eigen::Vector2d> pts_2d_detected);
            //     //converge problem need to be solved //-> fuck you, your Jacobian was wrong
            // void solveJacobian(Eigen::Matrix<double, 2, 6>& Jacob, Sophus::SE3d pose, Eigen::Vector3d point_3d);
        //LED extraction tool
            //objects
            double LANDING_DISTANCE = 0;
            int BINARY_THRES = 0;

            std::vector<Eigen::Vector3d> pts_on_body_frame_in_corres_order;
            std::vector<Eigen::Vector2d> pts_detected_in_corres_order;
            
            bool LED_pts_measurement(
                cv::Mat& frame, 
                cv::Mat& depth, 
                std::vector<Eigen::Vector2d>& pts_2d_detected
            );
            std::vector<Eigen::Vector2d> LED_extract_POI(cv::Mat& frame, cv::Mat depth);
            std::vector<Eigen::Vector2d> LED_extract_POI_alter(cv::Mat& frame, cv::Mat depth);
            std::vector<Eigen::Vector3d> pointcloud_generate(std::vector<Eigen::Vector2d> pts_2d_detected, cv::Mat depthimage);
            bool get_final_POI(std::vector<Eigen::Vector2d>& pts_2d_detected);

        //initiation & correspondence 
            //objects
            bool LED_tracker_initiated_or_tracked = false;
            int LED_no;
            int LED_r_no;
            int LED_g_no;
            // int last_frame_no;
            // int current_frame_no;
            std::vector<Eigen::Vector2d> pts_2d_detect_correct_order;
            //functions    
            void get_correspondence(
                std::vector<Eigen::Vector2d>& pts_2d_detected
            );
            std::vector<Eigen::Vector2d> shift2D(
                std::vector<Eigen::Vector2d>& pts_2D_previous,
                std::vector<Eigen::Vector2d>& pts_detect_current
            );
            void correspondence_search_2D2DCompare(
                std::vector<Eigen::Vector2d>& pts_2d_detected,
                std::vector<Eigen::Vector2d>& pts_2d_detected_previous
            );   
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
            geometry_msgs::PoseStamped led_pose_estimated_msg;
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
            // inline Eigen::Vector3d q2rpy(Eigen::Quaterniond q);
            // inline Eigen::Quaterniond rpy2q(Eigen::Vector3d rpy);
            // inline Eigen::Vector3d q_rotate_vector(Eigen::Quaterniond q, Eigen::Vector3d v);

            inline Sophus::SE3d posemsg_to_SE3(const geometry_msgs::PoseStamped pose);
            inline geometry_msgs::PoseStamped SE3_to_posemsg(const Sophus::SE3d pose_on_SE3, const std_msgs::Header msgHeader);
   
            Eigen::VectorXd led_twist_current;    

//---------------------------------------------------------------------------------------
            virtual void onInit()
            {                
                ros::NodeHandle& nh = getMTNodeHandle();
                ROS_INFO("LED Nodelet Initiated...");

                RosTopicConfigs configs(nh, "/led");

                doALOTofConfigs(nh);
                                                 
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

            inline void doALOTofConfigs(ros::NodeHandle& nh)
            {
                POI_config(nh);
                camIntrinsic_config(nh);
                camExtrinsic_config(nh);
                LEDExtrinsicUAV_config(nh);
                CamInGeneralBody_config(nh);
                LEDInBodyAndOutlierSetting_config(nh);
                KF_config(nh);

                led_twist_current.resize(6);
            }

            inline void POI_config(ros::NodeHandle& nh)
            {
                // load POI_extract config
                nh.getParam("/alan_master/LANDING_DISTANCE", LANDING_DISTANCE);     
                nh.getParam("/alan_master/BINARY_threshold", BINARY_THRES);     
                nh.getParam("/alan_master/frame_width", _width);
                nh.getParam("/alan_master/frame_height", _height);

                // #define CAR_POSE_TOPIC POSE_SUB_TOPIC_A
                // std::cout<<CAR_POSE_TOPIC<<std::endl;
                // nh.getParam("/alan_master/CAR_POSE_TOPIC", CAR_POSE_TOPIC);
                // nh.getParam("/alan_master/UAV_POSE_TOPIC", UAV_POSE_TOPIC);
            }

            inline void camIntrinsic_config(ros::NodeHandle& nh)
            {
                // load camera intrinsics
                Eigen::Vector4d intrinsics_value;
                XmlRpc::XmlRpcValue intrinsics_list;
                nh.getParam("/alan_master/cam_intrinsics_455", intrinsics_list);
                                                
                for(int i = 0; i < 4; i++)
                {
                    intrinsics_value[i] = intrinsics_list[i];
                }
                
                cameraMat <<    
                    // inherintance here -> modifying value for all super/sub classes
                    intrinsics_value[0], 0, intrinsics_value[2], 
                    0, intrinsics_value[1], intrinsics_value[3],
                    0, 0,  1; 

            }

            inline void camExtrinsic_config(ros::NodeHandle& nh)
            {
                cameraEX.resize(6);
                XmlRpc::XmlRpcValue extrinsics_list;
                
                nh.getParam("/alan_master/cam_ugv_extrinsics", extrinsics_list);                
                
                for(int i = 0; i < 6; i++)
                {                    
                    cameraEX(i) = extrinsics_list[i];                    
                }

                pose_cam_inUgvBody_SE3 = Sophus::SE3d(
                    rpy2q(
                        Eigen::Vector3d(
                            cameraEX(3) / 180 * M_PI,
                            cameraEX(4) / 180 * M_PI,
                            cameraEX(5) / 180 * M_PI              
                        )
                    ),
                    Eigen::Vector3d(
                        cameraEX(0),
                        cameraEX(1),
                        cameraEX(2)
                    )            
                );
            }

            inline void LEDExtrinsicUAV_config(ros::NodeHandle& nh)
            {
                // load LED extrinsics
                LEDEX.resize(6);
                XmlRpc::XmlRpcValue extrinsics_list_led;

                nh.getParam("/alan_master/led_uav_extrinsics", extrinsics_list_led);

                for(int i = 0; i < 6; i++)
                {
                    LEDEX(i) = extrinsics_list_led[i];
                }

                pose_led_inUavBodyOffset_SE3 = Sophus::SE3d(
                    Eigen::Matrix3d::Identity(),
                    Eigen::Vector3d(
                        LEDEX(0),
                        LEDEX(1),
                        LEDEX(2)
                    )
                );
            }

            inline void CamInGeneralBody_config(ros::NodeHandle& nh)
            {
                // load cam in general body frame                
                Eigen::Matrix3d cam_to_body_rot;
                cam_to_body_rot << 
                    0,0,1,
                    -1,0,0,
                    0,-1,0;

                pose_cam_inGeneralBodySE3 = Sophus::SE3d(
                    cam_to_body_rot, 
                    Eigen::Vector3d::Zero()
                );
            }

            inline void LEDInBodyAndOutlierSetting_config(ros::NodeHandle& nh)
            {
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
            }
              
            inline void KF_config(ros::NodeHandle& nh)
            {
                double Q_val;
                double R_val;

                nh.getParam("/alan_master/Q_val", Q_val);
                nh.getParam("/alan_master/R_val", R_val);
                nh.getParam("/alan_master/Q_alpha", QAdaptiveAlpha);
                nh.getParam("/alan_master/R_beta", RAdaptiveBeta);
                nh.getParam("/alan_master/kf_size", kf_size);
                nh.getParam("/alan_master/kfZ_size", kfZ_size);
                nh.getParam("/alan_master/OPT_MAX_ITERATION", MAX_ITERATION);
                nh.getParam("/alan_master/CONVERGE_THRESHOLD", CONVERGE_THRESHOLD);

                Q_init.resize(kf_size, kf_size);
                Q_init.setIdentity();
                Q_init = Q_init * Q_val;

                R_init.resize(kfZ_size, kfZ_size);
                R_init.setIdentity();
                R_init = R_init * R_val;
            }
    };

    PLUGINLIB_EXPORT_CLASS(alan::LedNodelet, nodelet::Nodelet)
}

#endif