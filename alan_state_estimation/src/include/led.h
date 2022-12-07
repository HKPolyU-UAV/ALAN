/*
    A ROS Nodelet H + CPP file for
    vision-based relative localization (6D) for UAV and UGV.
    Created on 28/07/2022
    (c) pattylo
    from the RCUAS of Hong Kong Polytechnic University
*/

/**
 * \file led.h + led.cpp
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

#include "tools/munkres.hpp"

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
            ros::Time led_pose_stamp;
            double last_request = 0;

            //camera related
            Eigen::MatrixXd cameraMat = Eigen::MatrixXd::Zero(3,3);
            Eigen::VectorXd cameraEX;
            Eigen::Quaterniond q_c2b;
            Eigen::Translation3d t_c2b;

            //LED config and correspondences
            vector<Eigen::Vector3d> pts_on_body_frame;
            vector<correspondence::matchid> corres_global;

            //poses
            Sophus::SE3d pose_global_sophus;
            geometry_msgs::PoseStamped ugv_pose_msg, uav_pose_msg;
            Eigen::Isometry3d ugv_pose;
            Eigen::Isometry3d cam_pose;
            Eigen::Isometry3d ugv_cam_pose, led_cambody_pose;
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
            vector<cv::KeyPoint> blobs_for_initialize;
            int _width = 0, _height = 0;
            string CAR_POSE_TOPIC;
            string UAV_POSE_TOPIC;

        //subscribe                                    
            //objects
            message_filters::Subscriber<sensor_msgs::CompressedImage> subimage;
            message_filters::Subscriber<sensor_msgs::Image> subdepth;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
            typedef message_filters::Synchronizer<MySyncPolicy> sync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<sync> sync_;                    
            ros::Subscriber ugv_pose_sub, uav_pose_sub;
            //functions
            void camera_callback(const sensor_msgs::CompressedImage::ConstPtr & rgbimage, const sensor_msgs::Image::ConstPtr & depth);            
            void ugv_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
            void uav_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
        
        //publisher 
            //objects
            ros::Publisher ledpose_pub, campose_pub, ugvpose_pub, uavpose_pub;
            image_transport::Publisher pubimage;
            image_transport::Publisher pubimage_input;
            //functions
        
        //solve pose & tools
            void solve_pose_w_LED(cv::Mat& frame, cv::Mat depth);             
            Eigen::Vector2d reproject_3D_2D(Eigen::Vector3d P, Sophus::SE3d pose);            

        //main process
            void recursive_filtering(cv::Mat& frame, cv::Mat depth);
            bool search_corres_and_pose_predict(vector<Eigen::Vector2d> pts_2d_detect);

        //pnp + BA
            void solve_pnp_initial_pose(vector<Eigen::Vector2d> pts_2d, vector<Eigen::Vector3d> body_frame_pts, Eigen::Matrix3d& R, Eigen::Vector3d& t);
            void optimize(Sophus::SE3d& pose, vector<Eigen::Vector3d> pts_3d_exists, vector<Eigen::Vector2d> pts_2d_detected);
                //converge problem need to be solved //-> fuck you, your Jacobian was wrong
            void solveJacobian(Eigen::Matrix<double, 2, 6>& Jacob, Sophus::SE3d pose, Eigen::Vector3d point_3d);

        //LED extraction tool
            //objects
            double LANDING_DISTANCE = 0;
            int BINARY_THRES = 0;
            vector<Eigen::Vector2d> LED_extract_POI(cv::Mat& frame, cv::Mat depth);
            vector<Eigen::Vector3d> pointcloud_generate(vector<Eigen::Vector2d> pts_2d_detected, cv::Mat depthimage);
            bool get_final_POI(vector<Eigen::Vector2d>& pts_2d_detected);

        //initiation & correspondence 
            //objects
            correspondence::munkres hungarian1; 
            correspondence::munkres hungarian2; 
            bool LED_tracker_initiated_or_tracked = false;
            int LED_no;
            //functions       
            void correspondence_search_kmeans(vector<Eigen::Vector2d> pts_2d_detected);        
            bool LED_tracking_initialize(cv::Mat& frame, cv::Mat depth);
                      
        //outlier rejection 
            //objects
            cv::Point3f pcl_center_point_wo_outlier_previous;
            double MAD_x_threshold = 0, MAD_y_threshold = 0, MAD_z_threshold = 0;
            double min_blob_size = 0;
            //functions
            void reject_outlier(vector<Eigen::Vector2d>& pts_2d_detect, cv::Mat depth);
            double calculate_MAD(vector<double> norm_of_points);

        //reinitialization
            bool reinitialization(vector<Eigen::Vector2d> pts_2d_detect, cv::Mat depth);
            
        //publish
            //objects
            geometry_msgs::PoseStamped led_pose_estimated;
            //functions
            void map_SE3_to_pose(Sophus::SE3d pose);
            void set_image_to_publish(
                double t2, 
                double t1, 
                const sensor_msgs::CompressedImageConstPtr & rgbmsg
            );
            //functions
            Eigen::Vector3d q2rpy(Eigen::Quaterniond q);
            Eigen::Quaterniond rpy2q(Eigen::Vector3d rpy);
            Eigen::Vector3d q_rotate_vector(Eigen::Quaterniond q, Eigen::Vector3d v);


            virtual void onInit()
            {                
                ros::NodeHandle& nh = getMTNodeHandle();
                ROS_INFO("LED Nodelet Initiated...");
                                
            //load POI_extract config
                nh.getParam("/alan_master/LANDING_DISTANCE", LANDING_DISTANCE);     
                nh.getParam("/alan_master/BINARY_threshold", BINARY_THRES);     
                nh.getParam("/alan_master/frame_width", _width);
                nh.getParam("/alan_master/frame_height", _height);
                nh.getParam("/alan_master/CAR_POSE_TOPIC", CAR_POSE_TOPIC);
                nh.getParam("/alan_master/UAV_POSE_TOPIC", UAV_POSE_TOPIC);

                
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

                cameraEX.resize(6);
                XmlRpc::XmlRpcValue extrinsics_list;
                
                nh.getParam("/alan_master/cam_extrinsics_d455", extrinsics_list);                
                
                for(int i = 0; i < 6; i++)
                {                    
                    cameraEX(i) = extrinsics_list[i];                    
                }

                // cout<<Eigen::Vector3d(
                //         cameraEX(3) / 180 * M_PI,
                //         cameraEX(4) / 180 * M_PI,
                //         cameraEX(5) / 180 * M_PI              
                //     )<<endl;

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
                // t_led_cambody = Eigen::
                // cam_origin_in_body_frame = ugv_cam_pose.rotation() * Eigen::Vector3d(0.0,0.0,0.0) + ugv_cam_pose.translation();
                // cam_origin = cam_origin_in_body_frame;
                // ugv_cam_pose = ugv_cam_pose.inverse();



            Eigen::Matrix3d body_to_cam;//rpy = 0 -90 90
            body_to_cam << 
                0.0000000,  -1.0000000,  0.0000000,
                0.0000000,  0.0000000,  -1.0000000,
                1.0000000, 0.0000000,  0.0000000;

            //load LED potisions in body frame
                XmlRpc::XmlRpcValue LED_list;
                nh.getParam("/alan_master/LED_positions", LED_list); 

                for(int i = 0; i < LED_list.size(); i++)
                {
                    Eigen::Vector3d temp(LED_list[i]["x"], LED_list[i]["y"], LED_list[i]["z"]);
                    temp = body_to_cam * temp;
                    cout<<temp<<endl;
                    cout<<"-----"<<endl;
                    pts_on_body_frame.push_back(temp);
                }   
                LED_no = pts_on_body_frame.size();


                // vector<Eigen::Vector2d> pts_2d_temp;
                // XmlRpc::XmlRpcValue pts_2d_list;
                // nh.getParam("/alan_master/pts_2d_list", pts_2d_list); 

                // for(int i = 0; i < pts_2d_list.size(); i++)
                // {
                //     Eigen::Vector2d temp(pts_2d_list[i]["x"], pts_2d_list[i]["y"]);
                //     pts_2d_temp.push_back(temp);
                // }   

                // Eigen::Matrix3d R;
                // Eigen::Vector3d t;

                // solve_pnp_initial_pose(pts_2d_temp, pts_on_body_frame, R, t);
                // pose_global_sophus = Sophus::SE3d(R,t);

                // double reproject_error = 0;

                // for(int i = 0 ; i < pts_on_body_frame.size(); i++)
                // {
                //     Eigen::Vector2d reproject = reproject_3D_2D(
                //         pts_on_body_frame[i], 
                //         pose_global_sophus
                //     ); 

                //     double temp_error = (reproject - pts_2d_temp[i]).norm();
                //     //can save as reprojection for next frame
                //     reproject_error = reproject_error + temp_error;
                    
                // }

                // cout<<"final: error"<<reproject_error<<endl;
                // cout<<"result:"<<endl;
                // cout<<R<<endl;
                // cout<<t<<endl;

                // ros::shutdown();



            //load outlier rejection info
                nh.getParam("/alan_master/MAD_x_threshold", MAD_x_threshold);
                nh.getParam("/alan_master/MAD_y_threshold", MAD_y_threshold);
                nh.getParam("/alan_master/MAD_z_threshold", MAD_z_threshold);

                LED_no = pts_on_body_frame.size();
                                                                
            //subscribe
                subimage.subscribe(nh, "/camera/color/image_raw/compressed", 1);
                subdepth.subscribe(nh, "/camera/aligned_depth_to_color/image_raw", 1);                
                sync_.reset(new sync( MySyncPolicy(10), subimage, subdepth));            
                sync_->registerCallback(boost::bind(&LedNodelet::camera_callback, this, _1, _2));
                
                ugv_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    (CAR_POSE_TOPIC, 1, &LedNodelet::ugv_pose_callback, this);
                uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    (UAV_POSE_TOPIC, 1, &LedNodelet::uav_pose_callback, this);

            //publish
                image_transport::ImageTransport image_transport_(nh);

                pubimage = image_transport_.advertise("/processed_image",1);
                pubimage_input = image_transport_.advertise("/input_image", 1);

                ledpose_pub = nh.advertise<geometry_msgs::PoseStamped>
                                ("/alan_state_estimation/LED/pose", 1, true);
                campose_pub = nh.advertise<geometry_msgs::PoseStamped>
                                ("/alan_state_estimation/CAM/pose", 1, true); 
                ugvpose_pub = nh.advertise<geometry_msgs::PoseStamped>
                                ("/alan_state_estimation/UGV/pose", 1, true); 
                uavpose_pub = nh.advertise<geometry_msgs::PoseStamped>
                                ("/alan_state_estimation/UAV/pose", 1, true);

                

            }


            //below is the courtesy of UZH Faessler et al.
            //which was tested but NOT used in this project
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
            double time_previous = 0;
            double time_current = 0;
            double time_predicted = 0;
            int global_counter = 0;
            //functions
            void set_pose_predict();
            Eigen::VectorXd logarithmMap(Eigen::Matrix4d trans);
            Eigen::Matrix4d exponentialMap(Eigen::VectorXd& twist);
            Eigen::Matrix3d skewSymmetricMatrix(Eigen::Vector3d w);  
    };

    PLUGINLIB_EXPORT_CLASS(alan::LedNodelet, nodelet::Nodelet)
}

#endif