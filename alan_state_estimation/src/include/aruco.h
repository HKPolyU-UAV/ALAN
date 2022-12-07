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
#include <sensor_msgs/Image.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <sophus/se3.hpp>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>


#include <pthread.h>
#include <boost/thread.hpp>

#include "tools/RosTopicConfigs.h"


namespace alan
{
    class ArucoNodelet : public nodelet::Nodelet
    {
        private:
            pthread_t tid;

            //publisher
            ros::Publisher nodelet_pub;
            ros::Publisher mypose_pub, arucopose_pub, campose_pub, ugvpose_pub, uavpose_pub;
            image_transport::Publisher pubimage;
            
            //subscriber
            message_filters::Subscriber<sensor_msgs::CompressedImage> subimage;
            message_filters::Subscriber<sensor_msgs::Image> subdepth;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
            typedef message_filters::Synchronizer<MySyncPolicy> sync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<sync> sync_;
            
            //pose-processing            
            void ugv_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
            void uav_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
            ros::Subscriber ugv_pose_sub, uav_pose_sub;
            geometry_msgs::PoseStamped ugv_pose_msg, uav_pose_msg;

            Eigen::Isometry3d ugv_pose;
            Eigen::Isometry3d cam_pose;
            Eigen::Translation3d ugvcam_t_;
            Eigen::Quaterniond ugvcam_q_;
            Eigen::VectorXd cameraEX;

            Eigen::Quaterniond q_cam;
            Eigen::Vector3d t_cam;

            Eigen::Quaterniond q_c2b;
            Eigen::Translation3d t_c2b;

            Eigen::Quaterniond q_aruco_2_body;

            //private variables
            cv::Mat frame;
            vector<Eigen::Vector3d> body_frame_pts;
            Eigen::MatrixXd cameraMat = Eigen::MatrixXd::Zero(3,3);
            std_msgs::Bool test;
            geometry_msgs::PoseStamped pose_estimated;
            bool add_noise = false;       
            int temp_i = 0;     

            cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);
            cv::Mat distCoeffs;
            std::vector<cv::Vec3d> rvecs, tvecs;
            Sophus::SE3d pose_aruco;


            //functions
            void camera_callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth);

            //pnp + BA
            void pose_w_aruco_pnp(cv::Mat& frame);

            Eigen::Vector2d reproject_3D_2D(Eigen::Vector3d P, Sophus::SE3d pose);
            Eigen::Vector2d reproject_3D_2D_temp(Eigen::Vector3d P, Sophus::SE3f pose);

            void get_initial_pose(vector<Eigen::Vector2d> pts_2d, vector<Eigen::Vector3d> body_frame_pts, Eigen::Matrix3d& R, Eigen::Vector3d& t);

            bool aruco_detect(cv::Mat& frame, vector<Eigen::Vector2d>& pts_2d);

            void solveJacobian(Eigen::Matrix<double, 2, 6>& Jacob, Sophus::SE3d pose, Eigen::Vector3d point_3d);

            void optimize(Sophus::SE3d& pose, vector<Eigen::Vector3d> pts_3d_exists, vector<Eigen::Vector2d> pts_2d_detected);//converge problem need to be solved //-> fuck you, your Jacobian was wrong

            void map_SE3_to_pose(Sophus::SE3d pose);

            Sophus::SE3d pose_add_noise(Eigen::Vector3d t, Eigen::Matrix3d R);

            void use_pnp_instead(cv::Mat frame, vector<Eigen::Vector2d> pts_2d_detect, Sophus::SE3d& pose);

            //ICP
            void pose_w_aruco_icp(cv::Mat& rgbframe, cv::Mat& depthframe);

            void solveicp_svd(vector<Eigen::Vector3d> pts_3d, vector<Eigen::Vector3d> body_frame_pts, Eigen::Matrix3d& R, Eigen::Vector3d& t);

            vector<Eigen::Vector3d> pointcloud_generate(vector<Eigen::Vector2d> pts_2d_detected, cv::Mat depthimage);

            Eigen::Vector3d get_CoM(vector<Eigen::Vector3d> pts_3d);


            Eigen::Vector3d q2rpy(Eigen::Quaterniond q);
            Eigen::Quaterniond rpy2q(Eigen::Vector3d rpy);
            Eigen::Vector3d q_rotate_vector(Eigen::Quaterniond q, Eigen::Vector3d v);

//---------------------------------------------------------------------------------------
            virtual void onInit() 
            {
                ros::NodeHandle& nh = getMTNodeHandle();
                ROS_INFO("Aruco Nodelet Initiated...");

                RosTopicConfigs configs(nh, "/alan_master");

                // cout<<"in oninit..."<<temp<<endl;
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

                cameraMatrix.at<double>(0,0) = intrinsics_value[0];
                cameraMatrix.at<double>(1,1) = intrinsics_value[1];
                cameraMatrix.at<double>(0,2) = intrinsics_value[2];
                cameraMatrix.at<double>(1,2) = intrinsics_value[3];

                distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

                cameraEX.resize(6);
                XmlRpc::XmlRpcValue extrinsics_list;
                nh.getParam("/alan_master/cam_extrinsics_d455", extrinsics_list);

                for(int i = 0; i < 6; i++)
                {
                    cameraEX(i) = extrinsics_list[i];
                }

                // cout<<cameraEX<<endl;

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

                q_aruco_2_body = rpy2q(
                    Eigen::Vector3d(
                        0.0 / 180.0 * M_PI,
                        -90 / 180.0 * M_PI,
                        180 / 180.0 * M_PI
                    )
                );

                cout<<q_aruco_2_body.toRotationMatrix()<<endl;


                // cout<<cameraMat.inverse()<<endl;

                Eigen::Matrix3d body_to_cam;//rpy = 0 -90 90
                body_to_cam << 
                    0.0000000,  -1.0000000,  0.0000000,
                    0.0000000,  0.0000000,  -1.0000000,
                    1.0000000, 0.0000000,  0.0000000;


                //load LED potisions in body frame
                XmlRpc::XmlRpcValue LED_list;
                nh.getParam("/alan_master/ARUCO_positions", LED_list); 
                cout<<endl<<"Pts here:..."<<endl;

                for(int i = 0; i < LED_list.size(); i++)
                {
                    Eigen::Vector3d temp(LED_list[i]["x"], LED_list[i]["y"], LED_list[i]["z"]);
                    // temp = body_to_cam * temp;
                    cout<<temp<<endl;
                    cout<<"------"<<endl;
                    body_frame_pts.push_back(temp);
                }
                cout<<endl;
                
                //initialize publisher
                arucopose_pub = nh.advertise<geometry_msgs::PoseStamped>(configs.getTopicName(POSE_PUB_TOPIC_A), 1);
                mypose_pub = nh.advertise<geometry_msgs::PoseStamped>(configs.getTopicName(POSE_PUB_TOPIC_B), 1);

                ugvpose_pub = nh.advertise<geometry_msgs::PoseStamped>(configs.getTopicName(POSE_PUB_TOPIC_C), 1);
                campose_pub = nh.advertise<geometry_msgs::PoseStamped>(configs.getTopicName(POSE_PUB_TOPIC_D), 1);
                uavpose_pub = nh.advertise<geometry_msgs::PoseStamped>(configs.getTopicName(POSE_PUB_TOPIC_E), 1);
                // test_pub = nh.advertise<std_msgs::Bool>("/ob_found",1);

                // pthread_create(&tid, NULL, ArucoNodelet::PubMainLoop, (void*)this);

                image_transport::ImageTransport image_transport_(nh);
                pubimage = image_transport_.advertise(configs.getTopicName(IMAGE_PUB_TOPIC_A),1);

                //initialize subscribe
                // subimage = nh.subscribe("/camera/color/image_raw/compressed", 1, &ArucoNodelet::camera_callback, this);
                subimage.subscribe(nh, configs.getTopicName(CAMERA_SUB_TOPIC_A), 1);
                subdepth.subscribe(nh, configs.getTopicName(DEPTH_SUB_TOPIC_A), 1);                
                sync_.reset(new sync( MySyncPolicy(10), subimage, subdepth));            
                sync_->registerCallback(boost::bind(&ArucoNodelet::camera_callback, this, _1, _2));
                
                uav_pose_sub = nh.subscribe(configs.getTopicName(POSE_SUB_TOPIC_A), 1, &ArucoNodelet::uav_pose_callback, this);
                ugv_pose_sub = nh.subscribe(configs.getTopicName(POSE_SUB_TOPIC_B), 1, &ArucoNodelet::ugv_pose_callback, this);

                
            }     

            public:
                static void* PubMainLoop(void* tmp);   

    };
    
    PLUGINLIB_EXPORT_CLASS(alan::ArucoNodelet, nodelet::Nodelet)
}


#endif