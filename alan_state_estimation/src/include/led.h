//mainly for relative localization between the two

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
        public:
        private:
            //general objects
            cv::Mat frame, display;
            Eigen::MatrixXd cameraMat = Eigen::MatrixXd::Zero(3,3);
            vector<correspondence::matchid> LED_v_Detected;
            vector<Eigen::Vector3d> pts_on_body_frame, pts_on_body_frame_normalized;
            
            vector<Eigen::Vector2d> pts_obj_configuration;
            double x_avg_pts_config_ = 0;
            double y_avg_pts_config_ = 0;

            Sophus::SE3d pose_global;
            vector<correspondence::matchid> corres_global;
            

            //temp objects
            // double temp = 0;
            int i = 0;
            int detect_no = 0;
            double BA_error = 0;
            vector<cv::KeyPoint> blobs_for_initialize;

            //subscribe            
            void camera_callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth);
            
            message_filters::Subscriber<sensor_msgs::CompressedImage> subimage;
            message_filters::Subscriber<sensor_msgs::Image> subdepth;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
            typedef message_filters::Synchronizer<MySyncPolicy> sync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<sync> sync_;                    
            
            //publisher 
            ros::Publisher pubpose;

            image_transport::Publisher pubimage;





            //solve pose & tools
            void solve_pose_w_LED(cv::Mat& frame, cv::Mat depth); 

            
            Eigen::Vector2d reproject_3D_2D(Eigen::Vector3d P, Sophus::SE3d pose);

            //pnp + BA
            void solve_pnp_initial_pose(vector<Eigen::Vector2d> pts_2d, vector<Eigen::Vector3d> body_frame_pts, Eigen::Matrix3d& R, Eigen::Vector3d& t);

            void optimize(Sophus::SE3d& pose, vector<Eigen::Vector3d> pts_3d_exists, vector<Eigen::Vector2d> pts_2d_detected);//converge problem need to be solved //-> fuck you, your Jacobian was wrong

            void solveJacobian(Eigen::Matrix<double, 2, 6>& Jacob, Sophus::SE3d pose, Eigen::Vector3d point_3d);




            //LED extraction tool
            vector<Eigen::Vector2d> LED_extract_POI(cv::Mat& frame, cv::Mat depth);

            vector<Eigen::Vector3d> pointcloud_generate(vector<Eigen::Vector2d> pts_2d_detected, cv::Mat depthimage);
            
            double LANDING_DISTANCE = 0;
            int BINARY_THRES = 0;




            //initiation & correspondence        
            void correspondence_search(vector<Eigen::Vector3d> pts_3d_detected, vector<Eigen::Vector2d> pts_2d_detected);    
            void correspondence_search_test(vector<Eigen::Vector3d> pts_3d_detected, vector<Eigen::Vector2d> pts_2d_detected);    

            bool LED_tracking_initialize(cv::Mat& frame, cv::Mat depth);

            vector<Eigen::Vector2d> pts_2d_normlization(vector<Eigen::Vector2d> pts_2d);

            correspondence::munkres hungarian; 
            bool LED_tracker_initiated = false;
            int LED_no;




            //main process
            void recursive_filtering(cv::Mat& frame, cv::Mat depth);


            //outlier rejection
            void reject_outlier(vector<Eigen::Vector3d>& pts_3d_detect, vector<Eigen::Vector2d>& pts_2d_detect);

            double calculate_MAD(vector<double> norm_of_points);

            cv::Point3f pcl_center_point_wo_outlier_previous;
            double MAD_x_threshold = 0, MAD_y_threshold = 0, MAD_z_threshold = 0;



            //publish
            void map_SE3_to_pose(Sophus::SE3d pose);

            geometry_msgs::PoseStamped uav_pose_estimated;
            

            //others
            void solveicp_svd(vector<Eigen::Vector3d> pts_3d, vector<Eigen::Vector3d> body_frame_pts, Eigen::Matrix3d& R, Eigen::Vector3d& t);
            
            Eigen::Vector3d get_CoM(vector<Eigen::Vector3d> pts_3d);

            void use_pnp_instead(cv::Mat frame, vector<Eigen::Vector2d> pts_2d_detect, vector<Eigen::Vector3d> pts_3d_detect, Sophus::SE3d& pose);




            virtual void onInit()
            {
                ros::NodeHandle& nh = getNodeHandle();

                cout<<"sup, we are now at led"<<endl;
                
                //load POT_extract config
                nh.getParam("/alan_master/LANDING_DISTANCE", LANDING_DISTANCE);     
                nh.getParam("/alan_master/BINARY_threshold", BINARY_THRES);     

                
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


                //load LED potisions in body frame
                XmlRpc::XmlRpcValue LED_list;
                nh.getParam("/alan_master/LED_positions", LED_list); 
                for(int i = 0; i < LED_list.size(); i++)
                {
                    Eigen::Vector3d temp(LED_list[i]["x"], LED_list[i]["y"], LED_list[i]["z"]);
                    pts_on_body_frame.push_back(temp);
                }   
                LED_no = pts_on_body_frame.size();

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

                //publish
                image_transport::ImageTransport image_transport_(nh);
                pubimage = image_transport_.advertise("/processed_image",1);

                //test algo here:
                
                

            }

            public:
                static void* PubMainLoop(void* tmp);

    };

    PLUGINLIB_EXPORT_CLASS(alan::LedNodelet, nodelet::Nodelet)
}

#endif