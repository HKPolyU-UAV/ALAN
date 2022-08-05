#include "essential.h"

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

#include "munkres.hpp"

namespace alan_pose_estimation
{

    class LedNodelet : public nodelet::Nodelet
    {
        public:
        private:
            //general objects
            cv::Mat frame;
            Eigen::MatrixXd cameraMat = Eigen::MatrixXd::Zero(3,3);
            vector<correspondence::matchid> LED_v_Detected;
            vector<Eigen::Vector3d> pts_on_body_frame, pts_on_body_frame_normalized;
            vector<Eigen::Vector3d> pts_detected_in_corres_order;
            

            //temp objects
            double temp = 0;
            int i = 0;

            //subscribe            
            void camera_callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth);
            
            message_filters::Subscriber<sensor_msgs::CompressedImage> subimage;
            message_filters::Subscriber<sensor_msgs::Image> subdepth;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
            typedef message_filters::Synchronizer<MySyncPolicy> sync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<sync> sync_;                    
            
            //solve pose & tools
            void solve_pose_w_LED(cv::Mat& frame, cv::Mat depth); 

            void solveicp_svd(vector<Eigen::Vector3d> pts_3d, vector<Eigen::Vector3d> body_frame_pts, Eigen::Matrix3d& R, Eigen::Vector3d& t);
            
            Eigen::Vector3d get_CoM(vector<Eigen::Vector3d> pts_3d);

            void use_pnp_instead(cv::Mat frame, vector<Eigen::Vector2d> pts_2d_detect, vector<Eigen::Vector3d> pts_3d_detect, Sophus::SE3d& pose);

            Eigen::Vector2d reproject_3D_2D(Eigen::Vector3d P, Sophus::SE3d pose);
            
            //pnp + BA
            void get_initial_pose(vector<Eigen::Vector2d> pts_2d, vector<Eigen::Vector3d> body_frame_pts, Eigen::Matrix3d& R, Eigen::Vector3d& t);

            void optimize(Sophus::SE3d& pose, vector<Eigen::Vector3d> pts_3d_exists, vector<Eigen::Vector2d> pts_2d_detected);//converge problem need to be solved //-> fuck you, your Jacobian was wrong

            void solveJacobian(Eigen::Matrix<double, 2, 6>& Jacob, Sophus::SE3d pose, Eigen::Vector3d point_3d);


            //LED extraction tool
            vector<Eigen::Vector2d> LED_extract_POI(cv::Mat& frame, cv::Mat depth);

            vector<Eigen::Vector3d> pointcloud_generate(vector<Eigen::Vector2d> pts_2d_detected, cv::Mat depthimage);
            
            double LANDING_DISTANCE = 0;
            int BINARY_THRES = 0;

            //initiation & correspondence        
            void correspondence_search(vector<Eigen::Vector3d> pts_on_body_frame, vector<Eigen::Vector3d> pts_detected);    

            vector<Eigen::Vector3d> normalization_2d(vector<Eigen::Vector3d> v_pts, int i_x, int i_y);            

            vector<Eigen::Vector3d> sort_the_points_in_corres_order(vector<Eigen::Vector3d> pts, vector<correspondence::matchid> corres);
            
            correspondence::munkres hungarian; 
            bool LED_tracking_initialize(cv::Mat& frame, cv::Mat depth, Sophus::SE3d& pose, vector<correspondence::matchid>& corres);

            bool LED_tracker_initiated = false;
            int LED_no;

            //main process
            void recursive_filtering(cv::Mat& frame, cv::Mat depth, Sophus::SE3d& pose, vector<correspondence::matchid>& corres);

            //track
            void tracking(vector<Eigen::Vector3d> pts_3d_pcl_detect, vector<Eigen::Vector3d> pts_on_body_frame);

            vector<Eigen::Vector3d> filter_out_nondetected_body_points(vector<Eigen::Vector3d> pts_3d_pcl_detect, correspondence::matchid tracking_result);

            //outlier rejection
            void reject_outlier(vector<Eigen::Vector3d>& pts_3d_detect, vector<Eigen::Vector2d>& pts_2d_detect);

            double calculate_MAD(vector<double> norm_of_points);

            cv::Point3f point_wo_outlier_previous;
            double MAD_x_threshold = 0, MAD_y_threshold = 0, MAD_z_threshold = 0;

            //publish
            void map_SE3_to_pose(Sophus::SE3d pose);

            geometry_msgs::PoseStamped uav_pose_estimated;
            


            virtual void onInit()
            {
                ros::NodeHandle& nh = getNodeHandle();
                
                //load POT_extract config
                nh.getParam("/alan_pose/LANDING_DISTANCE", LANDING_DISTANCE);     
                nh.getParam("/alan_pose/BINARY_threshold", BINARY_THRES);     

                
                //load camera intrinsics
                Eigen::Vector4d intrinsics_value;
                XmlRpc::XmlRpcValue intrinsics_list;
                nh.getParam("/alan_pose/cam_intrinsics_455", intrinsics_list);                
                                
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
                nh.getParam("/alan_pose/LED_positions", LED_list); 
                for(int i = 0; i < LED_list.size(); i++)
                {
                    Eigen::Vector3d temp(LED_list[i]["x"], LED_list[i]["y"], LED_list[i]["z"]);
                    pts_on_body_frame.push_back(temp);
                }   
                LED_no = pts_on_body_frame.size();

                //load outlier rejection info
                nh.getParam("/alan_pose/MAD_x_threshold", MAD_x_threshold);
                nh.getParam("/alan_pose/MAD_y_threshold", MAD_y_threshold);
                nh.getParam("/alan_pose/MAD_z_threshold", MAD_z_threshold);

                LED_no = pts_on_body_frame.size();
                
                // pts_on_body_frame_normalized = normalization_2d(pts_on_body_frame, 1, 2);
                                                
                // //subscribe
                subimage.subscribe(nh, "/camera/color/image_raw/compressed", 1);
                subdepth.subscribe(nh, "/camera/aligned_depth_to_color/image_raw", 1);                
                sync_.reset(new sync( MySyncPolicy(10), subimage, subdepth));            
                sync_->registerCallback(boost::bind(&LedNodelet::camera_callback, this, _1, _2));

                // // x = [-0.36926538, -0.35783651, -0.30663395, -0.37761885, -0.28259838, -0.32332534]
                // // y = [-0.17193949, -0.17355335,  -0.17994796,  -0.1793365, -0.19169508,  -0.20557153]
                // // z = [0.71600002, 0.71799999, 0.72549999, 0.68800002,0.70550001, 0.727]
            
                // //test initialization
                // vector<Eigen::Vector2d> pts_2d_detect;
                // vector<Eigen::Vector3d> pts_3d_detect;

                // XmlRpc::XmlRpcValue pts_2d_list, pts_3d_list;
                
                // nh.getParam("/alan_pose/pts_2d_list", pts_2d_list); 
                // nh.getParam("/alan_pose/pts_3d_list", pts_3d_list); 
                // cout<<"hi"<<endl;

                // cout<<pts_2d_list.size()<<endl;
                // cout<<pts_3d_list.size()<<endl;


                // for(int i = 0; i < pts_2d_list.size(); i++)
                // {
                //     Eigen::Vector2d temp1(pts_2d_list[i]["x"], pts_2d_list[i]["y"]);                
                //     Eigen::Vector3d temp2(pts_3d_list[i]["x"], pts_3d_list[i]["y"], pts_3d_list[i]["z"]);

                //     pts_2d_detect.push_back(temp1);
                //     pts_3d_detect.push_back(temp2);

                // }   

                // //now I have pts_2d_detect pts_3d_detect and pts_on_body_frame;        
                

                // double t1 = ros::Time::now().toSec();
                // int i = 0;
                
                // vector<int> test;
                
                // for(int i = 0; i < 6; i++)
                // {
                //     test.push_back(i);
                // }

                // double error_total = INFINITY;
                
                // Eigen::Matrix3d R;
                // Eigen::Vector3d t;
                // vector<int> final_permutation;

                // do 
                // {       
                //     vector<Eigen::Vector2d> pts_2d_detect_temp;       
                //     vector<Eigen::Vector3d> pts_3d_detect_temp;     

                //     for(auto what : test)
                //     {
                //         pts_2d_detect_temp.push_back(pts_2d_detect[what]);
                //         pts_3d_detect_temp.push_back(pts_3d_detect[what]);
                //     }
                                                            
                //     get_initial_pose(pts_2d_detect_temp, pts_on_body_frame, R, t);
                //     // solveicp_svd(pts_3d_detect_temp, pts_on_body_frame, R, t);
                    
                //     Sophus::SE3d pose(R, t);

                //     Eigen::Vector2d reproject, error; 
                //     double e = 0;

                //     for(int i = 0 ; i < pts_on_body_frame.size(); i++)
                //     {
                //         reproject = reproject_3D_2D(pts_on_body_frame[i], pose);  
                //         error = pts_2d_detect_temp[i] - reproject;
                //         e = e + error.norm();
                //     }

                //     if(e < error_total)
                //     {                    
                //         error_total = e;
                //         final_permutation = test;
                //         if(error_total < 5)
                //             break;
                //     }
                    
                // }
                // while(next_permutation(test.begin(), test.end()));

                // // vector<Eigen::Vector3d> pts_3d_detect_temp;
                // // pts_3d_detect_temp.push_back(pts_3d_detect[4]);
                // // pts_3d_detect_temp.push_back(pts_3d_detect[2]);
                // // pts_3d_detect_temp.push_back(pts_3d_detect[1]);
                // // pts_3d_detect_temp.push_back(pts_3d_detect[5]);
                // // pts_3d_detect_temp.push_back(pts_3d_detect[0]);
                // // pts_3d_detect_temp.push_back(pts_3d_detect[3]);

                // // vector<Eigen::Vector2d> pts_2d_detect_temp;
                // // pts_2d_detect_temp.push_back(pts_2d_detect[0]);
                // // pts_2d_detect_temp.push_back(pts_2d_detect[1]);
                // // pts_2d_detect_temp.push_back(pts_2d_detect[2]);
                // // pts_2d_detect_temp.push_back(pts_2d_detect[3]);
                // // pts_2d_detect_temp.push_back(pts_2d_detect[4]);
                // // pts_2d_detect_temp.push_back(pts_2d_detect[5]);

                // // solveicp_svd(pts_3d_detect_temp, pts_on_body_frame, R, t);

                // // Sophus::SE3d pose(R, t);

                // // Eigen::Vector2d reproject, error; 
                // // double e = 0;

                // // for(int i = 0 ; i < pts_on_body_frame.size(); i++)
                // // {
                // //     reproject = reproject_3D_2D(pts_on_body_frame[i], pose);  
                // //     error = pts_2d_detect[final_permutation[i]] - reproject;
                // //     e = e + error.norm();
                // // }
                // // cout<<e<<endl;

                // cout<<"final: "<<error_total<<endl;
                // for(auto what : final_permutation)
                //     cout<<what;
                // cout<<endl;
                // double t2 = ros::Time::now().toSec();

                // cout<<"Hz: "<< 1 / (t2-t1) <<endl;






            }

            public:
                static void* PubMainLoop(void* tmp);

    };

    PLUGINLIB_EXPORT_CLASS(alan_pose_estimation::LedNodelet, nodelet::Nodelet)
}