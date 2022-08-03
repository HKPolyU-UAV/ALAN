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

// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/registration/icp.h>
// #include <pcl/registration/correspondence_estimation.h>
// #include <pcl/registration/correspondence_rejection_sample_consensus.h>
// #include <pcl/registration/transformation_estimation_svd.h>

#include "munkres.hpp"

namespace alan_pose_estimation
{
    typedef struct lala
    {
        int gan;
    }lala;

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

            //subscribe            
            void camera_callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth);
            
            message_filters::Subscriber<sensor_msgs::CompressedImage> subimage;
            message_filters::Subscriber<sensor_msgs::Image> subdepth;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
            typedef message_filters::Synchronizer<MySyncPolicy> sync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<sync> sync_;                    
            
            //solve pose & tools
            void pose_w_LED_icp(cv::Mat& frame, cv::Mat depth);            
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

            //initiation & correspondence
            bool LED_tracking_initialize(cv::Mat& frame, cv::Mat depth);
            bool LED_tracker_initiated = false;
            int LED_no;

            void correspondence_search(vector<Eigen::Vector3d> pts_on_body_frame, vector<Eigen::Vector3d> pts_detected);    
            vector<Eigen::Vector3d> normalization_2d(vector<Eigen::Vector3d> v_pts, int i_x, int i_y);            
            vector<Eigen::Vector3d> sort_the_points_in_corres_order(vector<Eigen::Vector3d> pts, vector<correspondence::matchid> corres);
            
            correspondence::munkres hungarian; 

            //track
            correspondence::matchid track(vector<Eigen::Vector3d> pts_3d_pcl_detect, vector<Eigen::Vector3d> pts_on_body_frame);
            vector<Eigen::Vector3d> filter_out_nondetected_body_points(vector<Eigen::Vector3d> pts_3d_pcl_detect, correspondence::matchid tracking_result);

            //outlier rejection
            void reject_outlier(vector<Eigen::Vector3d>& pts_3d_detect, vector<Eigen::Vector2d>& pts_2d_detect);
            double calculate_MAD(vector<double> norm_of_points);
            
            cv::Point3f point_wo_outlier_previous;
            double MAD_threshold = 0;


            virtual void onInit()
            {
                ros::NodeHandle& nh = getNodeHandle();
                
                //load landing config
                nh.getParam("/alan_pose/LANDING_DISTANCE", LANDING_DISTANCE);     
                
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
                nh.getParam("/alan_pose/MAD_threshold", MAD_threshold);

                LED_no = pts_on_body_frame.size();
                pts_on_body_frame_normalized = normalization_2d(pts_on_body_frame, 1, 2);
                                                
                //subscribe
                subimage.subscribe(nh, "/camera/color/image_raw/compressed", 1);
                subdepth.subscribe(nh, "/camera/aligned_depth_to_color/image_raw", 1);                
                sync_.reset(new sync( MySyncPolicy(10), subimage, subdepth));            
                sync_->registerCallback(boost::bind(&LedNodelet::camera_callback, this, _1, _2));

                // Eigen::Vector3d a(0, 1, 2), b(10, 8, 7), c(20, 21, 20), d(40, 38, 41);
                // Eigen::Vector3d a_(-1, 0.1, 2), b_(100, 100, 105), c_(19, 19, 20), d_(40, 37, 41);

                // vector<Eigen::Vector3d> first;
                // first.push_back(a);
                // first.push_back(b);
                // first.push_back(c);
                // first.push_back(d);

                // vector<Eigen::Vector3d> second;
                // second.push_back(c_);
                // second.push_back(d_);
                // second.push_back(a_);
                // second.push_back(b_);

                // correspondence::munkres lala;   
                
                // double t1 = ros::Time::now().toSec();          

                // // LED_v_Detected = lala.solution(first, second);
                // // cout<<LED_v_Detected.size()<<endl;
                
                // for(auto what : lala.solution(first, second))
                // {
                //     cout<<what.detected_indices<<endl;
                //     cout<<what.detected_ornot<<endl;
                //     cout<<"next!"<<endl;
                // }
                
                // ;

                // // for(auto what : normalization_2d(first, 1, 2))
                // // {
                // //     cout << what << endl << endl;
                // // }
                // cout<<endl<<"normalized....."<<endl<<endl;

                // vector<Eigen::Vector3d> test1 = normalization_2d(first, 1, 2), test2 = normalization_2d(second, 1, 2);
                
                // // LED_v_Detected = lala.solution(first, second);
                // // cout<<LED_v_Detected.size()<<endl;


                // for(auto what : lala.solution(test1, test2))
                // {
                //     cout<<what.detected_indices<<endl;
                //     cout<<what.detected_ornot<<endl;
                //     cout<<"next!"<<endl;
                // }


                // double t2 = ros::Time::now().toSec();
                // cout<<1/(t2-t1)<<" fps"<<endl;


            }

            public:
                static void* PubMainLoop(void* tmp);

    };

    PLUGINLIB_EXPORT_CLASS(alan_pose_estimation::LedNodelet, nodelet::Nodelet)
}