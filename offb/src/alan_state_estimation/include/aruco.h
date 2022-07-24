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

#include "alan/obj.h"

#include <pthread.h>

namespace alan_pose_estimation
{
    class ArucoNodelet : public nodelet::Nodelet
    {
        private:
            pthread_t tid;

            //publisher
            ros::Publisher nodelet_pub;
            ros::Publisher pubpose;
            image_transport::Publisher pubimage;
            

            //subscriber
            message_filters::Subscriber<sensor_msgs::CompressedImage> subimage;
            message_filters::Subscriber<sensor_msgs::Image> subdepth;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
            typedef message_filters::Synchronizer<MySyncPolicy> sync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<sync> sync_;

            //private variables
            cv::Mat frame;
            vector<Eigen::Vector3d> body_frame_pts;
            Eigen::MatrixXd cameraMat = Eigen::MatrixXd::Zero(3,3);
            std_msgs::Bool test;
            geometry_msgs::PoseStamped pose_estimated;
            bool add_noise = false;       
            int temp_i = 0;     


            //functions
            void camera_callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth);

            //pnp + BA
            void pose_w_aruco_pnp(cv::Mat& frame);

            Eigen::Vector2d reproject_3D_2D(Eigen::Vector3d P, Sophus::SE3d pose);

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


            virtual void onInit() 
            {
                ros::NodeHandle& nh = getNodeHandle();

                //load camera intrinsics
                Eigen::Vector4d intrinsics_value;
                XmlRpc::XmlRpcValue intrinsics_list;
                nh.getParam("/aruco/cam_intrinsics_455", intrinsics_list);                
                                
                for(int i = 0; i < 4; i++)
                {
                    intrinsics_value[i] = intrinsics_list[i];
                }

                cameraMat <<    
                    intrinsics_value[0], 0, intrinsics_value[2], 
                    0, intrinsics_value[1], intrinsics_value[3],
                    0, 0,  1;    

                // cout<<cameraMat.inverse()<<endl;


                //load LED potisions in body frame
                XmlRpc::XmlRpcValue LED_list;
                nh.getParam("/alan_pose/LED_positions", LED_list); 
                for(int i = 0; i < LED_list.size(); i++)
                {
                    Eigen::Vector3d temp(LED_list[i]["x"], LED_list[i]["y"], LED_list[i]["z"]);
                    body_frame_pts.push_back(temp);
                }
                
                //initialize publisher
                nodelet_pub = nh.advertise<std_msgs::Bool>("/obj_found",1);
                pubpose = nh.advertise<geometry_msgs::PoseStamped>("/alan_pose/pose", 1);
                // test_pub = nh.advertise<std_msgs::Bool>("/ob_found",1);

                pthread_create(&tid, NULL, ArucoNodelet::PubMainLoop, (void*)this);

                image_transport::ImageTransport image_transport_(nh);
                pubimage = image_transport_.advertise("/processed_image",1);

                //initialize subscribe
                // subimage = nh.subscribe("/camera/color/image_raw/compressed", 1, &ArucoNodelet::camera_callback, this);
                subimage.subscribe(nh, "/camera/color/image_raw/compressed", 1);
                subdepth.subscribe(nh, "/camera/aligned_depth_to_color/image_raw", 1);                
                sync_.reset(new sync( MySyncPolicy(10), subimage, subdepth));            
                sync_->registerCallback(boost::bind(&ArucoNodelet::camera_callback, this, _1, _2));

                ROS_INFO("Aruco Nodelet Initiated...");
            }     

            public:
                static void* PubMainLoop(void* tmp);   

    };
    
    PLUGINLIB_EXPORT_CLASS(alan_pose_estimation::ArucoNodelet, nodelet::Nodelet)
}