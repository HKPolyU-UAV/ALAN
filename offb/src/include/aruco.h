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

namespace aruco_ros_nodelet
{
    class ArucoNodelet : public nodelet::Nodelet
    {
        private:
            pthread_t tid;
            ros::Publisher nodelet_pub ;
            std_msgs::Bool test;


            message_filters::Subscriber<sensor_msgs::CompressedImage> subimage;
            message_filters::Subscriber<sensor_msgs::Image> subdepth;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
            typedef message_filters::Synchronizer<MySyncPolicy> sync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<sync> sync_;

            //private variables
            cv::Mat frame;
            vector<Eigen::Vector3d> body_frame_pts;
            Eigen::MatrixXd cameraMat = Eigen::MatrixXd::Zero(3,3);

            //functions
            void camera_callback(
                const sensor_msgs::CompressedImageConstPtr & rgbimage,
                const sensor_msgs::ImageConstPtr & depth
            );

            Eigen::Vector2d reproject_3D_2D(Eigen::Vector3d P, Sophus::SE3d pose);

            void solvepnp(vector<Eigen::Vector2d> pts_2d, vector<Eigen::Vector3d> body_frame_pts, Eigen::Matrix3d& R, Eigen::Vector3d& t);

            bool aruco_detect(cv::Mat& frame, vector<Eigen::Vector2d>& pts_2d);

            void solveJacobian(Eigen::Matrix<double, 2, 6>& Jacob, Sophus::SE3d pose, Eigen::Vector3d point_3d);

            void optimize(Sophus::SE3d& pose, vector<Eigen::Vector3d> pts_3d_exists, vector<Eigen::Vector2d> pts_2d_detected);//converge problem need to be solved //-> fuck you, your Jacobian was wrong

            static void* PubMainLoop(void* tmp);


            virtual void onInit() 
            {
                ros::NodeHandle& nh = getNodeHandle();
                nodelet_pub = nh.advertise<std_msgs::Bool>("/obj_found",1);

                pthread_create(&tid, NULL, ArucoNodelet::PubMainLoop, (void*)this);


                // PubMainLoop();

                subimage.subscribe(nh, "/camera/color/image_raw/compressed", 1);
                subdepth.subscribe(nh, "/camera/aligned_depth_to_color/image_raw", 1);
                
                sync_.reset(new sync( MySyncPolicy(10), subimage, subdepth));            
                sync_->registerCallback(boost::bind(&ArucoNodelet::camera_callback, this, _1, _2));

                ROS_INFO("Aruco Nodelet Initiated...");

                body_frame_pts.push_back(Eigen::Vector3d(0.055, -0.0225, -0.010)); //LU
                body_frame_pts.push_back(Eigen::Vector3d(0.055,  0.0225, -0.010)); //RU
                body_frame_pts.push_back(Eigen::Vector3d(0.055,  0.0225, -0.055)); //RD
                body_frame_pts.push_back(Eigen::Vector3d(0.055,  -0.0225, -0.055)); //LD
                
                double fx = 634.023193359375,
                        fy = 633.559814453125,
                        cx = 641.8981323242188,
                        cy = 387.1009521484375;

                cameraMat << 
                    fx, 0, cx, 
                    0, fy, cy,
                    0, 0,  1;

            }
        

    };
    
    PLUGINLIB_EXPORT_CLASS(aruco_ros_nodelet::ArucoNodelet, nodelet::Nodelet)
}