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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

namespace alan_pose_estimation
{
    typedef struct Match
    {
        int id;
        bool toofar;
    }Match;

    class LedNodelet : public nodelet::Nodelet
    {
        private:

            cv::Mat frame;
            double LANDING_DISTANCE = 0;
            Eigen::MatrixXd cameraMat = Eigen::MatrixXd::Zero(3,3);

            message_filters::Subscriber<sensor_msgs::CompressedImage> subimage;
            message_filters::Subscriber<sensor_msgs::Image> subdepth;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
            typedef message_filters::Synchronizer<MySyncPolicy> sync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<sync> sync_;
            
            vector<int> id_tracked;

            void camera_callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth);
            
            //solve pose
            void pose_w_LED_pnp(cv::Mat& frame, cv::Mat depth);            

            //LED extraction
            vector<Eigen::Vector2d> LED_extract_POI(cv::Mat& frame, cv::Mat depth);

            void LED_tracking();

            void LED_tracking_initialize(cv::Mat& frame, cv::Mat depth);

            Eigen::Matrix4d icp_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i);

            void solveicp_for_initialize(vector<Eigen::Vector3d> pts_3d, vector<Eigen::Vector3d> body_frame_pts, Eigen::Matrix3d& R, Eigen::Vector3d& t);
                        
            void solveicp_svd(vector<Eigen::Vector3d> pts_3d, vector<Eigen::Vector3d> body_frame_pts, Eigen::Matrix3d& R, Eigen::Vector3d& t);
            
            Eigen::Vector3d get_CoM(vector<Eigen::Vector3d> pts_3d);

            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_generate(vector<Eigen::Vector2d> pts_2d_detected, cv::Mat depthimage);
            
            //Munkres
            void cost_generate(vector<cv::Point> detected, vector<cv::Point> previous);
            vector<Match> solution(vector<cv::Point> measured, vector<cv::Point> previous);//return the corresponding ids
            vector<Match> id_match;

            void stp1(int &step);//reduce with the minima of row and column
            void stp2(int &step);
            void stp3(int &step);
            void stp4(int &step);
            void stp5(int &step);
            void stp6(int &step);
            void stp7();

            void find_a_zero(int& row, int& col);
            bool star_in_row(int row);
            void find_star_in_row(int row, int& col);
            void find_min(double& minval);
            void find_star_in_col(int col, int& row);
            void find_prime_in_row(int row, int& col);
            void augment_path();
            void clear_covers();
            void erase_primes();

            int step = 1;
            Eigen::MatrixXd cost, mask, path, copy;
            vector<int> cover_row;
            vector<int> cover_col;
            int path_row_0, path_col_0, path_count;


            virtual void onInit()
            {
                ros::NodeHandle& nh = getNodeHandle();

                nh.getParam("/alan_pose/LANDING_DISTANCE", LANDING_DISTANCE);     
                
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
        
                // //subscribe
                subimage.subscribe(nh, "/camera/color/image_raw/compressed", 1);
                subdepth.subscribe(nh, "/camera/aligned_depth_to_color/image_raw", 1);                
                sync_.reset(new sync( MySyncPolicy(10), subimage, subdepth));            
                sync_->registerCallback(boost::bind(&LedNodelet::camera_callback, this, _1, _2));


                // vector<Eigen::Vector3d> pts_3d_body;
                // pts_3d_body = {
                //     Eigen::Vector3d(0.055, -0.0225, -0.01),
                //     Eigen::Vector3d(0.055, 0.0225, -0.01),
                //     Eigen::Vector3d(0.055, 0.0225, -0.055),
                //     Eigen::Vector3d(0.055, -0.0225, -0.055)
                //                 };

                // vector<Eigen::Vector3d> pts_3d_camera;
                // pts_3d_camera = {
                //     Eigen::Vector3d(-0.141276, 0.198773, 0.512),
                //     Eigen::Vector3d(-0.142378, 0.157403, 0.525),
                //     Eigen::Vector3d(-0.100869, 0.155597, 0.5245),
                //     Eigen::Vector3d(-0.0981761, 0.195773, 0.5105),
                // };

                // Eigen::Matrix3d R;
                // Eigen::Vector3d t;
                // solveicp_svd(pts_3d_camera, pts_3d_body, R, t);

                // // cout<<R<<endl;
                // // cout<<t<<endl;
                
                
                // pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);

                // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(4,1));
                // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>(4,1));

                // double t1 = ros::Time::now().toSec();

                // // Fill in the CloudIn data
                // for (int i = 0; i < 4; i++)
                // {
                //     cloud_in->points[i].x = pts_3d_body[i].x();
                //     cloud_in->points[i].y = pts_3d_body[i].y();
                //     cloud_in->points[i].z = pts_3d_body[i].z();
                // }

                // vector<Eigen::Vector3d> pts_3d_body_transformed;

                // Eigen::Matrix3d RR;
                // RR <<  
                // -0.0401792, -0.0452793,  -0.998166,
                // -0.312381,  -0.948329,  0.0555929,
                // -0.949107,   0.314042,  0.0239587;

                // Eigen::Vector3d tt;
                // tt <<
                // -0.150905, 
                // 0.195874,
                // 0.57098;


                // for(int i = 0; i < 4; i++)
                // {
                //     Eigen::Vector3d temp;
                //     temp = RR * pts_3d_body[i] + tt;
                //     pts_3d_body_transformed.push_back(temp);
                // }

                // for (int i = 0; i < 4; i++)
                // {
                //     cloud_out->points[i].x = pts_3d_body_transformed[i].x();
                //     cloud_out->points[i].y = pts_3d_body_transformed[i].y();
                //     cloud_out->points[i].z = pts_3d_body_transformed[i].z();
                // }

                // std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;
                    
                // for (auto& point : *cloud_in)
                //     std::cout << point << std::endl;
                    
                // cout<<endl;


                // std::cout << "Saved " << cloud_out->size () << " data points to output:" << std::endl;
                    
                // for (auto& point : *cloud_out)
                //     std::cout << point << std::endl;

                // //////////////////////////////////////////////////////////////////////////////////////

                // pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>::Ptr rej_ransac(new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>);

                // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                // icp.setMaximumIterations(400);
                // icp.setInputSource(cloud_in);
                // icp.setInputTarget(cloud_out);
                // // icp.addCorrespondenceRejector(rej_ransac);

                // pcl::PointCloud<pcl::PointXYZ> Final;
                // // icp.settr

                


                // icp.align(Final);

                

                // // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                // // icp.getFitnessScore() << std::endl;

                // cout<<"below show results"<<endl;
                // for(auto what : Final)
                // {                    
                //     cout<<what.x<<", "<<what.y<<", "<<what.z<<", ";
                //     cout<<endl;
                // }
                // cout<<"above show results"<<endl;

                // std::cout << icp.getFinalTransformation() << std::endl;  

                // cout<<"????????????????????????????????????????????????????"<<endl;
                // for(auto what : *icp.correspondence_estimation_->getIndices())
                // {
                //     cout<<"gan: ";
                //     cout<<what<<endl;
                // };
                // double t2 = ros::Time::now().toSec();


                // // for (auto& what : Final)
                // // {
                // //     cout<<what<<endl;;
                // // }
                // cout << endl << "Hz: " << 1 / (t2-t1) <<endl;
                
            }

    };

    PLUGINLIB_EXPORT_CLASS(alan_pose_estimation::LedNodelet, nodelet::Nodelet)
}