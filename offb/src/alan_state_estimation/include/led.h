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
                // Eigen::Vector4d intrinsics_value;
                // XmlRpc::XmlRpcValue intrinsics_list;
                // nh.getParam("/aruco/cam_intrinsics_455", intrinsics_list);                
                                
                // for(int i = 0; i < 4; i++)
                // {
                //     intrinsics_value[i] = intrinsics_list[i];
                // }

                // cameraMat <<    
                //     intrinsics_value[0], 0, intrinsics_value[2], 
                //     0, intrinsics_value[1], intrinsics_value[3],
                //     0, 0,  1;         
        
                // //subscribe
                // subimage.subscribe(nh, "/camera/color/image_raw/compressed", 1);
                // subdepth.subscribe(nh, "/camera/aligned_depth_to_color/image_raw", 1);                
                // sync_.reset(new sync( MySyncPolicy(10), subimage, subdepth));            
                // sync_->registerCallback(boost::bind(&LedNodelet::camera_callback, this, _1, _2));




                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

                double t1 = ros::Time::now().toSec();

                // Fill in the CloudIn data
                for (auto& point : *cloud_in)
                {
                    point.x = 1024 * rand() / (RAND_MAX + 1.0f);
                    point.y = 1024 * rand() / (RAND_MAX + 1.0f);
                    point.z = 1024 * rand() / (RAND_MAX + 1.0f);
                }

                int temp_i = cloud_in->size() - 1;

                for (int i = temp_i; i >=0  ; i--)
                {
                    cout<<i<<endl;
                    cloud_out->push_back(cloud_in->points.at(i));
                    cout<<"lala"<<endl;
                    cout<<i<<endl<<endl;
                }                
                // *cloud_out = *cloud_in;

                // int i = 0;
                // i--;
                // cout<<i<<endl<<endl;;


                pcl::PointXYZ lala;
                lala.x = 0;
                lala.y = 0;
                lala.z = 0;

                cloud_in->emplace_back(lala);

                std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;
                    
                for (auto& point : *cloud_in)
                    std::cout << point << std::endl;
                    
                cout<<endl;


                //////////////////////////////////////////////////////////////////////////////////////


                for (auto& point : *cloud_out)
                    point.x += 10000;


                std::cout << "Saved " << cloud_out->size () << " data points to output:" << std::endl;
                    
                for (auto& point : *cloud_out)
                    std::cout << point << std::endl;

                //////////////////////////////////////////////////////////////////////////////////////

                pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>::Ptr rej_ransac(new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>);

                pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                icp.setInputSource(cloud_in);
                icp.setInputTarget(cloud_out);
                // icp.addCorrespondenceRejector(rej_ransac);

                pcl::PointCloud<pcl::PointXYZ> Final;
                // icp.settr

                


                icp.align(Final);

                

                // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                // icp.getFitnessScore() << std::endl;

                cout<<"below show results"<<endl;
                for(auto what : Final)
                {                    
                    cout<<what.x<<", "<<what.y<<", "<<what.z<<", ";
                    cout<<endl;
                }
                cout<<"above show results"<<endl;

                std::cout << icp.getFinalTransformation() << std::endl;  

                cout<<"hi"<<endl;

                pcl::PointCloud<pcl::PointXYZ>::Ptr results (new pcl::PointCloud<pcl::PointXYZ>);
                *results = Final;
                cout<<"hi"<<endl;

                pcl::CorrespondencesPtr corresps(new pcl::Correspondences);
                pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
                est.setInputSource (results);
                est.setInputTarget (cloud_out);
                est.determineCorrespondences (*corresps, 1.0);

                for (auto& what : *corresps)
                {
                    cout<<what.index_match<<endl;               
                }


                double t2 = ros::Time::now().toSec();
                pcl::IndicesPtr match_id = icp.getIndices();


                // for (auto& what : Final)
                // {
                //     cout<<what<<endl;;
                // }
                cout << endl << "Hz: " << 1 / (t2-t1) <<endl;
                
            }

    };

    PLUGINLIB_EXPORT_CLASS(alan_pose_estimation::LedNodelet, nodelet::Nodelet)
}