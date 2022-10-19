#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/PointCloud2.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

class SubAndPub
{
    ros::NodeHandle nh;
    ros::Subscriber subdep, subpcl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cv_mat_2_pcl(cv::Mat depth2b)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);//(new pcl::pointcloud<pcl::pointXYZ>);

        for(int i=0;i<depth2b.cols;i++)
        {
            pcl::PointXYZ point;

            //std::cout<<i<<endl;
            for(int j=0; j < depth2b.rows; j++)
            {
                point.x = i;
                point.y = j;
                // std::cout<<depth2b.at<float>(i,j)<<std::endl;
                point.z = depth2b.at<float>(j,i);

                point_cloud_ptr -> points.push_back(point);

            }

            // point.x = depth2b.at<float>(0,i);
            // point.y = depth2b.at<float>(1,i);
            // point.z = depth2b.at<float>(2,i);

            // when color needs to be added:
            //uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
            //point.rgb = *reinterpret_cast<float*>(&rgb);

        }
        point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
        point_cloud_ptr->height = 1;

        return point_cloud_ptr;

    }

    void depthimg_callback(const sensor_msgs::ImageConstPtr& depth)
    {
        cv_bridge::CvImageConstPtr depth_ptr;
        try 
        {
            depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
         
        cv::Mat temp = depth_ptr->image;  
        depthmat = temp;  

        

        cv::imshow("what", temp);
        cv::waitKey(20);
    }

    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& pcl)
    {
        this->output = *pcl;        
    }
    
public:
    
    cv::Mat depthmat;
    sensor_msgs::PointCloud2 output;
    SubAndPub()
    {
        subdep = nh.subscribe<sensor_msgs::Image> ("/camera/depth/image_rect_raw", 1, &SubAndPub::depthimg_callback, this);

    }
};
