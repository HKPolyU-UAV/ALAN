#include <sstream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>



#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::Time starto;

void Syncallback(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::PointCloud2::ConstPtr& pcl)
{
    cout<<"syn callback"<<endl;
}

void pcl_callback(const sensor_msgs::PointCloud2::ConstPtr& pcl)
{
    ROS_INFO("pcl");
    cout<<pcl->header.stamp.now().toSec() + starto.now().toSec()<<endl;
    
}

void image_callback(const sensor_msgs::Image::ConstPtr& image)
{
    ROS_INFO("image");
    cout<<image->header.stamp<<endl;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Synchronizer");
    ros::NodeHandle node;

    ROS_INFO("hi");


    // 建立需要订阅的消息对应的订阅器
    message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloudInfo_sub(node, "/livox/lidar", 1);
    message_filters::Subscriber<sensor_msgs::Image> ImageInfo_sub(node, "/camera/color/image_raw", 1);
    // message_filters::Subscriber<geometry_msgs::PoseStamped> vrpn(node, "/vrpn_client_node/livox/pose", 1);
    
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy; 
    
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ImageInfo_sub, PointCloudInfo_sub); //queue size=10
    sync.registerCallback(boost::bind(&Syncallback, _1, _2));
    
    ros::Subscriber image_sub = node.subscribe<sensor_msgs::Image>
            ("/camera/color/image_raw", 1, image_callback);

    ros::Subscriber pcl_sub = node.subscribe<sensor_msgs::PointCloud2>
            ("/cloud_registered", 1, pcl_callback);

    starto = ros::Time::now();


    // while(ros::ok())
    // {
    //     cout<<1<<endl;
    //     ros::spinOnce();
    // }

    // PointCloudInfo_pub = node.advertise<PointCloud2>("/djq_pc", 10);
    // ImageInfo_pub = node.advertise<Image>("/djq_image", 10);

    ros::spin();

    return 0;
}
