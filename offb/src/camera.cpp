#include "include/run_yolo.hpp"

#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/PointStamped.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include "offb/obj.h"
#include "include/KalmanFilter.hpp"

static cv::Mat frame, res, gt;

static cv::String weightpath ="/home/patty/alan_ws/src/alan/offb/src/include/yolo/uav_new.weights";
static cv::String cfgpath ="/home/patty/alan_ws/src/alan/offb/src/include/yolo/uav_new.cfg";
static cv::String classnamepath = "/home/patty/alan_ws/src/alan/offb/src/include/yolo/uav.names";

static run_yolo Yolonet(cfgpath, weightpath, classnamepath, float(0.1));

static int counter = 0;



void callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth)
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
    cv::Mat image_dep = depth_ptr->image;

    Yolonet.getdepthdata(image_dep);

    try
    {
        frame = cv::imdecode(cv::Mat(rgbimage->data),1);
        res   = cv::imdecode(cv::Mat(rgbimage->data),1);
        gt    = cv::imdecode(cv::Mat(rgbimage->data),1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cout<<frame.size<<endl;
}


int main(int argc, char** argv)
{

    cout<<"Object detection..."<<endl;

    ros::init(argc, argv, "yolotiny");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::CompressedImage> subimage(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<sensor_msgs::Image> subdepth(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subimage, subdepth);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Publisher publish_obj_w = nh.advertise<geometry_msgs::Point>("/pose_w",1);
    ros::Publisher publish_obj_c = nh.advertise<geometry_msgs::PointStamped>("/pose_camera",1);
    ros::Publisher publish_found = nh.advertise<std_msgs::Bool>("/obj_found",1);
    ros::Publisher publish_obj_vel = nh.advertise<offb::obj>("/obj_v", 1);



    while(ros::ok())
    {
        if(!frame.empty())
        {
            Yolonet.rundarknet(frame);
            Yolonet.display(frame);

        }
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}