#include <sstream>
#include <cmath>
#include <Eigen/Dense>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/PointStamped.h"


#include "include/cnn.h"
#include <string>
#include "offb/obj.h"

#include "include/run_ncnn.hpp"

using namespace std;
static cv::Mat frame, res, gt;

static cv::String weightpath ="/home/patty/pat_ws/src/AUTO/offb/src/include/yolo/better.weights";
static cv::String cfgpath ="/home/patty/pat_ws/src/AUTO/offb/src/include/yolo/better.cfg";
static cv::String classnamepath = "/home/patty/pat_ws/src/AUTO/offb/src/include/yolo/better.names";

static run_yolo Yolonet(cfgpath, weightpath, classnamepath, float(0.1));

static int counter = 0;

ncnn::Net net;




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
    // cout<<frame.size<<endl;
}


int main(int argc, char** argv)
{

    ncnn::Net yolov4;

    const char* devicepath;

    int target_size = 416;

    run_ncnn lala;
    std::vector<Object> objects;


    lala.ncnn_init(&yolov4, &target_size);
    // cv::Mat nano = cv::imread("/home/patty/alan_ws/nano.png");
    // lala.detect_yolo(nano, objects, 0, &yolov4);

    // return 0;

    cout<<"Object detection..."<<endl;

    ros::init(argc, argv, "yolotiny");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::CompressedImage> subimage(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<sensor_msgs::Image> subdepth(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subimage, subdepth);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    while(ros::ok())
    {
        
        if(!frame.empty())
        {
            // frame = cv::imread("/home/patty/ncnn/build/examples/test.png");
            lala.detect_yolo(frame, objects, 0, &yolov4);
            
        }
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}

void testtrack()
{

}



