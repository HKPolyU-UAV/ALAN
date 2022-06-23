// This ros node process 

#include <opencv2/aruco.hpp>
#include "include/essential.h"
#include "include/camera.h"

cv::Mat frame;
double fx = 911.2578125,
       fy = 911.5093994140625,
       cx = 639.192626953125,
       cy = 361.2229919433594;
cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);
cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);


void camera_callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth)
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

    try
    {
        frame = cv::imdecode(cv::Mat(rgbimage->data),1);
        // res   = cv::imdecode(cv::Mat(rgbimage->data),1);
        // gt    = cv::imdecode(cv::Mat(rgbimage->data),1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    // assert(src.type() == CV_8UC3);

    cv::cvtColor(frame, frame, cv::COLOR_RGB2HSV);

    cv::Mat lowred, onlyred;
    cv::inRange(frame, cv::Scalar(0,0,220), cv::Scalar(160,255,255), lowred);
    // cv::inRange(frame, cv::Scalar(0, 0, 100), cv::Scalar(160,0,0), onlyred);

    // cv::Mat higred = cv::inRange
    
    cv::imshow("before", lowred);
    cv::waitKey(20);

    // cv::inRange(frame, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 255), frame);



    // cv::imshow("after", frame);
    // cv::waitKey(20);
}

int main(int argc, char** argv)
{

    cout<<"Object detection..."<<endl;

    ros::init(argc, argv, "arucotest");
    ros::NodeHandle nh;
    cv::Mat markerImage;

    message_filters::Subscriber<sensor_msgs::CompressedImage> subimage(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<sensor_msgs::Image> subdepth(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subimage, subdepth);
    sync.registerCallback(boost::bind(&camera_callback, _1, _2));

    // cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);
    // cv::imwrite("marker23.png", markerImage);
    // cv::imshow("aruco", markerImage);
    // cv::imwrite("aruco.png", markerImage);
    // cv::waitKey(0);
    markerImage = cv::imread("testt.jpg");
    ros::Rate rate_manager(40);
    cameraMatrix.at<double>(0,0) = fx;
    cameraMatrix.at<double>(1,1) = fy;
    cameraMatrix.at<double>(0,2) = cx;
    cameraMatrix.at<double>(1,2) = cy;

    while(ros::ok())
    {
        ros::spinOnce();
        // rate_manager.sleep();
    }


  
    return 0;
}