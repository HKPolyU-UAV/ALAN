// This ros node process 

#include <opencv2/aruco.hpp>
#include "include/essential.h"
#include "include/camera.h"

cv::Mat frame;
double fx = 911.2578125,
       fy = 911.5093994140625,
       cx = 639.192626953125,
       cy = 361.2229919433594;
Eigen::MatrixXd cameraMatrix;
cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

vector<cv::Point3d> body_frame_pts;

Eigen::Vector2d reproject_3D_2D(Eigen::Vector4d P, Eigen::MatrixXd T)
{
    Eigen::Vector4d result;
    result = cameraMatrix * T * P;
    Eigen::Vector2d result2d;
    result2d <<
        result(0)/result(2), result(1)/result(2);
    
    return result2d;
}

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

    vector<int> markerids;
    vector<vector<cv::Point2f>> markercorners, rejected;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    
    double t1 = ros::Time::now().toSec();
    cv::aruco::detectMarkers(frame, dictionary, markercorners, markerids, parameters, rejected);
    
    for(auto& what : markercorners)
        for(auto& that : what)
        {
            // cout<<that<<endl;
            cout<<"draw"<<endl;
            cv::circle(frame, that, 4, CV_RGB(255,0,0),-1);
        }
    
    
    
    double area = 0;
    vector<cv::Point2f> aruco_candidate;
    int which;

    // vector<cv::Vec3d> rvecs, tvecs;
    
    // if(markercorners.size() != 0)
    // {
    //     for(int i = 0; i < markercorners.size();i++)//    auto what : markercorners)
    //     {
    //         if(cv::Rect2f(markercorners[i][0], markercorners[i][2]).area() > area)
    //         {
    //             area = cv::Rect2f(markercorners[i][0], markercorners[i][2]).area();
    //             which = i;
    //         }            
    //     }

    //     // markercorners.clear();
    //     // markercorners.push_back(aruco_candidate);
    //     cv::aruco::drawDetectedMarkers(frame, markercorners, markerids);

    //     //pose estimation
    //     cv::aruco::estimatePoseSingleMarkers(
    //         markercorners, 
    //         0.045, 
    //         cameraMatrix,
    //         distCoeffs, 
    //         rvecs, 
    //         tvecs);

    //     cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 0.1);

    // }

    cv::imshow("aruco", frame);
    cv::waitKey(20);


}

int main(int argc, char** argv)
{

    cout<<"Object detection..."<<endl;

    ros::init(argc, argv, "arucotest");
    ros::NodeHandle nh;
    cv::Mat markerImage;

    body_frame_pts.push_back(cv::Point3d(0.055, -0.0225, -0.010)); //LU
    body_frame_pts.push_back(cv::Point3d(0.055,  0.0225, -0.010)); //RU
    body_frame_pts.push_back(cv::Point3d(0.055, -0.0225, -0.055)); //LD
    body_frame_pts.push_back(cv::Point3d(0.055,  0.0225, -0.055)); //RD


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

    cameraMatrix <<
        fx, 0, cx, 
        0, fx, cy,
        0, 0,  1;


    while(ros::ok())
    {
        ros::spinOnce();
        // rate_manager.sleep();
    }


  
    return 0;
}