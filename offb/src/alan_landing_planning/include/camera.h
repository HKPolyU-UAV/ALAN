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

#include "alan/obj.h"
// #include "KalmanFilter.hpp"
// #include "run_yolo.hpp"

typedef struct veh_pose
{
    double x;
    double y;
    double z;
    double ow;
    double ox;
    double oy;
    double oz;
} veh_pose;


// cv::Mat cam_Mat = cv::Mat::eye(3,3, CV_64F);
// cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
// cv::solvePnP(body_frame_pts,markercorners[0], cam_Mat, distCoeffs, rvec, tvec);
//         cv::Mat rmat = cv::Mat::eye(3,3, CV_64F);
//         cv::Rodrigues(rvec, rmat);
// void solvepnp(
//     vector<cv::Point2f> pts_2d, 
//     Eigen::Vector3d& t, 
//     Eigen::MatrixXd& R
//     )

// vector<int> markerids;
//     vector<vector<cv::Point2f>> markercorners, rejected;
//     cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
//     cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    
//     double t1 = ros::Time::now().toSec();
//     cv::aruco::detectMarkers(frame, dictionary, markercorners, markerids, parameters, rejected);

//     if(markercorners.size() != 0)
//     {
//         for(auto& what : markercorners)
//         for(auto& that : what)
//         {
//             // cout<<that<<endl;
//             // cout<<"draw"<<endl;
//             cv::circle(frame, that, 4, CV_RGB(255,0,0),-1);
//         }
//         cout<<"get pnp"<<endl;
    
//         double area = 0;
//         vector<cv::Point2f> aruco_candidate;
//         int which;
        
//         // cout<<rmat<<endl;
//         Eigen::Vector3d t;
//         Eigen::Matrix3d R;

//         solvepnp(markercorners[0], t, R);
//         cout<<t<<endl<<R<<endl;
//     }    




    // for(auto& what : markercorners)
    //     for(auto& that : what)
    //     {
    //         // cout<<that<<endl;
    //         // cout<<"draw"<<endl;
    //         cv::circle(frame, that, 4, CV_RGB(255,0,0),-1);
    //     }
    //     cout<<"get pnp"<<endl;
    
    //     double area = 0;
    //     vector<cv::Point2f> aruco_candidate;
    //     int which;
        
    //     // cout<<rmat<<endl;
    //     Eigen::Vector3d t;
    //     Eigen::Matrix3d R;

    //     solvepnp(markercorners[0], t, R);
    //     cout<<t<<endl<<R<<endl;
// Eigen::MatrixXd Trans(R.rows(), R.cols() + t.cols());
//         Trans <<
//             R , t;
        // Trans.block<3,3>(0,0) = R;
        // Trans.block<3,1>(0,3) = t;
        // cout<<Trans<<endl;

    //     double fx = 630.63818359375,
    //    fy = 630.1819458007812,
    //    cx = 638.9374389648438,
    //    cy = 372.27935791015625;

    // 739.869
// 739.115
// 739.822
// 739.121

// K: [909.4946899414062, 0.0, 642.2376098632812, 0.0, 907.6828002929688, 370.00665283203125, 0.0, 0.0, 1.0]
// 427.217041015625, 0.0, 428.84991455078125, 0.0, 427.217041015625, 236.16522216796875


