// #include "include/camera.h"
// #include <opencv2/core.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/aruco.hpp>
// #include <opencv2/calib3d.hpp>

// using namespace std;

// double fx = 911.2578125,
//        fy = 911.5093994140625,
//        cx = 639.192626953125,
//        cy = 361.2229919433594;

// veh_pose car_info, uav_info;

// void car_position_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
// {
//     car_info.x = pose->pose.position.x;
//     car_info.y = pose->pose.position.y;
//     car_info.z = pose->pose.position.z;
//     car_info.ow = pose->pose.orientation.w;
//     car_info.ox = pose->pose.orientation.x;
//     car_info.oy = pose->pose.orientation.y;
//     car_info.oz = pose->pose.orientation.z;
// }

// void c2w(Eigen::Vector4d& camera_pt)
// {
//     double depth = camera_pt(2);
//     double z = camera_pt(2);
//     double x = z * (camera_pt(0) - cx) / fx;
//     double y = z * (camera_pt(1) - cy) / fy;
//     Eigen::Vector4d cam(x,y,z,1);

//     Eigen::Matrix<double, 4, 4> c2b_transformation;
//     c2b_transformation << 
//         0.0342161,   -0.334618,  -0.941732, 0.567003,
//         0.999403,     0.0159477,  0.0306448, -0.018069,
//         0.00476414,  -0.942219,   0.334964, 0.0174849,
//         0, 0, 0, 1;
    
//     Eigen::Quaterniond q2r_matrix(car_info.ow, car_info.ox, car_info.oy, car_info.oz);

//     Eigen::Matrix<double, 4, 4> b2w_transformation;
    
//     b2w_transformation <<
//     q2r_matrix.toRotationMatrix()(0,0), q2r_matrix.toRotationMatrix()(0,1), q2r_matrix.toRotationMatrix()(0,2), car_info.x,
//     q2r_matrix.toRotationMatrix()(1,0), q2r_matrix.toRotationMatrix()(1,1), q2r_matrix.toRotationMatrix()(1,2), car_info.y,
//     q2r_matrix.toRotationMatrix()(2,0), q2r_matrix.toRotationMatrix()(2,1), q2r_matrix.toRotationMatrix()(2,2), car_info.z,
//     0, 0, 0, 1;

//     camera_pt = b2w_transformation * c2b_transformation * cam;
// }



// void uav_position_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
// {
//     uav_info.x = pose->pose.position.x;
//     uav_info.y = pose->pose.position.y;
//     uav_info.z = pose->pose.position.z;
//     uav_info.ow = pose->pose.orientation.w;
//     uav_info.ox = pose->pose.orientation.x;
//     uav_info.oy = pose->pose.orientation.y;
//     uav_info.oz = pose->pose.orientation.z;
//     ROS_INFO("uav_pose");
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "kf");
//     ros::NodeHandle nh;














//     cv::Mat test;//, test;
    
//     test = cv::imread("/home/patty/alan_ws/src/alan/offb/src/test/images/led6.png");
//     // cv::resize(test, test, cv::Size(), 2,2);
//     // cv::imshow("test", test);
//     // cv::imwrite("/home/patty/alan_ws/src/alan/offb/src/test/images/low_resol_enlarge.png", test);
    
//     // Read image

//     // Mat im = imread( "blob.jpg", IMREAD_GRAYSCALE );

//     // Set up the detector with default parameters.

//     // Detect blobs.

//     // Setup SimpleBlobDetector parameters.
// 	cv::SimpleBlobDetector::Params params;

// 	// Change thresholds
// 	params.minThreshold = 10;
//     params.thresholdStep = 10;
// 	params.maxThreshold = 255;

//     params.filterByColor = true;
//     params.blobColor = 200;
// 	// Filter by Area.
// 	params.filterByArea = false;
// 	// params.minArea = 1;

// 	// Filter by Circularity
// 	params.filterByCircularity = false;
// 	// params.minCircularity = 0.1;

// 	// Filter by Convexity
// 	params.filterByConvexity = false;
// 	params.minConvexity = 0.87;

// 	// Filter by Inertia
// 	params.filterByInertia = false;
// 	// params.minInertiaRatio = 0.01;

//     std::vector<cv::KeyPoint> keypoints;
    
//     // Set up detector with params
// 	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);   

// 	// Detect blobs
// 	detector->detect( test, keypoints);  

    
                    
//     // Draw detected blobs as red circles.

//     // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob

//     cv::Mat im_with_keypoints;

//     cout<<"size: "<<keypoints.size()<<endl;



//     cv::drawKeypoints( test, keypoints, im_with_keypoints, CV_RGB(255, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

//     for(auto what : keypoints)
//     {
//         cout<<what.pt<<endl;
//         cv::circle(im_with_keypoints, what.pt, 2, CV_RGB(255, 255, 0), -1);
//     }
    
//     // cv::resize(im_with_keypoints, im_with_keypoints, cv::Size(), 0.5, 0.5);


//     // Show blobs

//     cv::imshow("keypoints", im_with_keypoints );

//     cv::waitKey(0);


//     return 0;




























//     // ros::Subscriber sub_car_info = nh.subscribe<geometry_msgs::PoseStamped>
//     //                                 ("/vrpn_client_node/gh034_car/pose", 1, car_position_callback);
    
//     // ros::Subscriber sub_uav_info = nh.subscribe<geometry_msgs::PoseStamped>
//     //                                 ("/vrpn_client_node/gh034_nano2/pose", 1, uav_position_callback);

//     // Eigen::MatrixXd F, P, Q, H, R;
//     // KalmanFilter KF(F, P, Q, H, R, linear, 6, 3);



//     // // return 0;
//     // // cout<<F<<endl<<endl;
//     // // cout<<P<<endl<<endl;
//     // // cout<<Q<<endl<<endl;
//     // // cout<<H<<endl<<endl;
//     // // cout<<R<<endl<<endl;
//     // // return 0;

//     // Eigen::Vector4d z_state;
//     // cv::Rect z_rect_detect;
//     // double z_depth;
//     // // Yolonet.rundarknet(frame);
//     //         // Yolonet.display(frame);
//     //         // cout<<endl<<car_info.x<<endl<<endl;
//     //         // z_rect_detect = Yolonet.obj_vector[0].boundingbox;
//     //         // z_depth = Yolonet.obj_vector[0].depth;
//     //         // z_state = Eigen::Vector4d(z_rect_detect.x + z_rect_detect.width/2, 
//     //         //                           z_rect_detect.y + z_rect_detect.height/2,
//     //         //                           z_depth,
//     //         //                           1);

//     //         c2w(z_state);
//     //         // cout<<z_state<<endl;
//     //         // cout<<
//     //         // Eigen::Vector4d(
//     //         //     z_state(0) - uav_info.x,
//     //         //     z_state(1) - uav_info.y,
//     //         //     z_state(2) - uav_info.z,
//     //         //     1 - 1
//     //         // ).norm()
//     //         // <<endl;;

   
//     // ros::spin();
//     // return 0;
// }




#include <iostream>
#include "include/camera.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
using namespace cv;
using namespace std;
Mat src_gray;
int thresh = 100;
RNG rng(12345);
void thresh_callback(int, void* );
int main( int argc, char** argv )
{
    ros::init(argc, argv, "kf");
    ros::NodeHandle nh;
    CommandLineParser parser( argc, argv, "{@input | /home/patty/alan_ws/src/alan/offb/src/test/images/44.png | input image}" );
    Mat src = imread( samples::findFile( parser.get<String>( "@input" ) ) );
    if( src.empty() )
    {
      cout << "Could not open or find the image!\n" << endl;
      cout << "Usage: " << argv[0] << " <Input image>" << endl;
      return -1;
    }
    cvtColor( src, src_gray, COLOR_BGR2GRAY );
    blur( src_gray, src_gray, Size(3,3) );
    const char* source_window = "Source";
    namedWindow( source_window );
    imshow( source_window, src );
    const int max_thresh = 255;
    createTrackbar( "Canny thresh:", source_window, &thresh, max_thresh, thresh_callback );
    thresh_callback( 0, 0 );
    waitKey();
    return 0;
}
void thresh_callback(int, void* )
{
    Mat canny_output;
    Canny( src_gray, canny_output, thresh, thresh*2 );
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( drawing, contours, (int)i, color, 0.5, LINE_8, hierarchy, 0 );
    }
    imshow( "Contours", drawing );
    imwrite("/home/patty/alan_ws/src/alan/offb/src/test/images/drawing.png", drawing);
}
