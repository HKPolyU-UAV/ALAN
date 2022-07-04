// This ros node process 

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include "include/essential.h"
#include "include/camera.h"
#include "sophus/se3.h"

cv::Mat frame;
double fx = 634.023193359375,
       fy = 633.559814453125,
       cx = 641.8981323242188,
       cy = 387.1009521484375;
Eigen::MatrixXd cameraMatrix = Eigen::MatrixXd::Zero(3,3);

vector<Eigen::Vector3d> body_frame_pts;

Eigen::Vector2d reproject_3D_2D(Eigen::Vector3d P, Eigen::MatrixXd R, Eigen::Vector3d t)
{
    Eigen::Vector3d result;
    // cout<<"debug:"<<endl;
    // cout<<T<<endl;
    // cout<<P<<endl;
    // cout<<cameraMatrix<<endl;
    
    result = cameraMatrix * (R * P + t); //dimention not right
    // cout<<result<<endl;
    Eigen::Vector2d result2d;
    

    result2d <<
        result(0)/result(2), 
        result(1)/result(2);
    
    // cout<<"3d:"<<endl<<result<<endl;

    // cout<<"2d:"<<endl<<result2d<<endl;
    
    return result2d;
}

void solvepnp(
    vector<Eigen::Vector2d> pts_2d, 
    Eigen::Vector3d& t, 
    Eigen::Matrix3d& R
    )
{
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    cv::Vec3d rvec, tvec;
    cv::Mat camMat = cv::Mat::eye(3,3,CV_64F);
    vector<cv::Point3d> pts_3d_;
    vector<cv::Point2d> pts_2d_;

    cv::Point3d temp3d;
    cv::Point2d temp2d;

    for(auto what : body_frame_pts)
    {
        temp3d.x = what(0);
        temp3d.y = what(1);
        temp3d.z = what(2);

        pts_3d_.push_back(temp3d);
    }

    for(auto what : pts_2d)
    {
        temp2d.x = what(0);
        temp2d.y = what(1);

        pts_2d_.push_back(temp2d);
    }


    camMat.at<double>(0,0) = cameraMatrix(0,0);
    camMat.at<double>(0,2) = cameraMatrix(0,2);
    camMat.at<double>(1,1) = cameraMatrix(1,1);
    camMat.at<double>(1,2) = cameraMatrix(1,2);
    // cout<<"pnp"<<endl<<camMat<<endl;

    cv::solvePnP(pts_3d_, pts_2d_ ,camMat, distCoeffs, rvec, tvec, false);
    
    //return values
    cv::Mat rmat = cv::Mat::eye(3,3,CV_64F);
    cv::Rodrigues(rvec, rmat);

    R <<
        rmat.at<double>(0,0), rmat.at<double>(0,1), rmat.at<double>(0,2),
        rmat.at<double>(1,0), rmat.at<double>(1,1), rmat.at<double>(1,2),
        rmat.at<double>(2,0), rmat.at<double>(2,1), rmat.at<double>(2,2);
    t <<
        tvec(0),
        tvec(1),
        tvec(2); 
}

bool aruco_detect(
    cv::Mat& frame,
    vector<Eigen::Vector2d>& pts_2d
    )
{
    vector<int> markerids;
    vector<vector<cv::Point2f>> markercorners, rejected;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(frame, dictionary, markercorners, markerids, parameters, rejected);

    if(markercorners.size() != 0)
    {
        
        for(auto& what : markercorners)
        {
            cout<<"what size: "<<what.size()<<endl;
            if(what.size() != 4)
                continue;
            for(auto& that : what)
            {
                cv::circle(frame, that, 4, CV_RGB(255,0,0),-1);
                Eigen::Vector2d result;
                result << 
                    that.x,
                    that.y;
                pts_2d.push_back(result);
            }
            if(markercorners.size() > 1)
                pts_2d.clear();
        }

        if(pts_2d.size() == 0)
            return false;
        else   
        {
            cout<<"size: "<<pts_2d.size()<<endl;
            return true;
        }                             
    }
    else
        return false;
}



void optimize(vector<Eigen::Vector2d>)
{
    //execute Gaussian-Newton Method
    cout<<"Bundle Adjustment Optimization"<<endl;


    
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
    // cout<<frame.size()<<endl;

    vector<Eigen::Vector2d> pts_2d;
    if(aruco_detect(frame, pts_2d))
    {
        cout<<"get pnp"<<endl;
        
        // cout<<rmat<<endl;
        Eigen::Vector3d t;
        Eigen::Matrix3d R;


        solvepnp(pts_2d, t, R);
        Sophus::SE3 pose(R,t);;
        cout<<"sophus here"<<endl<<pose<<"lala"<<endl;
        // cout<<t<<endl<<R<<endl;
        

        vector<Eigen::Vector2d> pts_2d_reproject;

        // cv::Point

        for(auto what : body_frame_pts)
        {
            Eigen::Vector2d reproject = reproject_3D_2D(what, R, t);
            
            // cout<<cv::Point(what(0), what(1))<<endl;
            cv::circle(frame, cv::Point(reproject(0), reproject(1)),4, CV_RGB(0,0,255),-1);
            pts_2d_reproject.push_back(reproject);
        }

        if(pts_2d_reproject.size() == pts_2d.size())
        {
            for(size_t i = 0; i < pts_2d.size(); i++)
            {
                double e2 = 
                    pow(pts_2d_reproject[i](0) - pts_2d[i][0],2) +
                    pow(pts_2d_reproject[i](1) - pts_2d[i][1],2);
                double e = sqrt(e2);
                cout<<e<<endl;
            }
            
            cout<<endl;
        }

    }    

    

    cv::imshow("aruco", frame);
    cv::waitKey(20);

}

int main(int argc, char** argv)
{

    cout<<"Object detection..."<<endl;

    ros::init(argc, argv, "arucotest");
    ros::NodeHandle nh;
    cv::Mat markerImage;

    body_frame_pts.push_back(Eigen::Vector3d(0.055, -0.0225, -0.010)); //LU
    body_frame_pts.push_back(Eigen::Vector3d(0.055,  0.0225, -0.010)); //RU
    body_frame_pts.push_back(Eigen::Vector3d(0.055,  0.0225, -0.055)); //RD
    body_frame_pts.push_back(Eigen::Vector3d(0.055,  -0.0225, -0.055)); //LD

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
    // markerImage = cv::imread("testt.jpg");
    ros::Rate rate_manager(40);

    cout<<"pass1"<<endl;

    cameraMatrix << 
        fx, 0, cx, 
        0, fx, cy,
        0, 0,  1;
    
    // cam_Mat.at<double>(0,0) = fx;
    // cam_Mat.at<double>(1,1) = fy;
    // cam_Mat.at<double>(0,2) = cx;
    // cam_Mat.at<double>(1,2) = cy;
    
    cout<<"pass2"<<endl;



    while(ros::ok())
    {
        ros::spinOnce();
        // rate_manager.sleep();
    }


  
    return 0;
}