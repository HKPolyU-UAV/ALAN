// This ros node process 

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include "include/essential.h"
#include <sophus/se3.hpp>
#include "include/camera.h"

cv::Mat frame;
vector<Eigen::Vector3d> body_frame_pts;
Eigen::MatrixXd cameraMat = Eigen::MatrixXd::Zero(3,3);

Eigen::Vector2d reproject_3D_2D(Eigen::Vector3d P, Sophus::SE3d pose)
{
    Eigen::Vector3d result;

    Eigen::MatrixXd R = pose.rotationMatrix();
    Eigen::Vector3d t = pose.translation();

    result = cameraMat * (R * P + t); //dimention not right

    Eigen::Vector2d result2d;
    

    result2d <<
        result(0)/result(2), 
        result(1)/result(2);
    
    return result2d;
}

void solvepnp(
    vector<Eigen::Vector2d> pts_2d,
    vector<Eigen::Vector3d> body_frame_pts,
    Eigen::Matrix3d& R,
    Eigen::Vector3d& t
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

    camMat.at<double>(0,0) = cameraMat(0,0);
    camMat.at<double>(0,2) = cameraMat(0,2);
    camMat.at<double>(1,1) = cameraMat(1,1);
    camMat.at<double>(1,2) = cameraMat(1,2);

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
            if(what.size() != 4)
                continue;

            for(auto& that : what)
            {
                cv::circle(frame, cv::Point(that.x, that.y), 4, CV_RGB(0,0,255),-1);
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
            return true;                                     
    }
    else
        return false;
}

void solveJacobian(
    Eigen::Matrix<double, 2, 6>& Jacob,
    Sophus::SE3d pose,
    Eigen::Vector3d point_3d)
{
    Eigen::Matrix3d R = pose.rotationMatrix();
    Eigen::Vector3d t = pose.translation();
                // cameraMat
    double fx = cameraMat(0,0);
    double fy = cameraMat(1,1);

    Eigen::Vector3d point_in_camera = R * point_3d + t;
    
    // cout<<"check jacob: "<<fx<<"\t"<<fy<<endl<<endl;;

    double x_c = point_in_camera(0),
           y_c = point_in_camera(1),
           z_c = point_in_camera(2);

    //save entries to Jacob and return
    Jacob << 
        //first row
        -fx / z_c, 
        0, 
        fx * x_c / z_c / z_c, 
        fx * x_c * y_c / z_c / z_c,
        -fx - fx * x_c * x_c / z_c / z_c,
        fx * y_c / z_c,

        //second row
        0,
        -fy / z_c,
        fy * y_c / z_c / z_c,
        fy + fy * y_c * y_c / z_c / z_c,
        -fy * x_c * y_c / z_c / z_c,
        -fy * x_c / z_c;
    
    // cout<<"Jacob here: "<<Jacob<<endl;
    
}

void optimize(
    Sophus::SE3d& pose,
    vector<Eigen::Vector3d> pts_3d_exists, 
    vector<Eigen::Vector2d> pts_2d_detected
    )//converge problem need to be solved //-> fuck you, your Jacobian was wrong
{
    //execute Gaussian-Newton Method
    cout<<"Bundle Adjustment Optimization"<<endl;

    const int MAX_ITERATION = 400;

    const double converge_threshold = 1e-12;

    const int points_no = pts_2d_detected.size();

    Eigen::Matrix<double, 2, 6> J;
    Eigen::Matrix<double, 6, 6> A; // R6*6
    Eigen::Matrix<double, 6, 1> b; // R6
    Eigen::Vector2d e; // R2
    Eigen::Matrix<double, 6, 1> dx;

    int i;
    double cost = 0, lastcost = INFINITY;
    
    for(i = 0; i < MAX_ITERATION; i++)
    {
        // cout<<"this is the: "<<i<<" th iteration."<<endl;
        A.setZero();
        b.setZero();

        cost = 0;
        

        for(int i=0; i < points_no; i++)
        {
            //get the Jacobian for this point
            solveJacobian(J, pose, pts_3d_exists[i]);

            e = pts_2d_detected[i] - reproject_3D_2D(pts_3d_exists[i], pose) ; 
            //jibai, forget to minus the detected points
            //you set your Jacobian according to "u-KTP/s", and you messed up the order, you fat fuck
    
            cost += e.squaredNorm();

            //form Ax = b
            A += J.transpose() * J;
            b += -J.transpose() * e;
        }
    
        //solve Adx = b
        dx = A.ldlt().solve(b);

        for(int i = 0; i < 6; i++)
            if(isnan(dx(i,0)))
                break;

        cout<<"previous: "/*<<cout.precision(10)*/<< lastcost<<endl;
        cout<<"currentt: "/*<<cout.precision(10)*/<< cost<<endl;

        if(i > 0 && cost >= lastcost)
            break;

        pose = Sophus::SE3d::exp(dx) * pose;

        lastcost = cost;

        if(dx.norm() < converge_threshold)        
            break;
    }

    cout<<"gone thru: "<<i<<" th, end optimize"<<endl;;

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
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    // cout<<frame.size()<<endl;

    vector<Eigen::Vector2d> pts_2d_detect;
    if(aruco_detect(frame, pts_2d_detect))
    {
        Eigen::Vector3d t;
        Eigen::Matrix3d R;

        solvepnp(pts_2d_detect, body_frame_pts, R, t);
        // Eigen::AngleAxisd row()
        Eigen::AngleAxisd rollAngle(0.174, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle(0.174, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(0.174, Eigen::Vector3d::UnitX());

        Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

        Eigen::Matrix3d rotationMatrix = q.matrix();
        R = R * rotationMatrix;

        Eigen::Vector3d error(0.1,0.1,0.1);
        t = t + error;

        Sophus::SE3d pose(R,t);
        
        for(auto what : body_frame_pts)
        {
            Eigen::Vector2d reproject = reproject_3D_2D(what, pose);            
            cv::circle(frame, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(255,0,0),-1);
        }

        double t1 = ros::Time::now().toSec();
        if(body_frame_pts.size() == pts_2d_detect.size())
            optimize(pose, body_frame_pts, pts_2d_detect);//pose, body_frame_pts, pts_2d_detect
        double t2 = ros::Time::now().toSec();

        for(auto what : body_frame_pts)
        {
            Eigen::Vector2d reproject = reproject_3D_2D(what, pose);            
            cv::circle(frame, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(0,255,0),-1);
        }

        // cout<<"ms: "<< t2 - t1 <<endl;
        cout<<"hz: "<<1 / (t2 - t1)<<endl<<endl;
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

    ros::Rate rate_manager(40);

    double fx = 634.023193359375,
       fy = 633.559814453125,
       cx = 641.8981323242188,
       cy = 387.1009521484375;

    cameraMat << 
        fx, 0, cx, 
        0, fy, cy,
        0, 0,  1;

    while(ros::ok())
    {
        ros::spinOnce();
        // rate_manager.sleep();
    }


  
    return 0;
}