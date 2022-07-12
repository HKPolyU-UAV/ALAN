#ifndef ARUCO_H
#define ARUCO_H
#include "include/aruco.h"

void alan_pose_estimation::ArucoNodelet::camera_callback(const sensor_msgs::CompressedImage::ConstPtr& rgbimage)
{
    try
    {
        this->frame = cv::imdecode(cv::Mat(rgbimage->data),1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    // std::cout<<frame.size()<<std::endl;

    double t1 = ros::Time::now().toSec();     


    vector<Eigen::Vector2d> pts_2d_detect;
    if(aruco_detect(frame, pts_2d_detect))
    {
        Eigen::Vector3d t;
        Eigen::Matrix3d R;

        solvepnp(pts_2d_detect, body_frame_pts, R, t);
        

        //generate noise
        double noise;
        
        
        std::default_random_engine generator;
        std::normal_distribution<double> dist(0, 0.32);

        generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
        noise = dist(generator);   
                
        // cout<<noise<<endl; 

        Eigen::AngleAxisd rollAngle(0.872 * noise, Eigen::Vector3d::UnitZ());

        // noise = - 1 + 2 * ((float)rand()) / RAND_MAX;
        Eigen::AngleAxisd yawAngle(0.872 * noise, Eigen::Vector3d::UnitY());

        // noise = - 1 + 2 * ((float)rand()) / RAND_MAX;
        Eigen::AngleAxisd pitchAngle(0.872 * noise, Eigen::Vector3d::UnitX());

        Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

        Eigen::Matrix3d rotationMatrix = q.matrix();
        R = R * rotationMatrix;

        // noise = - 1 + 2 * ((float)rand()) / RAND_MAX;
        // cout<<noise<<endl;
        Eigen::Vector3d error(0.1 * noise, 0.1 * noise, 0.1 * noise);
        t = t + error;

        Sophus::SE3d pose(R,t);


        for(auto what : body_frame_pts)
        {
            Eigen::Vector2d reproject = reproject_3D_2D(what, pose);            
            cv::circle(frame, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(255,0,0),-1);
        }

        if(body_frame_pts.size() == pts_2d_detect.size())
            optimize(pose, body_frame_pts, pts_2d_detect);//pose, body_frame_pts, pts_2d_detect

        for(auto what : body_frame_pts)
        {
            Eigen::Vector2d reproject = reproject_3D_2D(what, pose);            
            cv::circle(frame, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(0,255,0),-1);
        }

        map_SE3_to_pose(pose);

        // cout<<"ms: "<< t2 - t1 <<endl;
    }    

    // this->test.data = !this->test.data;
    double t2 = ros::Time::now().toSec();
    // cout<<"hz: "<<1 / (t2 - t1)<<endl<<endl;

    char hz[40];
    char fps[5] = " fps";
    sprintf(hz, "%.2f", 1 / (t2 - t1));
    strcat(hz, fps);
    cv::putText(frame, hz, cv::Point(20,40), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));
    cv::Mat imageoutput = frame.clone();
    cv_bridge::CvImage for_visual;
    for_visual.header = rgbimage->header;
    for_visual.encoding = sensor_msgs::image_encodings::BGR8;
    for_visual.image = imageoutput;

    this->pubimage.publish(for_visual.toImageMsg());



    // this->test.data = !this->test.data;
    // nodelet_pub.publish(this->test);

    // cv::imshow("aruco", this->frame);
    // cv::waitKey(20);

}

inline Eigen::Vector2d alan_pose_estimation::ArucoNodelet::reproject_3D_2D(Eigen::Vector3d P, Sophus::SE3d pose)
{
    Eigen::Vector3d result;

    Eigen::Matrix3d R = pose.rotationMatrix();
    Eigen::Vector3d t = pose.translation();

    result = this->cameraMat * (R * P + t); //dimenㄋion not right

    Eigen::Vector2d result2d;
    

    result2d <<
        result(0)/result(2), 
        result(1)/result(2);
    
    return result2d;
}

void alan_pose_estimation::ArucoNodelet::solvepnp(vector<Eigen::Vector2d> pts_2d, vector<Eigen::Vector3d> body_frame_pts, Eigen::Matrix3d& R, Eigen::Vector3d& t)
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

bool alan_pose_estimation::ArucoNodelet::aruco_detect(cv::Mat& frame, vector<Eigen::Vector2d>& pts_2d)
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

void alan_pose_estimation::ArucoNodelet::solveJacobian(Eigen::Matrix<double, 2, 6>& Jacob, Sophus::SE3d pose, Eigen::Vector3d point_3d)
{
    Eigen::Matrix3d R = pose.rotationMatrix();
    Eigen::Vector3d t = pose.translation();
                // cameraMat
    double fx = cameraMat(0,0);
    double fy = cameraMat(1,1);

    Eigen::Vector3d point_in_camera = R * point_3d + t;
    
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

void alan_pose_estimation::ArucoNodelet::optimize(Sophus::SE3d& pose, vector<Eigen::Vector3d> pts_3d_exists, vector<Eigen::Vector2d> pts_2d_detected)
//converge problem need to be solved //-> fuck you, your Jacobian was wrong
{
    //execute Gaussian-Newton Method
    // cout<<"Bundle Adjustment Optimization"<<endl;

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

        // cout<<"previous: "/*<<cout.precision(10)*/<< lastcost<<endl;
        // cout<<"currentt: "/*<<cout.precision(10)*/<< cost<<endl;

        // if(i > 0 && cost >= lastcost)
        //     break;

        pose = Sophus::SE3d::exp(dx) * pose;

        lastcost = cost;

        if(dx.norm() < converge_threshold)        
            break;
    }

    // cout<<"gone thru: "<<i<<" th, end optimize"<<endl;;

}

void alan_pose_estimation::ArucoNodelet::map_SE3_to_pose(Sophus::SE3d pose)
{
    pose_estimated.pose.position.x = pose.translation().x();
    pose_estimated.pose.position.y = pose.translation().y();
    pose_estimated.pose.position.z = pose.translation().z();

    Eigen::Quaterniond q = Eigen::Quaterniond(pose.rotationMatrix());
    pose_estimated.pose.orientation.w = q.w();
    pose_estimated.pose.orientation.x = q.x();
    pose_estimated.pose.orientation.y = q.y();
    pose_estimated.pose.orientation.z = q.z();

}

void* alan_pose_estimation::ArucoNodelet::PubMainLoop(void* tmp)
{
    ArucoNodelet* pub = (ArucoNodelet*) tmp;

    ros::Rate loop_rate(50);
    while (ros::ok()) 
    {
        // ROS_INFO("%d,publish!", num++);
        pub->pubpose.publish(pub->pose_estimated);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

#endif