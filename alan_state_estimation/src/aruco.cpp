/*
    This file is part of ALan - the non-robocentric dynamic landing system for quadrotor

    ALan is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ALan is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ALan.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file aruco.cpp
 * \date 15/06/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for vision-based relative localization for UAV and UGV based on ARUCO markers
 */

#include "include/aruco.h"

void alan::ArucoNodelet::camera_callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth)
//void alan::ArucoNodelet::camera_callback(const sensor_msgs::CompressedImage::ConstPtr& rgbimage)
{
    total_no++;
    // std::cout<<rgbimage.size
    // std::cout<<depth->data.at<uchar>(0,0)<<std::endl;
    cv_bridge::CvImageConstPtr depth_ptr;
    aruco_pose_stamp = rgbimage->header.stamp;

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
    cv::Mat test;

    try
    {
        frame = cv::imdecode(cv::Mat(rgbimage->data), 1);
        test = frame.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    double t1 = ros::Time::now().toSec();  

    // pose_w_aruco_icp(frame, image_dep);
    pose_w_aruco_pnp(frame);//, image_dep);

    double t2 = ros::Time::now().toSec();
   
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


void alan::ArucoNodelet::ugv_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    // std::cout<<1<<std::endl;
    ugv_pose_msg = *pose;
    ugv_pose_msg.header.frame_id = "world";
    
    Eigen::Translation3d t_(
        ugv_pose_msg.pose.position.x,
        ugv_pose_msg.pose.position.y,
        ugv_pose_msg.pose.position.z
    );

    Eigen::Quaterniond q_(
        ugv_pose_msg.pose.orientation.w,
        ugv_pose_msg.pose.orientation.x,
        ugv_pose_msg.pose.orientation.y,
        ugv_pose_msg.pose.orientation.z
    );

    // ugv_pose = ugvcam_t_ * ugvcam_q_;    


    Eigen::Vector3d cam_origin;
    cam_origin.setZero();

    // q_cam =  q_ * q_c2b;
    // t_cam = q_.toRotationMatrix() * t_c2b.translation() + t_.translation();

    q_cam =  q_ * q_c2b;
    t_cam = q_.toRotationMatrix() * t_c2b.translation() + t_.translation();

    cam_pose = Eigen::Translation3d(t_cam) * q_cam;

    geometry_msgs::PoseStamped pose_cam;
    pose_cam.header.frame_id = "world";
    pose_cam.pose.position.x = t_cam.x();
    pose_cam.pose.position.y = t_cam.y();
    pose_cam.pose.position.z = t_cam.z();

    pose_cam.pose.orientation.w = q_cam.w();
    pose_cam.pose.orientation.x = q_cam.x();
    pose_cam.pose.orientation.y = q_cam.y();
    pose_cam.pose.orientation.z = q_cam.z();

    campose_pub.publish(pose_cam);
    ugvpose_pub.publish(ugv_pose_msg);
}

void alan::ArucoNodelet::uav_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    // std::cout<<1<<std::endl;
    uav_pose_msg = *pose;
    uav_pose_msg.header.frame_id = "world";

    uavpose_pub.publish(uav_pose_msg);
}


void alan::ArucoNodelet::map_SE3_to_pose(Sophus::SE3d pose)
{
    Eigen::Matrix3d cam_to_body;
    cam_to_body << 
        0,0,1,
        -1,0,0,
        0,-1,0;

    // cam_to_body.setIdentity();


    Eigen::Quaterniond q_aruco = Eigen::Quaterniond(pose.rotationMatrix() );
    Eigen::Translation3d t_arcuo = Eigen::Translation3d(pose.translation() );

    aruco_pose = t_arcuo * q_aruco;//in {c} frame

    // transfer to {w} frame
    Eigen::Quaterniond q_final = Eigen::Quaterniond(cam_pose.rotation() * cam_to_body * aruco_pose.rotation());
    Eigen::Vector3d led_in_world = cam_pose.rotation() * cam_to_body * aruco_pose.translation() + cam_pose.translation();// Eigen::Translation3d(led_pose.translation());


    Eigen::Vector3d uav_led_pose = q_final.toRotationMatrix() 
            * Eigen::Vector3d(LEDEX(0), LEDEX(1), LEDEX(2)) 
            + led_in_world; //in {w} frame but with offset

    Eigen::Translation3d t_final = Eigen::Translation3d(uav_led_pose);

    aruco_pose = t_final * q_final;

    //////////////////////////////
    aruco_pose_msg.header.stamp = aruco_pose_stamp;
    aruco_pose_msg.header.frame_id = "world";

    aruco_pose_msg.pose.position.x = t_final.translation().x();
    aruco_pose_msg.pose.position.y = t_final.translation().y();
    aruco_pose_msg.pose.position.z = t_final.translation().z();

    aruco_pose_msg.pose.orientation.w = q_final.w();
    aruco_pose_msg.pose.orientation.x = q_final.x();
    aruco_pose_msg.pose.orientation.y = q_final.y();
    aruco_pose_msg.pose.orientation.z = q_final.z();

    arucopose_pub.publish(aruco_pose_msg);

}

//pnp + BA implementation
void alan::ArucoNodelet::pose_w_aruco_pnp(cv::Mat& frame)
{
     std::vector<Eigen::Vector2d> pts_2d_detect;
    if(aruco_detect(frame, pts_2d_detect))
    {
        Eigen::Vector3d t;
        Eigen::Matrix3d R;
        // std::cout<<pts_2d_detect.size()<<std::endl;

        get_initial_pose(pts_2d_detect, body_frame_pts, R, t);
        

        //generate noise to validate BA
        Sophus::SE3d pose;
        if(add_noise)
            pose = pose_add_noise(t,R);
        else
            pose = Sophus::SE3d(R,t);


        //draw initial pose estimation
        // for(auto what : body_frame_pts)
        // {
        //     Eigen::Vector2d reproject = reproject_3D_2D(what, pose);            
        //     cv::circle(frame, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(255,0,0),-1);
        // }

        // if(body_frame_pts.size() == pts_2d_detect.size())
        //     optimize(pose, body_frame_pts, pts_2d_detect);//pose, body_frame_pts, pts_2d_detect

        // std::cout<<body_frame_pts.size()<<std::endl;
        for(auto what : body_frame_pts)
        {
            Eigen::Vector2d reproject = reproject_3D_2D(what, pose);    
            // std::cout<<reproject<<std::endl;        
            cv::circle(frame, cv::Point(reproject(0), reproject(1)), 5, CV_RGB(0,255,0),-1);
        }

        map_SE3_to_pose(pose);
    }    

}

inline Sophus::SE3d alan::ArucoNodelet::pose_add_noise(Eigen::Vector3d t, Eigen::Matrix3d R)
{
    //generate noise to validate BA
    double noise;
    
    std::default_random_engine generator;
    std::normal_distribution<double> dist(0, 0.32);

    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    noise = dist(generator);   
            
    Eigen::AngleAxisd rollAngle(0.872 * noise, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(0.872 * noise, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(0.872 * noise, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3d rotationMatrix = q.matrix();

    R = R * rotationMatrix;

    Eigen::Vector3d error(0.1 * noise, 0.1 * noise, 0.1 * noise);
    t = t + error;

    Sophus::SE3d pose_with_noise(R,t);
    
    return pose_with_noise;
}

inline Eigen::Vector2d alan::ArucoNodelet::reproject_3D_2D(Eigen::Vector3d P, Sophus::SE3d pose)
{
    Eigen::Vector3d result;


    Eigen::Matrix3d R = pose.rotationMatrix();
    Eigen::Vector3d t = pose.translation();

    result = this->cameraMat * (R * P + t); //dimension not right

    Eigen::Vector2d result2d;
    

    result2d <<
        result(0)/result(2), 
        result(1)/result(2);
    
    return result2d;
}

inline Eigen::Vector2d alan::ArucoNodelet::reproject_3D_2D_temp(Eigen::Vector3d P, Sophus::SE3f pose)
{
    Eigen::Vector3d result;

    Eigen::Matrix3d R = pose.rotationMatrix().cast<double>();
    Eigen::Vector3d t = pose.translation().cast<double>();

    result = this->cameraMat * (R * P + t); //dimension not right

    Eigen::Vector2d result2d;
    

    result2d <<
        result(0)/result(2), 
        result(1)/result(2);
    
    return result2d;
}

void alan::ArucoNodelet::get_initial_pose(std::vector<Eigen::Vector2d> pts_2d, std::vector<Eigen::Vector3d> body_frame_pts, Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    cv::Vec3d rvec, tvec;
    cv::Mat camMat = cv::Mat::eye(3,3,CV_64F);
    std::vector<cv::Point3d> pts_3d_;
    std::vector<cv::Point2d> pts_2d_;

    cv::Point3d temp3d;
    cv::Point2d temp2d;

    // std::cout<<"body_frame_pts..."<<std::endl;
    for(auto what : body_frame_pts)
    {
        // what = r_mirrored1 * what;
        // what = r_mirrored2 * what;


        // std::cout<<what<<std::endl<<std::endl;;
        

        temp3d.x = what(0);
        temp3d.y = what(1);
        temp3d.z = what(2);

        pts_3d_.push_back(temp3d);
    }

    // std::cout<<"end body_frame_pts..."<<std::endl;


    for(auto what : pts_2d)
    {
        temp2d.x = what(0);
        temp2d.y = what(1);


        pts_2d_.push_back(temp2d);
    }

    std::vector<cv::Point3d> pts_3d_final;
    std::vector<cv::Point2d> pts_2d_final;

    pts_3d_final.emplace_back(pts_3d_[2]);
    pts_3d_final.emplace_back(pts_3d_[1]);
    pts_3d_final.emplace_back(pts_3d_[0]);
    pts_3d_final.emplace_back(pts_3d_[3]);

    pts_2d_final.emplace_back(pts_2d_[2]);
    pts_2d_final.emplace_back(pts_2d_[1]);
    pts_2d_final.emplace_back(pts_2d_[0]);
    pts_2d_final.emplace_back(pts_2d_[3]);


    camMat.at<double>(0,0) = cameraMat(0,0);
    camMat.at<double>(0,2) = cameraMat(0,2);
    camMat.at<double>(1,1) = cameraMat(1,1);
    camMat.at<double>(1,2) = cameraMat(1,2);

    // std::cout<<"use epnp"<<std::endl;

    cv::solvePnP(pts_3d_final,pts_2d_final ,camMat, distCoeffs, rvec, tvec, cv::SOLVEPNP_EPNP);//, cv::SOLVEPNP_EPNP

    
    //return values
    cv::Mat rmat = cv::Mat::eye(3,3,CV_64F);
    cv::Rodrigues(rvec, rmat);


    R <<
        rmat.at<double>(0,0), rmat.at<double>(0,1), rmat.at<double>(0,2),
        rmat.at<double>(1,0), rmat.at<double>(1,1), rmat.at<double>(1,2),
        rmat.at<double>(2,0), rmat.at<double>(2,1), rmat.at<double>(2,2);

    Eigen::Matrix3d reverse_mat;
    reverse_mat <<
            1.0000000,  0.0000000,  0.0000000,
            0.0000000, -1.0000000, -0.0000000,
            0.0000000,  0.0000000, -1.0000000;

    

    t =  Eigen::Vector3d(
          tvec(0),
          tvec(1),
          tvec(2)  
        );

    if(tvec(2) < 0) //sometimes opencv yeilds reversed results, flip it 
    {
        R = R * reverse_mat;
        t = (-1) * t;
    }

}

bool alan::ArucoNodelet::aruco_detect(cv::Mat& frame, std::vector<Eigen::Vector2d>& pts_2d)
{
    std::vector<int> markerids;
    std::vector<std::vector<cv::Point2f>> markercorners, rejected;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(frame, dictionary, markercorners, markerids, parameters, rejected);

    if(markercorners.size() != 0)
    {
        if(markercorners.size() == 1)
        {
            double markerlength = 0.045;

            cv::aruco::estimatePoseSingleMarkers(markercorners, 0.045, cameraMatrix, distCoeffs, rvecs, tvecs);
      
            cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 0.1);
            cv::Mat rmat = cv::Mat::eye(3,3,CV_64F);
            cv::Rodrigues(rvecs[0], rmat);

            Eigen::Matrix3d R;
            Eigen::Vector3d t;

            Eigen::Matrix<double, 4, 4> cam_to_body;
            cam_to_body << 0,0,1,0,
                -1,0,0,0,
                0,-1,0,0,
                0,0,0,1;

            R <<
                rmat.at<double>(0,0), rmat.at<double>(0,1), rmat.at<double>(0,2),
                rmat.at<double>(1,0), rmat.at<double>(1,1), rmat.at<double>(1,2),
                rmat.at<double>(2,0), rmat.at<double>(2,1), rmat.at<double>(2,2);
            // std::cout<<"R_aruco:"<<std::endl;
            // std::cout<<R<<std::endl;

            t <<
                tvecs[0](0),
                tvecs[0](1),
                tvecs[0](2);
            
            pose_aruco = Sophus::SE3d(R,t);
        }
        
        for(auto& what : markercorners)
        {
            if(what.size() != 4)
                continue;

            for(auto& that : what)
            {
                // std::cout<<that<<std::endl;
                cv::circle(frame, cv::Point(that.x, that.y), 4, CV_RGB(0,0,255),-1);
                Eigen::Vector2d result;
                result << 
                    that.x,
                    that.y;
                pts_2d.push_back(result);
            }

            // std::cout<<"end aruco_detect"<<std::endl;
            // std::cout<<"------"<<std::endl;

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

void alan::ArucoNodelet::solveJacobian(Eigen::Matrix<double, 2, 6>& Jacob, Sophus::SE3d pose, Eigen::Vector3d point_3d)
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
    
    // std::cout<<"Jacob here: "<<Jacob<<std::endl;        
}

void alan::ArucoNodelet::optimize(Sophus::SE3d& pose, std::vector<Eigen::Vector3d> pts_3d_exists, std::vector<Eigen::Vector2d> pts_2d_detected)
//converge problem need to be solved //-> fuck you, your Jacobian was wrong
{
    //execute Gaussian-Newton Method
    // std::cout<<"Bundle Adjustment Optimization"<<std::endl;

    const int MAX_ITERATION = 400;

    const double converge_threshold = 1e-6;

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
        // std::cout<<"this is the: "<<i<<" th iteration."<<std::endl;
        A.setZero();
        b.setZero();

        cost = 0;
        

        for(int i=0; i < points_no; i++)
        {
            //get the Jacobian for this point
            pts_3d_exists[i] = pts_3d_exists[i];
            solveJacobian(J, pose, pts_3d_exists[i]);

            e = pts_2d_detected[i] - reproject_3D_2D(pts_3d_exists[i], pose) ; 
            //jibai, forget to minus the detected points
            //you set your Jacobian according to "u-KTP/s", and you messed up the order, you fat fuck
    
            cost += e.norm();

            //form Ax = b
            A += J.transpose() * J;
            b += -J.transpose() * e;
        }
    
        //solve Adx = b
        dx = A.ldlt().solve(b);

        for(int i = 0; i < 6; i++)
            if(isnan(dx(i,0)))
                break;

        // std::cout<<"previous: "/*<<std::cout.precision(10)*/<< lastcost<<std::endl;
        // std::cout<<"currentt: "/*<<std::cout.precision(10)*/<< cost<<std::endl;

        // if(i > 0 && cost >= lastcost)
        //     break;

        pose = Sophus::SE3d::exp(dx) * pose;

        lastcost = cost;

        if(dx.norm() < converge_threshold)        
            break;
    }

    // std::cout<<"BA: "<<lastcost<<std::endl;

    // std::cout<<"gone thru: "<<i<<" th, end optimize"<<std::endl<<std::endl;;;

}


void* alan::ArucoNodelet::PubMainLoop(void* tmp)
{
    ArucoNodelet* pub = (ArucoNodelet*) tmp;

    // ros::Rate loop_rate(50);
    // while (ros::ok()) 
    // {
    //     // ROS_INFO("%d,publish!", num++);
    //     pub->pubpose.publish(pub->pose_estimated);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    
    void* return_value;

    return return_value;
}

//ICP
void alan::ArucoNodelet::pose_w_aruco_icp(cv::Mat& rgbframe, cv::Mat& depthframe)
{
    std::vector<Eigen::Vector2d> pts_2d_detect;

    if(aruco_detect(rgbframe, pts_2d_detect))
    {
        std::vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depthframe);  

        // for(auto what : pts_3d_pcl_detect)
        // {
        //     std::cout<<what<<std::endl;
        // }      
        // std::cout<<std::endl;

        Eigen::Vector3f t;
        Eigen::Matrix3f R;

        // solveicp_svd(pts_3d_pcl_detect, body_frame_pts, R, t);





        // std::cout<<pts_3d_pcl_detect.size()<<std::endl;
        // std::cout<<body_frame_pts.size()<<std::endl;
        // std::cout<<

        // pcl::PointCloud<pcl::PointXYZ>::Ptr pts_3d_pcl_detect_pcl = eigen_2_pcl(pts_3d_pcl_detect);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr body_frame_pts_pcl = eigen_2_pcl(body_frame_pts);

        // std::cout<<"here shows the size"<<std::endl;
        // std::cout<<pts_3d_pcl_detect_pcl->size()<<std::endl;
        // std::cout<<body_frame_pts_pcl->size()<<std::endl<<std::endl;;

        // Eigen::Matrix4f Transformation = icp_pcl(body_frame_pts_pcl, pts_3d_pcl_detect_pcl);

        // std::cout<<Transformation.size()<<std::endl;

        // R = Transformation.block<3,3>(0,0);
        // t = Transformation.block<3,1>(0,3);
        
        //generate noise to validate BA
        Sophus::SE3f pose;
        // if(add_noise)
        //     std::cout<<"hji"<<std::endl;
        //     // pose = pose_add_noise(t,R);
        // else
        // {            
        //     std::cout<<R<<std::endl;
        //     std::cout<<t<<std::endl;  
        //     pose = Sophus::SE3f(R,t);        
        // }

        // double e = 0;
        // Eigen::Vector2d reproject, error; 

        // for(int i = 0 ; i < body_frame_pts.size(); i++)
        // {
        //     reproject = reproject_3D_2D(body_frame_pts[i], pose);  
        //     error = pts_2d_detect[i] - reproject;
        //     e = e + error.norm();
        // }

        // std::cout<<"error: "<<e<<std::endl;

        for(auto what : body_frame_pts)
        {
            Eigen::Vector2d reproject = reproject_3D_2D_temp(what, pose);            
            cv::circle(frame, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(255,0,0),-1);
        }

        // if(e > 20.0)
        // {
        //     std::cout<<"use pnp instead"<<std::endl;
        //     use_pnp_instead(frame, pts_2d_detect, pose);
        // }

        // // optimize(pose, body_frame_pts, pts_2d_detect);//pose, body_frame_pts, pts_2d_detect

        // for(auto what : body_frame_pts)
        // {
        //     Eigen::Vector2d reproject = reproject_3D_2D(what, pose);            
        //     cv::circle(frame, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(0,255,0),-1);
        // }

        // map_SE3_to_pose(pose);
    }    

}

void alan::ArucoNodelet::use_pnp_instead(cv::Mat frame, std::vector<Eigen::Vector2d> pts_2d_detect, Sophus::SE3d& pose)
{
    Eigen::Vector3d t;
    Eigen::Matrix3d R;

    get_initial_pose(pts_2d_detect, body_frame_pts, R, t);
    
    //generate noise to validate BA
    Sophus::SE3d pose_;
    if(add_noise)
        pose_ = pose_add_noise(t,R);
    else
        pose_ = Sophus::SE3d(R,t);


    optimize(pose_, body_frame_pts, pts_2d_detect);

    pose = pose_;

}

void alan::ArucoNodelet::solveicp_svd(std::vector<Eigen::Vector3d> pts_3d_camera, std::vector<Eigen::Vector3d> pts_3d_body, Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    //here we assume known correspondences

    //SVD Solution proposed in ->
    //Arun, K. Somani, Thomas S. Huang, and Steven D. Blostein. 
    //"Least-squares fitting of two 3-D point sets." 
    //IEEE Transactions on pattern analysis and machine intelligence 5 (1987): 698-700.
    
    // std::cout<<"enter icp"<<std::endl;


    Eigen::Vector3d CoM_camera = get_CoM(pts_3d_camera);
    Eigen::Vector3d CoM_body   = get_CoM(pts_3d_body);

    // std::cout<<"enter 0"<<std::endl;

    int no_of_paired_points = pts_3d_body.size();

    std::vector<Eigen::Vector3d> q(no_of_paired_points), q_(no_of_paired_points);

    // std::cout<<"enter 1"<<std::endl;
    
    for(int i = 0; i < no_of_paired_points; i++)
    {
        q [i] = pts_3d_body[i]   - CoM_body;   //R3*1
        q_[i] = pts_3d_camera[i] - CoM_camera; //R3*3
    }
    
    // std::cout<<"icp half"<<std::endl;

    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

    for (int i = 0 ; i < no_of_paired_points; i++)
    {
        H += Eigen::Vector3d(q_[i].x(), q_[i].y(), q_[i].z()) 
             * Eigen::Vector3d(q[i].x(), q[i].y(), q[i].z()).transpose();
    }

    //solve SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    Eigen::Matrix3d R_ = U * V.transpose();

    if(R_.determinant() < 0)
    {
        // std::cout<<"R.det < 0" <<std::endl;
        R_ = -R_;
    }

    R = R_;
    t = CoM_camera  - R * CoM_body;
}

Eigen::Vector3d alan::ArucoNodelet::get_CoM(std::vector<Eigen::Vector3d> pts_3d)
{
    double x = 0, y = 0, z = 0;
    for(int i = 0; i < pts_3d.size(); i++)
    {
        x = x + pts_3d[i].x();
        y = y + pts_3d[i].y();
        z = z + pts_3d[i].z();
    }

    Eigen::Vector3d CoM = Eigen::Vector3d(
        x / pts_3d.size(),
        y / pts_3d.size(),
        z / pts_3d.size());
    
    // std::cout<<"CoM: "<<CoM<<std::endl;

    return CoM;
}

std::vector<Eigen::Vector3d> alan::ArucoNodelet::pointcloud_generate(std::vector<Eigen::Vector2d> pts_2d_detected, cv::Mat depthimage)
{
    //get 9 pixels around the point of interest

    int no_pixels = 9;
    int POI_width = (sqrt(9) - 1 ) / 2;

    std::vector<Eigen::Vector3d> pointclouds;

    int x_pixel, y_pixel;
    Eigen::Vector3d temp;

    for(int i = 0; i < pts_2d_detected.size(); i++)
    {

        x_pixel = pts_2d_detected[i].x();
        y_pixel = pts_2d_detected[i].y();
        
        cv::Point depthbox_vertice1 = cv::Point(x_pixel - POI_width, y_pixel - POI_width);
        cv::Point depthbox_vertice2 = cv::Point(x_pixel + POI_width, y_pixel + POI_width);
        cv::Rect letsgetdepth(depthbox_vertice1, depthbox_vertice2);

        cv::Mat ROI(depthimage, letsgetdepth);
        cv::Mat ROIframe;
        ROI.copyTo(ROIframe);
        std::vector<cv::Point> nonzeros;

        cv::findNonZero(ROIframe, nonzeros);
        std::vector<double> nonzerosvalue;
        for(auto temp : nonzeros)
        {
            double depth = ROIframe.at<ushort>(temp);
            nonzerosvalue.push_back(depth);
        }

        double depth_average;
        if(nonzerosvalue.size() != 0)
            depth_average = accumulate(nonzerosvalue.begin(), nonzerosvalue.end(),0.0)/nonzerosvalue.size();

        double z_depth = 0.001 * depth_average;

        temp.x() = x_pixel;
        temp.y() = y_pixel;
        temp.z() = 1;

        temp = z_depth * cameraMat.inverse() * temp;

        
        pointclouds.push_back(temp);
    }

    return pointclouds;
}


Eigen::Vector3d alan::ArucoNodelet::q2rpy(Eigen::Quaterniond q) 
{
    return q.toRotationMatrix().eulerAngles(2,1,0);
}

Eigen::Quaterniond alan::ArucoNodelet::rpy2q(Eigen::Vector3d rpy)
{
    Eigen::AngleAxisd rollAngle(rpy(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rpy(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rpy(2), Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    return q;
}

Eigen::Vector3d alan::ArucoNodelet::q_rotate_vector(Eigen::Quaterniond q, Eigen::Vector3d v)
{
    return q * v;
}