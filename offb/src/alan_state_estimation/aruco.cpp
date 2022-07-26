//the aruco library is used for baseline validations as well as trial and error of algorithms
//Author: Li-Yu LO

#ifndef ARUCO_H
#define ARUCO_H
#include "include/aruco.h"

void alan_pose_estimation::ArucoNodelet::camera_callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth)
//void alan_pose_estimation::ArucoNodelet::camera_callback(const sensor_msgs::CompressedImage::ConstPtr& rgbimage)
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

    // if(temp_i < 4)
    // {
    //     if(temp_i<1)
    //     {
    //         ofstream save("/home/patty/alan_ws/src/alan/offb/src/alan_state_estimation/test/test.txt", ios::app); 
    //         save<<image_dep<<endl;
    //         save.close();
    //     }
    //     cv::imwrite("/home/patty/alan_ws/src/alan/offb/src/alan_state_estimation/test/" + to_string(temp_i) + ".png", frame);
    //     cv::imwrite("/home/patty/alan_ws/src/alan/offb/src/alan_state_estimation/test/" + to_string(temp_i) + "_d.png", image_dep);
    // }

    // temp_i ++ ;

    // std::cout<<frame.size()<<std::endl;
    // std::cout<<image_dep.size()<<std::endl;

    double t1 = ros::Time::now().toSec();  

    pose_w_aruco_icp(frame, image_dep);
    // pose_w_aruco_pnp(test);//, image_dep);

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




//pnp + BA implementation
void alan_pose_estimation::ArucoNodelet::pose_w_aruco_pnp(cv::Mat& frame)
{

    vector<Eigen::Vector2d> pts_2d_detect;
    if(aruco_detect(frame, pts_2d_detect))
    {
        Eigen::Vector3d t;
        Eigen::Matrix3d R;

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

        if(body_frame_pts.size() == pts_2d_detect.size())
            optimize(pose, body_frame_pts, pts_2d_detect);//pose, body_frame_pts, pts_2d_detect

        for(auto what : body_frame_pts)
        {
            Eigen::Vector2d reproject = reproject_3D_2D(what, pose);            
            cv::circle(frame, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(0,255,0),-1);
        }

        // map_SE3_to_pose(pose);
    }    

}

inline Sophus::SE3d alan_pose_estimation::ArucoNodelet::pose_add_noise(Eigen::Vector3d t, Eigen::Matrix3d R)
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

inline Eigen::Vector2d alan_pose_estimation::ArucoNodelet::reproject_3D_2D(Eigen::Vector3d P, Sophus::SE3d pose)
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

void alan_pose_estimation::ArucoNodelet::get_initial_pose(vector<Eigen::Vector2d> pts_2d, vector<Eigen::Vector3d> body_frame_pts, Eigen::Matrix3d& R, Eigen::Vector3d& t)
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

    cv::solvePnP(pts_3d_, pts_2d_ ,camMat, distCoeffs, rvec, tvec, cv::SOLVEPNP_EPNP);
    
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

    // cout<<"PnP: "<<R<<endl<<endl;;

    // cout<<"PnP: "<<t<<endl<<endl;;

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

        // cout<<"previous: "/*<<cout.precision(10)*/<< lastcost<<endl;
        // cout<<"currentt: "/*<<cout.precision(10)*/<< cost<<endl;

        // if(i > 0 && cost >= lastcost)
        //     break;

        pose = Sophus::SE3d::exp(dx) * pose;

        lastcost = cost;

        if(dx.norm() < converge_threshold)        
            break;
    }

    cout<<"BA: "<<lastcost<<endl;

    cout<<"gone thru: "<<i<<" th, end optimize"<<endl<<endl;;;

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



//ICP
void alan_pose_estimation::ArucoNodelet::pose_w_aruco_icp(cv::Mat& rgbframe, cv::Mat& depthframe)
{
    vector<Eigen::Vector2d> pts_2d_detect;

    if(aruco_detect(rgbframe, pts_2d_detect))
    {
        vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depthframe);  

        // for(auto what : pts_3d_pcl_detect)
        // {
        //     cout<<what<<endl;
        // }      
        // cout<<endl;

        Eigen::Vector3d t;
        Eigen::Matrix3d R;

        solveicp_svd(pts_3d_pcl_detect, body_frame_pts, R, t);
        

        //generate noise to validate BA
        Sophus::SE3d pose;
        if(add_noise)
            pose = pose_add_noise(t,R);
        else
            pose = Sophus::SE3d(R,t);

        double e = 0;
        Eigen::Vector2d reproject, error; 

        for(int i = 0 ; i < body_frame_pts.size(); i++)
        {
            reproject = reproject_3D_2D(body_frame_pts[i], pose);  
            error = pts_2d_detect[i] - reproject;
            e = e + error.norm();
        }

        cout<<"error: "<<e<<endl;

        for(auto what : body_frame_pts)
        {
            Eigen::Vector2d reproject = reproject_3D_2D(what, pose);            
            cv::circle(frame, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(255,0,0),-1);
        }

        if(e > 20.0)
        {
            cout<<"use pnp instead"<<endl;
            use_pnp_instead(frame, pts_2d_detect, pose);
        }

        // optimize(pose, body_frame_pts, pts_2d_detect);//pose, body_frame_pts, pts_2d_detect

        for(auto what : body_frame_pts)
        {
            Eigen::Vector2d reproject = reproject_3D_2D(what, pose);            
            cv::circle(frame, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(0,255,0),-1);
        }

        map_SE3_to_pose(pose);
    }    

}

void alan_pose_estimation::ArucoNodelet::use_pnp_instead(cv::Mat frame, vector<Eigen::Vector2d> pts_2d_detect, Sophus::SE3d& pose)
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

void alan_pose_estimation::ArucoNodelet::solveicp_svd(vector<Eigen::Vector3d> pts_3d_camera, vector<Eigen::Vector3d> pts_3d_body, Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    //here we assume known correspondences

    //SVD Solution proposed in ->
    //Arun, K. Somani, Thomas S. Huang, and Steven D. Blostein. 
    //"Least-squares fitting of two 3-D point sets." 
    //IEEE Transactions on pattern analysis and machine intelligence 5 (1987): 698-700.
    
    // cout<<"enter icp"<<endl;


    Eigen::Vector3d CoM_camera = get_CoM(pts_3d_camera);
    Eigen::Vector3d CoM_body   = get_CoM(pts_3d_body);

    // cout<<"enter 0"<<endl;

    int no_of_paired_points = pts_3d_body.size();

    vector<Eigen::Vector3d> q(no_of_paired_points), q_(no_of_paired_points);

    // cout<<"enter 1"<<endl;
    
    for(int i = 0; i < no_of_paired_points; i++)
    {
        q [i] = pts_3d_body[i]   - CoM_body;   //R3*1
        q_[i] = pts_3d_camera[i] - CoM_camera; //R3*3
    }
    
    // cout<<"icp half"<<endl;

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
        // cout<<"R.det < 0" <<endl;
        R_ = -R_;
    }

    R = R_;
    t = CoM_camera  - R * CoM_body;
}

Eigen::Vector3d alan_pose_estimation::ArucoNodelet::get_CoM(vector<Eigen::Vector3d> pts_3d)
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
    
    // cout<<"CoM: "<<CoM<<endl;

    return CoM;
}

vector<Eigen::Vector3d> alan_pose_estimation::ArucoNodelet::pointcloud_generate(vector<Eigen::Vector2d> pts_2d_detected, cv::Mat depthimage)
{
    //get 9 pixels around the point of interest

    int no_pixels = 9;
    int POI_width = (sqrt(9) - 1 ) / 2;

    vector<Eigen::Vector3d> pointclouds;

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
        vector<cv::Point> nonzeros;

        cv::findNonZero(ROIframe, nonzeros);
        vector<double> nonzerosvalue;
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



#endif