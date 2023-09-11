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
 * \file led.cpp
 * \date 28/07/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for vision-based relative localization for UAV and UGV based on LED markers
 */

#include "include/led.h"

void alan::LedNodelet::camera_callback(const sensor_msgs::CompressedImage::ConstPtr & rgbmsg, const sensor_msgs::Image::ConstPtr & depthmsg)
{
    total_no++;
    cv_bridge::CvImageConstPtr depth_ptr;
    led_pose_header = rgbmsg->header;

    try
    {
        depth_ptr  = cv_bridge::toCvCopy(depthmsg, depthmsg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat depth = depth_ptr->image;

    try
    {
        frame = cv::imdecode(cv::Mat(rgbmsg->data), 1);
        display = frame.clone();
        hsv = frame.clone();
        frame_temp = frame.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    double tick = ros::Time::now().toSec(); 

    if(tick - last_request > ros::Duration(0.1).toSec() && nodelet_activated)
    {
        LED_tracker_initiated_or_tracked = false;
        printf("\033c");
        ROS_RED_STREAM("RESET TERMINAL!");
    }           
           
    solve_pose_w_LED(frame, depth);
    
    double tock = ros::Time::now().toSec();  

    terminal_msg_display(1 / (tock - tick));

    set_image_to_publish(1 / (tock - tick), rgbmsg);

    if(LED_tracker_initiated_or_tracked)
        log(tock - tick);
                    
    last_request = ros::Time::now().toSec();

    if(!nodelet_activated)
        nodelet_activated = true;

    led_pose_header_previous = led_pose_header;

} 

Sophus::SE3d alan::LedNodelet::posemsg_to_SE3(const geometry_msgs::PoseStamped pose)
{
    return Sophus::SE3d(
        Eigen::Quaterniond(
            ugv_pose_msg.pose.orientation.w,
            ugv_pose_msg.pose.orientation.x,
            ugv_pose_msg.pose.orientation.y,
            ugv_pose_msg.pose.orientation.z  
        ).normalized().toRotationMatrix(),
        Eigen::Translation3d(
            ugv_pose_msg.pose.position.x,
            ugv_pose_msg.pose.position.y,
            ugv_pose_msg.pose.position.z
        ).translation()
    );
}

geometry_msgs::PoseStamped alan::LedNodelet::SE3_to_posemsg(
    const Sophus::SE3d pose_on_SE3, 
    const std_msgs::Header msgHeader
)
{
    geometry_msgs::PoseStamped returnPoseMsg;
    returnPoseMsg.header = msgHeader;

    returnPoseMsg.pose.position.x = pose_on_SE3.translation().x();
    returnPoseMsg.pose.position.y = pose_on_SE3.translation().y();
    returnPoseMsg.pose.position.z = pose_on_SE3.translation().z();

    returnPoseMsg.pose.orientation.w = pose_on_SE3.unit_quaternion().w();
    returnPoseMsg.pose.orientation.x = pose_on_SE3.unit_quaternion().x();
    returnPoseMsg.pose.orientation.y = pose_on_SE3.unit_quaternion().y();
    returnPoseMsg.pose.orientation.z = pose_on_SE3.unit_quaternion().z();

    return returnPoseMsg;
}

void alan::LedNodelet::ugv_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    pose_cam_inWorld_SE3 = pose_ugv_inWorld_SE3 * pose_cam_inUgvBody_SE3;

    geometry_msgs::PoseStamped cam_pose_msg = SE3_to_posemsg(
        pose_cam_inWorld_SE3, 
        pose->header
    );

    campose_pub.publish(cam_pose_msg);

    pose_ugv_inWorld_SE3 = posemsg_to_SE3(*pose); 
    
    ugv_pose_msg = *pose;
    ugv_pose_msg.header.frame_id = "world";
    ugvpose_pub.publish(ugv_pose_msg);
}

void alan::LedNodelet::uav_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    pose_uav_inWorld_SE3 = posemsg_to_SE3(*pose);
    
    uav_pose_msg = *pose;
    uav_pose_msg.header.frame_id = "world";
    uavpose_pub.publish(uav_pose_msg);
}

void alan::LedNodelet::uav_setpt_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uav_stpt_msg = *pose;
}

void alan::LedNodelet::map_SE3_to_pose(Sophus::SE3d pose_led_inCamera_SE3)
{   
    pose_cam_inGeneralBodySE3 * pose_led_inCamera_SE3; //now we in body frame

    pose_led_inWorld_SE3 = 
        pose_cam_inWorld_SE3 
        * pose_led_inUavBodyOffset_SE3 
        * pose_cam_inGeneralBodySE3 
        * pose_led_inCamera_SE3;
    
    led_pose_header.frame_id = "world";
    led_pose_estimated_msg = SE3_to_posemsg(pose_led_inWorld_SE3, led_pose_header);

    ledpose_pub.publish(led_pose_estimated_msg);

    //odom publish
    // led_odom_estimated.header.stamp = led_pose_header;
    // led_odom_estimated.header.frame_id = "world";

    // led_odom_estimated.pose.pose.position.x = t_final.translation().x();
    // led_odom_estimated.pose.pose.position.y = t_final.translation().y();
    // led_odom_estimated.pose.pose.position.z = t_final.translation().z();

    // led_odom_estimated.pose.pose.orientation.w = q_final.w();
    // led_odom_estimated.pose.pose.orientation.x = q_final.x();
    // led_odom_estimated.pose.pose.orientation.y = q_final.y();
    // led_odom_estimated.pose.pose.orientation.z = q_final.z();

    // led_odom_estimated.twist.twist.linear.x = led_twist_current(0);
    // led_odom_estimated.twist.twist.linear.y = led_twist_current(1);
    // led_odom_estimated.twist.twist.linear.z = led_twist_current(2);

    // led_odom_estimated.twist.twist.angular.x = led_twist_current(3);
    // led_odom_estimated.twist.twist.angular.y = led_twist_current(4);
    // led_odom_estimated.twist.twist.angular.z = led_twist_current(5);
    
    // ledodom_pub.publish(led_odom_estimated);

    // Sophus::SE3d led_twist_sophus = Sophus::SE3d(led_twist_current);
    // led_twist_estimated.twist.linear.x = led_twist_sophus.ro         
 
}

void alan::LedNodelet::solve_pose_w_LED(cv::Mat& frame, cv::Mat depth)
{    
    if(!LED_tracker_initiated_or_tracked)        
    {
        LED_tracker_initiated_or_tracked = LED_tracking_initialize(frame, depth);
        // try to initialize here...

        if(LED_tracker_initiated_or_tracked)
        {
            printf("\n");
            ROS_GREEN_STREAM("TRACKER INITIALIZED!");
            ROS_GREEN_STREAM("SHOULD BE FINE...HOPEFULLY?\n");

            if(BA_error > LED_no * 2)
            {
                ROS_WARN("REPROJECTION_ERROR OVER @ INITIALIZATION %d", LED_no * 2);          
            }                    

            if(!kf_initiated)
            {
                kf_initiated = true;
                apiKF(kfINITIATE);                
            }
            else
                apiKF(kfREINITIATE);            
            
            map_SE3_to_pose(pose_global_sophus);
        }
        else
        {
            ROS_CYAN_STREAM("WAITING FOR INITIALIZATION...");
        }

    }
    else
    {
        recursive_filtering(frame, depth);

        if(!LED_tracker_initiated_or_tracked)
        {
            // error_no++;
            ROS_RED_STREAM("TRACKER FAIL");
            // reinitKF()
            // cv::imwrite("/home/patty/alan_ws/haha.jpg", frame);
            // cv::imwrite("/home/patty/alan_ws/lala.jpg", frame_input);
            // detect_no = 0;            
            ros::shutdown();
        }
        else       
            map_SE3_to_pose(pose_global_sophus);
        
    }

}

void alan::LedNodelet::apiKF(int DOKF)
{
    switch (DOKF)
    {
    case kfINITIATE:
        /* code */
        initKF(pose_global_sophus);
        break;
    
    case kfREINITIATE:
        /* code */
        reinitKF(pose_global_sophus);
        break;

    case kfNORMALKF:
        /* code */
        // Measurement Here;
        run_AIEKF(
            led_pose_header.stamp.toSec() - led_pose_header_previous.stamp.toSec(),
            pts_on_body_frame_in_corres_order, 
            pts_detected_in_corres_order
        );
        break;
    
    default:
        ROS_RED_STREAM("APIKF GOT ERROR; PLS DEBUG...");
        break;
    }
}

void alan::LedNodelet::recursive_filtering(cv::Mat& frame, cv::Mat depth)
{
    std::vector<Eigen::Vector2d> pts_2d_detect;
    double t0 = ros::Time::now().toSec();
    bool sufficient_pts = LED_pts_measurement(
        frame,
        depth,
        pts_2d_detect
    );
    double t1 = ros::Time::now().toSec();

    if(!sufficient_pts)
    {
        ROS_RED_STREAM("INSUFFICIENT LED DETECTED");
        LED_tracker_initiated_or_tracked = false;
        return;
    }

    double t2 = ros::Time::now().toSec();
    if(!search_corres_and_pose_predict(pts_2d_detect))
    {
        LED_tracker_initiated_or_tracked = false;                                    
    }
    double t3 = ros::Time::now().toSec();

    // std::cout<<"==================="<<std::endl;
    // std::cout<<(t1 - t0)<<std::endl;
    // std::cout<<(t3 - t2)<<std::endl;
}

bool alan::LedNodelet::search_corres_and_pose_predict(std::vector<Eigen::Vector2d> pts_2d_detect)
{
    double t0 = ros::Time::now().toSec();
    correspondence_search_kmeans(pts_2d_detect);
    double t1 = ros::Time::now().toSec();
    // std::cout<<"====corr time===="<<std::endl;
    // std::cout<<(t1 - t0)<<std::endl;

    pts_on_body_frame_in_corres_order.clear();
    pts_detected_in_corres_order.clear(); 

    for(int i = 0; i < corres_global.size(); i++)
    {            
        if(corres_global[i].detected_ornot)
        {   
            pts_detected_in_corres_order.push_back(corres_global[i].pts_2d_correspond);             
            pts_on_body_frame_in_corres_order.push_back(pts_on_body_frame[i]);
            corres_global[i].detected_ornot = false; // reset for next time step
        }        
    }

    detect_no = pts_on_body_frame_in_corres_order.size();

    if(detect_no < 4)
    {
        std::cout<<"in search_corres: less than 4"<<std::endl;
        return false;
    }
    else
    {      
        double t10 = ros::Time::now().toSec();
        apiKF(kfNORMALKF);
        double reproject_error = get_reprojection_error(
            pts_on_body_frame_in_corres_order,
            pts_detected_in_corres_order,
            pose_global_sophus,
            true
        );
        double t11 = ros::Time::now().toSec();

        // std::cout<<"====opti time===="<<std::endl;
        // std::cout<<(t11 - t10)<<std::endl;

        
        pose_global_sophus = returnResults.X.X_SE3;
        BA_error = returnResults.residual_error;


        return true;
    }    
}

void alan::LedNodelet::solve_pnp_initial_pose(std::vector<Eigen::Vector2d> pts_2d, std::vector<Eigen::Vector3d> pts_3d)
{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    // distCoeffs.at<double>(0) = -0.056986890733242035;
    // distCoeffs.at<double>(1) = 0.06356718391180038;
    // distCoeffs.at<double>(2) = -0.0012483829632401466;
    // distCoeffs.at<double>(3) = -0.00018130485841538757;
    // distCoeffs.at<double>(4) = -0.019809694960713387;

    cv::Mat no_ro_rmat = cv::Mat::eye(3,3,CV_64F);
    
    cv::Vec3d rvec, tvec;
    // cv::Rodrigues(no_ro_rmat, rvec);

    cv::Mat camMat = cv::Mat::eye(3,3,CV_64F);
    std::vector<cv::Point3d> pts_3d_;
    std::vector<cv::Point2d> pts_2d_;

    cv::Point3d temp3d;
    cv::Point2d temp2d;

    for(auto what : pts_3d)
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
    //opt pnp algorithm
    //, cv::SOLVEPNP_EPNP
    //, cv::SOLVEPNP_IPPE
    //, cv::SOLVEPNP_P3P

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

    pose_epnp_sophus = Sophus::SE3d(R, t);
    
    // pose_depth_sophus = Sophus::SE3d(R, t);

    // if(LED_tracker_initiated_or_tracked)
    // {
    //     // cout<<"depth"<<endl;
    //     t = led_3d_posi_in_camera_frame_depth;
    //     pose_depth_sophus = Sophus::SE3d(R, t);
    // }

}


/* ================ POI Extraction utilities function below ================ */

bool alan::LedNodelet::LED_pts_measurement(
    cv::Mat& frame, 
    cv::Mat& depth, 
    std::vector<Eigen::Vector2d>& pts_2d_detect
)
{
    pts_2d_detect = LED_extract_POI(frame, depth);
    //to extract POI point of interest  

    if(pts_2d_detect.empty())
    {
        LED_tracker_initiated_or_tracked = false;
        return false;
    }

    reject_outlier(pts_2d_detect, depth);

    bool sufficient_pts = get_final_POI(pts_2d_detect);
    // the former retrieved POI could be noisy,
    // so get final POI
    
    return sufficient_pts;
}

std::vector<Eigen::Vector2d> alan::LedNodelet::LED_extract_POI(cv::Mat& frame, cv::Mat depth)
{   
    cv::Mat depth_mask_src = depth.clone(), depth_mask_dst1, depth_mask_dst2;

    cv::threshold(depth_mask_src, depth_mask_dst1, LANDING_DISTANCE * 1000, 50000, cv::THRESH_BINARY_INV);
    //filter out far depths

    cv::threshold(depth_mask_src, depth_mask_dst2, 0.5, 50000, cv::THRESH_BINARY); 
    //filter out zeros

    cv::bitwise_and(depth_mask_dst1, depth_mask_dst2, depth_mask_src);
    
    depth_mask_src.convertTo(depth_mask_src, CV_8U);
   
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    cv::threshold(frame, frame, BINARY_THRES, 255, cv::THRESH_BINARY);
    frame_initial_thresholded = frame.clone();

    // detect frame after filter out background
    cv::bitwise_and(depth_mask_src, frame, frame); //filter out with depth information

    // Blob method
    std::vector<cv::KeyPoint> keypoints_rgb_d;
	cv::SimpleBlobDetector::Params params;

	params.filterByArea = false;
    params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByConvexity = false;
	params.filterByInertia = false;
    params.minDistBetweenBlobs = 0.01;

	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    detector->detect(frame, keypoints_rgb_d);
	cv::drawKeypoints( frame, keypoints_rgb_d, im_with_keypoints,CV_RGB(255,0,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    
    blobs_for_initialize = keypoints_rgb_d;

    min_blob_size = INFINITY;

    std::vector<Eigen::Vector2d> POI;
    for(auto what : keypoints_rgb_d)
    {
        min_blob_size =(what.size < min_blob_size ? what.size : min_blob_size);
        POI.push_back(Eigen::Vector2d(what.pt.x, what.pt.y));
    }

    return POI;
}

std::vector<Eigen::Vector3d> alan::LedNodelet::pointcloud_generate(std::vector<Eigen::Vector2d> pts_2d_detected, cv::Mat depthimage)
{
    //get 9 pixels around the point of interest
    int no_pixels = 9;
    int POI_width = (sqrt(9) - 1 ) / 2;

    std::vector<Eigen::Vector3d> pointclouds;

    int x_pixel, y_pixel;
    Eigen::Vector3d temp;

    depth_avg_of_all = 0;
    

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

        depth_avg_of_all = depth_avg_of_all + z_depth;

        temp.x() = x_pixel;
        temp.y() = y_pixel;
        temp.z() = 1;

        temp = z_depth * cameraMat.inverse() * temp;
        
        pointclouds.push_back(temp);
    }

    depth_avg_of_all = depth_avg_of_all / pointclouds.size();

    // cout<<"pcl"<<endl;
    // cout<<depth_avg_of_all<<endl;

    return pointclouds;
}

bool alan::LedNodelet::get_final_POI(std::vector<Eigen::Vector2d>& pts_2d_detected)
{
    double x_min = INFINITY, y_min = INFINITY;
    double x_max = -INFINITY, y_max = -INFINITY;

    for(auto what : pts_2d_detected)
    {
        x_min = (what.x() < x_min ? what.x() : x_min);        
        y_min = (what.y() < y_min ? what.y() : y_min);        
        x_max = (what.x() > x_max ? what.x() : x_max);
        y_max = (what.y() > y_max ? what.y() : y_max);

    }
    
    double image_ROI_width = (x_max - x_min) * 1.5;
    double image_ROI_height = (y_max - y_min) * 1.5;

    cv::Rect rect_ROI(x_min, y_min, image_ROI_width, image_ROI_height);

    cv::Mat ROI_mask = cv::Mat::zeros(
        frame_initial_thresholded.size(),
        frame_initial_thresholded.type()
    );

    cv::rectangle(ROI_mask, rect_ROI, CV_RGB(255, 255, 255), -1, 8, 0);

    cv::Mat final_ROI;
    frame_initial_thresholded.copyTo(final_ROI, ROI_mask);

    // Blob method
    std::vector<cv::KeyPoint> keypoints_rgb_d, keypoints_rgb;
	cv::SimpleBlobDetector::Params params;

	params.filterByArea = false;
    params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByConvexity = false;
	params.filterByInertia = false;
    params.minDistBetweenBlobs = 0.01;

	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    

    detector->detect(final_ROI, keypoints_rgb_d);
	cv::drawKeypoints( final_ROI, keypoints_rgb_d, im_with_keypoints,CV_RGB(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
   
    // cv::imshow("final_ROI", final_ROI);
    // cv::waitKey(10);
        
    blobs_for_initialize = keypoints_rgb_d;

    std::vector<cv::Point2f> POI_pts;
    std::vector<cv::Point2f> centers;
    cv::Mat labels;

    for(auto what : keypoints_rgb_d)
    {
        POI_pts.emplace_back(cv::Point2f(what.pt.x, what.pt.y));
    }

    int no_cluster = (POI_pts.size() > LED_no ? LED_no : POI_pts.size());


    // if(no_cluster > LED_no)
    //     no_cluster = LED_no;
    // else
    //     no_cluster = no_cluster;


    if(no_cluster < 4)
    {
        // cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/wrong/final_ROI_gan"+ std::to_string(i)+ ".png", final_ROI);
        // i++;
        return false;
    }

    cv::kmeans(POI_pts, no_cluster, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1), 8, cv::KMEANS_PP_CENTERS, centers);

    pts_2d_detected.clear();
    for(int i = 0; i < centers.size(); i++)
    {
        pts_2d_detected.emplace_back(centers[i].x, centers[i].y);
    }

    frame_input = im_with_keypoints.clone();
    // cout<<"final POI:..."<<pts_2d_detected.size()<<endl;
    if(pts_2d_detected.size() < 4)
        return false;
    else
        return true;
}


/* ================ Init. utilities function below ================ */

bool alan::LedNodelet::LED_tracking_initialize(cv::Mat& frame, cv::Mat depth)
{
    corres_global.clear();

    std::vector<Eigen::Vector2d> pts_2d_detect = LED_extract_POI(frame, depth); 
    std::vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);

    //after above, I got:
    //pointcloud in {c}
    std::vector<double> norm_of_x_points, norm_of_y_points, norm_of_z_points;

    for(auto what :  pts_3d_pcl_detect)
    {
        norm_of_x_points.push_back(what.x());
        norm_of_y_points.push_back(what.y());
        norm_of_z_points.push_back(what.z());
    }

    // cout<<pts_3d_pcl_detect.size()<<endl;
    // cout<<LED_no<<endl;

    if(pts_3d_pcl_detect.size() == LED_no  //we got LED_no
        && calculate_MAD(norm_of_x_points) < MAD_x_threshold //no outlier
        && calculate_MAD(norm_of_y_points) < MAD_y_threshold 
        && calculate_MAD(norm_of_z_points) < MAD_z_threshold) 
    {
        Sophus::SE3d pose;

        int i = 0;

        //hsv detect color feature
        cv::cvtColor(hsv, hsv, CV_RGB2HSV);
        std::vector<bool> g_or_r; //g = true

        std::vector<int> corres_g;
        std::vector<int> corres_r;

        for(int i = 0 ; i < blobs_for_initialize.size(); i++)
        {
            cv::Point hsv_vertice1 = cv::Point(pts_2d_detect[i].x() - 2 * blobs_for_initialize[i].size,
                                               pts_2d_detect[i].y() - 2 * blobs_for_initialize[i].size);
            cv::Point hsv_vertice2 = cv::Point(pts_2d_detect[i].x() + 2 * blobs_for_initialize[i].size,
                                               pts_2d_detect[i].y() + 2 * blobs_for_initialize[i].size);

            cv::Rect letsgethsv(hsv_vertice1, hsv_vertice2);

            cv::Mat ROI(hsv, letsgethsv);

            int size = ROI.cols * ROI.rows;
            
            double accu = 0;

            cv::Vec3b hsv_value;

            for(int i = 0; i < ROI.rows; i++)
            {
                for(int j = 0; j < ROI.cols; j++)
                {
                    hsv_value = ROI.at<cv::Vec3b>(i, j);

                    if(hsv_value[0] == 0)                    
                        size = size - 1;                
                    else
                        accu = accu + hsv_value[0];
                }
            }

            if(accu/size < 100)
                corres_g.push_back(i);
            else   
                corres_r.push_back(i);
        }

        std::vector<int> corres(LED_no);

        if(corres_g.size() != LED_g_no || corres_r.size() != LED_r_no)
        {
            // cout<<"color"<<endl;
            // cout<<corres_g.size()<<endl;
            // cout<<corres_r.size()<<endl;
            return false;
        }

        std::vector<int> final_corres;
        double error_total = INFINITY;

        Eigen::Matrix3d R;
        Eigen::Vector3d t;

        do
        {
            do
            {
                corres.clear();
                for(auto what : corres_g)
                    corres.push_back(what);
                for(auto what : corres_r)
                    corres.push_back(what);

                std::vector<Eigen::Vector2d> pts_2d_detect_temp;   

                for(auto what : corres)
                {
                    pts_2d_detect_temp.push_back(pts_2d_detect[what]);
                }
                // cout<<pts_2d_detect_temp.size()<<endl;
                // cout<<pts_on_body_frame.size()<<endl;
                                                        
                solve_pnp_initial_pose(pts_2d_detect_temp, pts_on_body_frame);
                // cout<<"gan"<<endl;
                
                pose_global_sophus = pose_epnp_sophus;

                double e = get_reprojection_error(                    
                    pts_on_body_frame,
                    pts_2d_detect_temp,
                    pose_global_sophus,
                    false
                );

                if(e < error_total)
                {                    
                    error_total = e;
                    final_corres = corres;
                    
                    if(error_total < 5)
                        break;
                }                        
            } while (next_permutation(corres_r.begin(), corres_r.end()));

        } while(next_permutation(corres_g.begin(), corres_g.end()));

                
        BA_error = error_total;

        correspondence::matchid corres_temp;
        
        std::vector<Eigen::Vector2d> pts_2d_detect_correct_order;
        
        for(auto what : final_corres)
        {
            corres_temp.detected_indices = what;
            corres_temp.detected_ornot = true;
            corres_temp.pts_3d_correspond = pts_3d_pcl_detect[what];            
            corres_temp.pts_2d_correspond = pts_2d_detect[what];

            pts_2d_detect_correct_order.push_back(pts_2d_detect[what]); 

            corres_global.push_back(corres_temp);
        }        

        camOptimize(
            pose_global_sophus, 
            pts_on_body_frame, 
            pts_2d_detect_correct_order,
            BA_error
        );

        detect_no = 6;

        return true;
    }
    else
        return false;
}

bool alan::LedNodelet::reinitialization(std::vector<Eigen::Vector2d> pts_2d_detect, cv::Mat depth)
{
    corres_global.clear();
    std::vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);

    std::vector<double> norm_of_x_points, norm_of_y_points, norm_of_z_points;

    for(auto what :  pts_3d_pcl_detect)
    {
        norm_of_x_points.push_back(what.x());
        norm_of_y_points.push_back(what.y());
        norm_of_z_points.push_back(what.z());
    }

    if(pts_3d_pcl_detect.size() == LED_no  //we got LED_no
        && calculate_MAD(norm_of_x_points) < MAD_x_threshold //no outlier
        && calculate_MAD(norm_of_y_points) < MAD_y_threshold 
        && calculate_MAD(norm_of_z_points) < MAD_z_threshold) 
    {
        Sophus::SE3d pose;

        int i = 0;

        //hsv detect feature
        cv::cvtColor(hsv, hsv, CV_RGB2HSV);
        std::vector<bool> g_or_r; //g = true

        std::vector<int> corres_g;
        std::vector<int> corres_r;

        for(int i = 0 ; i < pts_2d_detect.size(); i++)
        {
            cv::Point hsv_vertice1 = cv::Point(pts_2d_detect[i].x() - 2 * min_blob_size,
                                               pts_2d_detect[i].y() - 2 * min_blob_size);
            cv::Point hsv_vertice2 = cv::Point(pts_2d_detect[i].x() + 2 * min_blob_size,
                                               pts_2d_detect[i].y() + 2 * min_blob_size);

            cv::Rect letsgethsv(hsv_vertice1, hsv_vertice2);

            cv::Mat ROI(hsv, letsgethsv);

            int size = ROI.cols * ROI.rows;
            
            double accu = 0;

            cv::Vec3b hsv_value;

            for(int i = 0; i < ROI.rows; i++)
            {
                for(int j = 0; j < ROI.cols; j++)
                {
                    hsv_value = ROI.at<cv::Vec3b>(i, j);
                    
                    if(hsv_value[0] == 0)                    
                        size = size - 1;                
                    else
                        accu = accu + hsv_value[0];
                }
            }  

            if(accu/size < 100)
                corres_g.push_back(i);
            else   
                corres_r.push_back(i);
        }

        std::vector<int> corres(LED_no);

        if(corres_g.size() != 3 || corres_r.size() != 3)
        {
            return false;
        }

        std::vector<int> final_corres;
        double error_total = INFINITY;

        do
        {
            do
            {
                corres.clear();
                for(auto what : corres_g)
                    corres.push_back(what);
                for(auto what : corres_r)
                    corres.push_back(what);

                std::vector<Eigen::Vector2d> pts_2d_detect_temp;                   

                for(auto what : corres)
                {
                    pts_2d_detect_temp.push_back(pts_2d_detect[what]);
                }
                                                                        
                solve_pnp_initial_pose(pts_2d_detect_temp, pts_on_body_frame);
                
                pose_global_sophus = pose_epnp_sophus;

                double e = get_reprojection_error(
                    pts_on_body_frame,
                    pts_2d_detect_temp,
                    pose_global_sophus,
                    false
                );

                if(e < error_total)
                {                    
                    error_total = e;
                    final_corres = corres;                   

                    if(error_total < 5)
                        break;
                }                        
            } while (next_permutation(corres_r.begin(), corres_r.end()));

        } while(next_permutation(corres_g.begin(), corres_g.end()));

        BA_error = error_total;

        correspondence::matchid corres_temp;
        
        std::vector<Eigen::Vector2d> pts_2d_detect_correct_order;
        
        for(auto what : final_corres)
        {
            corres_temp.detected_indices = what;
            corres_temp.detected_ornot = true;
            corres_temp.pts_3d_correspond = pts_3d_pcl_detect[what];            
            corres_temp.pts_2d_correspond = pts_2d_detect[what];

            pts_2d_detect_correct_order.push_back(pts_2d_detect[what]);                     

            corres_global.push_back(corres_temp);
        }        

        camOptimize(
            pose_global_sophus, 
            pts_on_body_frame, 
            pts_2d_detect_correct_order,
            BA_error
        );
        
        detect_no = 6;

        return true;
    }
    else        
        return false;
}


/* ================ k-means utilities function below ================ */

void alan::LedNodelet::correspondence_search_kmeans(std::vector<Eigen::Vector2d> pts_2d_detected)
{
    std::vector<cv::Point2f> pts;
    Eigen::Vector2d reproject_temp;  

    for(auto what : pts_on_body_frame)
    {

        reproject_temp = reproject_3D_2D(
            what, 
            pose_global_sophus
        );

        pts.emplace_back(cv::Point2f(reproject_temp.x(), reproject_temp.y()));
    }

    // double lala = get_reprojection_error()

    //first emplace back pts_on_body_frame in 2D at this frame
    //in preset order

    for(auto what : pts_2d_detected)
        pts.emplace_back(cv::Point2f(what.x(), what.y()));
    
    std::vector<cv::Point2f> centers;
    cv::Mat labels;

    cv::kmeans(pts, LED_no, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1), 8, cv::KMEANS_PP_CENTERS, centers);

    int error_count = 0;

    for(int i = 0; i < LED_no; i++)
    {
        int clusterIdx = labels.at<int>(i);
        for(int k = 0; k < pts.size(); k++)
        {
            if(i == k)
                continue;
            int clusterIdx_detected = labels.at<int>(k);
            if(clusterIdx == clusterIdx_detected)
            {
                if(k < LED_no)
                {
                    error_count++;
                    break;
                }  
                else
                {
                    corres_global[i].detected_ornot = true;
                    corres_global[i].pts_2d_correspond = pts_2d_detected[k - LED_no];
                }                                                    

            }
        }
    }
}


/* ================ Outlier Rejection utilities function below ================ */
    /* in outlier rejection, we first calculate the MAD (mean average deviation)
    to see whether there exists some outlier or not.
    then, we try to do clustering with k-means algorithm.
    as we are processing 3D points, at most time, 
    the LED blobs should be close enough, 
    while others being at some other coordinates that are pretty far away
    hence, we set the clustering no. as 2.
    we then calcullate the distance between the centroid of the cluster to the
    center at previous time step(pcl_center_point_wo_outlier_previous)
    and determine which cluster is the one that we want */

void alan::LedNodelet::reject_outlier(std::vector<Eigen::Vector2d>& pts_2d_detect, cv::Mat depth)
{
    std::vector<Eigen::Vector3d> pts_3d_detect = pointcloud_generate(pts_2d_detect, depth);
    //what is this for?
    //to get 3d coordinates in body frame, so that 
    //outlier rejection could be performed
    int n = pts_3d_detect.size();

    std::vector<cv::Point3f> pts;
    std::vector<double> norm_of_x_points;
    std::vector<double> norm_of_y_points;
    std::vector<double> norm_of_z_points;

    for(auto what :  pts_3d_detect)
    {
        norm_of_x_points.push_back(what.x());
        norm_of_y_points.push_back(what.y());
        norm_of_z_points.push_back(what.z());

        pts.push_back(cv::Point3f(what.x(), what.y(), what.z()));
    }

    cv::Mat labels;
    std::vector<cv::Point3f> centers;

    
    if(calculate_MAD(norm_of_x_points) > MAD_x_threshold  
        || calculate_MAD(norm_of_y_points) > MAD_y_threshold
        || calculate_MAD(norm_of_z_points) > MAD_z_threshold)
    {   
        // cout<<"got some rejection to do"<<endl;
        cv::kmeans(pts, 2, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1), 8, cv::KMEANS_PP_CENTERS, centers);

        double d0 = cv::norm(pcl_center_point_wo_outlier_previous - centers[0]);
        double d1 = cv::norm(pcl_center_point_wo_outlier_previous - centers[1]);

        std::vector<Eigen::Vector2d> pts_2d_result;
        std::vector<Eigen::Vector3d> pts_3d_result;

        if(d0 < d1) //then get index with 0
        {           
            for(int i = 0; i < labels.rows; i++)
            {
                if(labels.at<int>(0,i) == 0)
                {
                    pts_2d_result.push_back(pts_2d_detect[i]);
                    pts_3d_result.push_back(pts_3d_detect[i]); 
                }                    
            }            
            pcl_center_point_wo_outlier_previous = centers[0];
        }
        else
        {
            for(int i = 0; i < labels.rows; i++)
            {

                if(labels.at<int>(0,i) == 1)
                {                    
                    pts_2d_result.push_back(pts_2d_detect[i]);
                    pts_3d_result.push_back(pts_3d_detect[i]);                    
                }
            }
            pcl_center_point_wo_outlier_previous = centers[1];
        }
            
        pts_2d_detect.clear();
        pts_2d_detect = pts_2d_result;

        pts_3d_detect.clear();
        pts_3d_detect = pts_3d_result;

    }
    else
    {
        cv::Mat temp;
        
        cv::reduce(pts, temp, 01, CV_REDUCE_AVG);
        pcl_center_point_wo_outlier_previous = cv::Point3f(temp.at<float>(0,0), temp.at<float>(0,1), temp.at<float>(0,2));

    }

    
    led_3d_posi_in_camera_frame_depth = Eigen::Vector3d(
        pcl_center_point_wo_outlier_previous.x,
        pcl_center_point_wo_outlier_previous.y,
        pcl_center_point_wo_outlier_previous.z
    );

    // cout<<"reject outlier"<<endl;
    // cout<<led_3d_posi_in_camera_frame_depth.z()<<endl;


}

inline double alan::LedNodelet::calculate_MAD(std::vector<double> norm_of_points)
{
    int n = norm_of_points.size();
    double mean = 0, delta_sum = 0, MAD;
    if(n != 0)
    {
        mean = accumulate(norm_of_points.begin(), norm_of_points.end(), 0.0) / n;
        for(int i = 0; i < n; i++)
            delta_sum = delta_sum + abs(norm_of_points[i] - mean);        
        MAD = delta_sum / n;
    }   

    return MAD;
}

/* ================ UI utilities function below ================ */

void alan::LedNodelet::set_image_to_publish(double freq, const sensor_msgs::CompressedImageConstPtr & rgbmsg)
{    
    char hz[40];
    char fps[10] = " fps";
    sprintf(hz, "%.2f", freq);
    strcat(hz, fps);

    char BA[40] = "BA: ";
    char BA_error_display[10];
    sprintf(BA_error_display, "%.2f", BA_error);
    strcat(BA, BA_error_display);

    char depth[40] = "DPTH: ";
    char depth_display[10];
    sprintf(depth_display, "%.2f", depth_avg_of_all);
    strcat(depth, depth_display);
    
    cv::putText(display, hz, cv::Point(20,40), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));  
    cv::putText(display, std::to_string(detect_no), cv::Point(720,460), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));
    cv::putText(display, BA, cv::Point(720,60), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));
    cv::putText(display, depth, cv::Point(20,460), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));

    cv::Mat imageoutput = display.clone();
    cv_bridge::CvImage for_visual;
    for_visual.header = rgbmsg->header;
    for_visual.encoding = sensor_msgs::image_encodings::BGR8;
    for_visual.image = imageoutput;
    this->pubimage.publish(for_visual.toImageMsg());


    cv_bridge::CvImage for_visual_input;
    for_visual_input.header = rgbmsg->header;
    for_visual_input.encoding = sensor_msgs::image_encodings::BGR8;
    for_visual_input.image = frame_input;
    this->pubimage_input.publish(for_visual_input.toImageMsg());   

    // if(BA_error > 100)
    // {
    //     cv::imwrite("/home/patty/alan_ws/haha.jpg", display);
    //     // cv::imwrite("/home/patty/alan_ws/lala.jpg", frame_input);
    //     detect_no = 0;            
    //     ros::shutdown();
    // }
    // cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/in_camera_frame/"+ std::to_string(i)+ ".png", display); 
    // i++;
}

void alan::LedNodelet::log(double ms)
{
    alan_state_estimation::alan_log logdata_entry_led;
    
    logdata_entry_led.px = pose_led_inWorld_SE3.translation().x();
    logdata_entry_led.py = pose_led_inWorld_SE3.translation().y();
    logdata_entry_led.pz = pose_led_inWorld_SE3.translation().z();

    logdata_entry_led.set_px = uav_stpt_msg.pose.position.x;
    logdata_entry_led.set_py = uav_stpt_msg.pose.position.y;
    logdata_entry_led.set_pz = uav_stpt_msg.pose.position.z;
    
    Eigen::Vector3d rpy = q2rpy(
        Eigen::Quaterniond(pose_led_inWorld_SE3.rotationMatrix())
    );

    logdata_entry_led.roll  = rpy(0);
    logdata_entry_led.pitch = rpy(1);
    logdata_entry_led.yaw   = rpy(2);

    Eigen::AngleAxisd angle_axis_led = Eigen::AngleAxisd(pose_led_inWorld_SE3.rotationMatrix());
    logdata_entry_led.orientation = angle_axis_led.angle();

    logdata_entry_led.ms = ms;
    logdata_entry_led.depth = abs(
        sqrt(
            pow(
                pose_cam_inWorld_SE3.translation().x() - pose_uav_inWorld_SE3.translation().x(),
                2
            ) +
            pow(pose_cam_inWorld_SE3.translation().y() - pose_uav_inWorld_SE3.translation().y(),
                2
            ) +
            pow(pose_cam_inWorld_SE3.translation().z() - pose_uav_inWorld_SE3.translation().z(),
                2
            )
        )        
    );

    logdata_entry_led.header.stamp = led_pose_header.stamp;

    record_led_pub.publish(logdata_entry_led);

    ///////////////////////////////////////////////////////////

    alan_state_estimation::alan_log logdata_entry_uav;
    
    logdata_entry_uav.px = pose_uav_inWorld_SE3.translation().x();
    logdata_entry_uav.py = pose_uav_inWorld_SE3.translation().y();
    logdata_entry_uav.pz = pose_uav_inWorld_SE3.translation().z();
    
    rpy = q2rpy(
        Eigen::Quaterniond(pose_uav_inWorld_SE3.rotationMatrix())
    );

    logdata_entry_uav.roll  = rpy(0);
    logdata_entry_uav.pitch = rpy(1);
    logdata_entry_uav.yaw   = rpy(2);

    Eigen::AngleAxisd angle_axis_uav = Eigen::AngleAxisd(pose_uav_inWorld_SE3.rotationMatrix());
    logdata_entry_uav.orientation = angle_axis_uav.angle();

    logdata_entry_uav.header.stamp = led_pose_header.stamp;

    record_uav_pub.publish(logdata_entry_uav);

    // std::cout<<"orientation: "<<abs(logdata_entry_led.orientation - logdata_entry_uav.orientation)<<std::endl;
    // std::ofstream save("/home/patty/alan_ws/src/alan/alan_state_estimation/src/temp/lala.txt", std::ios::app);
    // save<<abs(logdata_entry_led.orientation - logdata_entry_uav.orientation)<<std::endl;
}

void alan::LedNodelet::terminal_msg_display(double hz)
{
    std::string LED_terminal_display = "DETECT_no: " + std::to_string(detect_no);

    std::ostringstream out1;
    out1.precision(2);
    out1<<std::fixed<<BA_error;
    std::string BA_terminal_display = " || BA_ERROR: " + out1.str();

    std::ostringstream out2;
    out2.precision(2);
    out2<<std::fixed<<depth_avg_of_all;
    std::string depth_terminal_display = " || depth: " + out2.str();

    std::ostringstream out3;
    out3.precision(2);
    out3<<std::fixed<<hz;
    std::string hz_terminal_display = " || hz: " + out3.str();

    std::string final_msg = LED_terminal_display 
        + BA_terminal_display 
        + depth_terminal_display
        + hz_terminal_display;

    std::string LED_tracker_status_display;

    if(LED_tracker_initiated_or_tracked)
    {
        final_msg = "LED GOOD || " + final_msg;
        ROS_GREEN_STREAM(final_msg);
    }
    else
    {
        final_msg = "LED BAD! || " + final_msg;
        ROS_RED_STREAM(final_msg);
        error_no ++;
    }
    std::cout<<"fail: "<< error_no<<" / "<<total_no<<std::endl;
}
