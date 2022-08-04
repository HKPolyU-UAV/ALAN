#ifndef LED_H
#define LED_H

#include "include/led.h"

void alan_pose_estimation::LedNodelet::camera_callback(const sensor_msgs::CompressedImageConstPtr & rgbmsg, const sensor_msgs::ImageConstPtr & depthmsg)
{
    cv_bridge::CvImageConstPtr depth_ptr;
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
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    if(i == 0)
    {
        i++;
        temp = ros::WallTime::now().toSec();
    }

    ROS_INFO("ROS Timestamp: %d", ros::WallTime::now().toSec() - temp);
    // cout<<ros::Time::now().toSec() - temp<<endl;;

    double t1 = ros::Time::now().toSec(); 

    // pose_w_LED_icp(frame, depth);
    
    double t2 = ros::Time::now().toSec();
    
    char hz[40];
    char fps[5] = " fps";
    sprintf(hz, "%.2f", 1 / (t2 - t1));
    strcat(hz, fps);
    // cv::putText(frame, hz, cv::Point(20,40), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));  

    // cout<<hz<<endl;

    // cv::imshow("led first", frame);
    // cv::waitKey(1000/60);

} 

void alan_pose_estimation::LedNodelet::pose_w_LED_icp(cv::Mat& frame, cv::Mat depth)
{
    //vector<Eigen::Vector3d> pts_3d_LED_camera =    
    // cv::threshold(frame, frame, )

    if(!LED_tracker_initiated)        
        LED_tracker_initiated = LED_tracking_initialize(frame, depth);    
    else
    {
        vector<Eigen::Vector2d> pts_2d_detect = LED_extract_POI(frame, depth);
        vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);
        reject_outlier(pts_3d_pcl_detect, pts_2d_detect);

        //filter out the pts in body frame that was not detected this time
        correspondence::matchid tracking_result = track(pts_3d_pcl_detect, pts_on_body_frame);
        vector<Eigen::Vector3d> pts_on_body_frame_for_pose 
            = filter_out_nondetected_body_points(pts_on_body_frame, tracking_result);
        
        Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
        Eigen::Vector3d t = Eigen::Vector3d::Zero();

        solveicp_svd(pts_3d_pcl_detect, pts_on_body_frame_for_pose, R, t);
        // hungarian.solution(pts_on_body_frame_normalized, pts_3d_pcl_detect_normalized);

        Sophus::SE3d pose;
        pose = Sophus::SE3d(R,t);        

        double e = 0;
        Eigen::Vector2d reproject, error; 

        for(int i = 0 ; i < pts_on_body_frame_for_pose.size(); i++)
        {
            reproject = reproject_3D_2D(pts_on_body_frame_for_pose[i], pose);  
            error = pts_2d_detect[i] - reproject;
            e = e + error.norm();
        }

        cout<<"error: "<<e<<endl;

        if(e > 20.0)
        {
            cout<<"use pnp instead"<<endl;
            use_pnp_instead(frame, pts_2d_detect, pts_3d_pcl_detect, pose);
        }

        // optimize(pose, body_frame_pts, pts_2d_detect);//pose, body_frame_pts, pts_2d_detect

        for(auto what : pts_on_body_frame_for_pose)
        {
            Eigen::Vector2d reproject = reproject_3D_2D(what, pose);            
            cv::circle(frame, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(0,255,0),-1);
        }

    }

    vector<Eigen::Vector2d> pts_2d_detect = LED_extract_POI(frame, depth);
    vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);

    // for(auto what : pts_3d_pcl_detect)
    // {
    //     cout<<what<<endl<<endl;;
    // }

    // cout<<pts_3d_pcl_detect.size()<<endl;

    //reject outlier

    //

}

void alan_pose_estimation::LedNodelet::solveicp_svd(vector<Eigen::Vector3d> pts_3d_camera, vector<Eigen::Vector3d> pts_3d_body, Eigen::Matrix3d& R, Eigen::Vector3d& t)
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


Eigen::Vector3d alan_pose_estimation::LedNodelet::get_CoM(vector<Eigen::Vector3d> pts_3d)
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

void alan_pose_estimation::LedNodelet::use_pnp_instead(cv::Mat frame, vector<Eigen::Vector2d> pts_2d_detect, vector<Eigen::Vector3d> pts_3d_detect, Sophus::SE3d& pose)
{
    Eigen::Vector3d t;
    Eigen::Matrix3d R;

    get_initial_pose(pts_2d_detect, pts_3d_detect, R, t);
    
    //generate noise to validate BA
    Sophus::SE3d pose_;
    pose_ = Sophus::SE3d(R,t);

    optimize(pose_, pts_3d_detect, pts_2d_detect);

    pose = pose_;

}

inline Eigen::Vector2d alan_pose_estimation::LedNodelet::reproject_3D_2D(Eigen::Vector3d P, Sophus::SE3d pose)
{
    Eigen::Vector3d result;

    Eigen::Matrix3d R = pose.rotationMatrix();
    Eigen::Vector3d t = pose.translation();

    result = cameraMat * (R * P + t); 

    Eigen::Vector2d result2d;
    

    result2d <<
        result(0)/result(2), 
        result(1)/result(2);
    
    return result2d;
}


void alan_pose_estimation::LedNodelet::get_initial_pose(vector<Eigen::Vector2d> pts_2d, vector<Eigen::Vector3d> body_frame_pts, Eigen::Matrix3d& R, Eigen::Vector3d& t)
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

void alan_pose_estimation::LedNodelet::optimize(Sophus::SE3d& pose, vector<Eigen::Vector3d> pts_3d_exists, vector<Eigen::Vector2d> pts_2d_detected)
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

void alan_pose_estimation::LedNodelet::solveJacobian(Eigen::Matrix<double, 2, 6>& Jacob, Sophus::SE3d pose, Eigen::Vector3d point_3d)
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


vector<Eigen::Vector2d> alan_pose_estimation::LedNodelet::LED_extract_POI(cv::Mat& frame, cv::Mat depth)
{
    // frame.resize(frame.rows * 2, frame.cols * 2, cv::INTER_LINEAR);
    // cv::resize(frame, frame, cv::Size(frame.cols * 2, frame.rows * 2), 0, 0, cv::INTER_LINEAR);
    // cv::resize(depth, depth, cv::Size(depth.cols * 2, depth.rows * 2), 0, 0, cv::INTER_LINEAR);
    // cout<<frame.size<<endl;
    // depth.resize(depth.rows * 2, depth.cols * 2, cv::INTER_LINEAR);
    // cv::Mat hsv = frame.clone();
    // cv::cvtColor(hsv, hsv, cv::COLOR_RGB2HSV);
    // cv::imshow("hsv", hsv);
    // cv::waitKey(20);


    cv::Mat depth_mask_src = depth.clone(), depth_mask_dst1, depth_mask_dst2;

    cv::threshold(depth_mask_src, depth_mask_dst1, LANDING_DISTANCE * 1000, 50000, cv::THRESH_BINARY_INV);

    cv::threshold(depth_mask_src, depth_mask_dst2, 0.5, 50000, cv::THRESH_BINARY);

    cv::bitwise_and(depth_mask_dst1, depth_mask_dst2, depth_mask_src);
    
    depth_mask_src.convertTo(depth_mask_src, CV_8U);

    
    cv::Mat d_img = depth;
    int size=d_img.cols*d_img.rows;
    for(int i=0; i<size; i++)
    {
        if(isnan(d_img.at<ushort>(i)))
        {
            d_img.at<ushort>(i)=0;
        }
        if(d_img.at<ushort>(i)>10000||d_img.at<ushort>(i)<100)
        {
            d_img.at<ushort>(i)=0;
        }
    }
    cv::Mat adjMap;
    d_img.convertTo(adjMap,CV_8UC1, 255 / (10000.0), 0);
    cv::applyColorMap(adjMap, adjMap, cv::COLORMAP_RAINBOW);
    for(int i=0; i<size; i++)
    {
        if(d_img.at<ushort>(i)==0)
        {
            cv::Vec3b color = adjMap.at<cv::Vec3b>(i);
            color[0]=255;
            color[1]=255;
            color[2]=255;
            adjMap.at<cv::Vec3b>(i)=color;
        }
    }
    cv::imshow("depppth", adjMap);
    cv::waitKey(20);    

    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    cv::threshold(frame, frame, 200, 255, cv::THRESH_BINARY);
    

    // Blob method
    vector<cv::KeyPoint> keypoints_rgb_d, keypoints_rgb;
	cv::SimpleBlobDetector::Params params;

	params.filterByArea = false;
    params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByConvexity = false;
	params.filterByInertia = false;
    params.minDistBetweenBlobs = 1;

	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);   
    cv::Mat im_with_keypoints;

    //detect frame before filter out background
	// detector->detect( frame, keypoints_rgb);
	// cv::drawKeypoints( frame, keypoints_rgb, im_with_keypoints,CV_RGB(255,0,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    // cv::imshow("keypoints", im_with_keypoints );
	// cv::waitKey(20);

    //detect frame after filter out background
    cv::bitwise_and(depth_mask_src, frame, frame); //filter out with depth information

    double dilation_size = 2;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       cv::Point( dilation_size, dilation_size ) );
    cv::dilate(frame, frame,  element);// enlarge

    cv::imshow("keypoints", frame );
	cv::waitKey(20);
    
    detector->detect(frame, keypoints_rgb_d);
	cv::drawKeypoints( frame, keypoints_rgb_d, im_with_keypoints,CV_RGB(255,0,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    // cout<<keypoints_rgb_d.size()<<endl;
    

    // cv::String blob_size = to_string(keypoints_rgb_d.size());
    // cv::putText(im_with_keypoints, blob_size, cv::Point(20,40), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));

	// Show blobs
	cv::imshow("keypoints2", im_with_keypoints );
	cv::waitKey(20);

    vector<Eigen::Vector2d> POI;
    for(auto what : keypoints_rgb_d)
    {
        POI.push_back(Eigen::Vector2d(what.pt.x, what.pt.y));
    }

    return POI;
}


vector<Eigen::Vector3d> alan_pose_estimation::LedNodelet::pointcloud_generate(vector<Eigen::Vector2d> pts_2d_detected, cv::Mat depthimage)
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


bool alan_pose_estimation::LedNodelet::LED_tracking_initialize(cv::Mat& frame, cv::Mat depth)
{
    vector<Eigen::Vector2d> pts_2d_detect = LED_extract_POI(frame, depth);
    vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);
    //after above, I got:
    //pointcloud in {c}
    vector<double> norm_of_points;

    for(auto what :  pts_3d_pcl_detect)
        norm_of_points.push_back(what.norm());
    
    

    if(pts_3d_pcl_detect.size() == LED_no && calculate_MAD(norm_of_points) < MAD_threshold)
    {
        vector<Eigen::Vector3d> pts_3d_pcl_detect_normalized = normalization_2d(pts_3d_pcl_detect, 0, 1);
        LED_v_Detected = hungarian.solution(pts_on_body_frame_normalized, pts_3d_pcl_detect_normalized);
        pts_detected_in_corres_order = sort_the_points_in_corres_order(pts_3d_pcl_detect, hungarian.id_match);

        return true;
    }
    else
        return false;
    

    
    
    //now I match it with the points in {b}

    //now what?
    //save to where?

}

void alan_pose_estimation::LedNodelet::correspondence_search(vector<Eigen::Vector3d> pts_on_body_frame, vector<Eigen::Vector3d> pts_detected)
{
    // cv::kmeans()

}


vector<Eigen::Vector3d> alan_pose_estimation::LedNodelet::normalization_2d(vector<Eigen::Vector3d> v_pts, int i_x, int i_y)
{
    double x_min = INFINITY, y_min = INFINITY;    
    int i_x_min_final, i_y_min_final;
    
    double x_max = -INFINITY, y_max = -INFINITY;    
    int i_x_max_final, i_y_max_final;

    for(int i = 0; i < v_pts.size(); i++)
    {
        if(v_pts[i](i_x) < x_min) //get x min
        {
            i_x_min_final = i;
            x_min = v_pts[i](i_x);
        }

        if(v_pts[i](i_y) < y_min) //get y min
        {
            i_y_min_final = i;
            y_min = v_pts[i](i_y);
        }

        if(v_pts[i](i_x) > x_max) //get x max
        {
            i_x_max_final = i;
            x_max = v_pts[i](i_x);
        }

        if(v_pts[i](i_y) > y_max) //get y max
        {
            i_y_max_final = i;
            y_max = v_pts[i](i_y);
        }        

    }

    // cout << x_min << " " << i_x_min_final << endl;
    // cout << y_min << " " << i_y_min_final << endl;
    // cout << x_max << " " << i_x_max_final << endl;
    // cout << y_max << " " << i_y_max_final << endl;

    double delta_x, delta_y;

    delta_x = x_max - x_min;
    delta_y = y_max - y_min;

    vector<Eigen::Vector3d> normalized_v_pts;
    double x_temp, y_temp;
    Eigen::Vector3d v_temp;

    for(int i = 0; i < v_pts.size(); i++)
    {
        x_temp = (v_pts[i](i_x) - x_min) / delta_x;
        y_temp = (v_pts[i](i_y) - y_min) / delta_y;
        
        v_temp.x() = x_temp * 100;
        v_temp.y() = y_temp * 100;
        v_temp.z() = 0;

        normalized_v_pts.push_back(v_temp);
    }
    
    return normalized_v_pts;
}

vector<Eigen::Vector3d> alan_pose_estimation::LedNodelet::sort_the_points_in_corres_order(vector<Eigen::Vector3d> pts, vector<correspondence::matchid> corres)
{
    vector<Eigen::Vector3d> pts_return;
    Eigen::Vector3d pt_temp;
    for(auto what :  corres)
    {
        pt_temp = pts[what.detected_indices];
    }

}

//outlier rejection
void alan_pose_estimation::LedNodelet::reject_outlier(vector<Eigen::Vector3d>& pts_3d_detect, vector<Eigen::Vector2d>& pts_2d_detect)
{
    int n = pts_3d_detect.size();

    vector<cv::Point3f> pts;
    vector<double> norm_of_points;
    for(auto what :  pts_3d_detect)
    {
        norm_of_points.push_back(what.norm());
        pts.push_back(cv::Point3f(what.x(), what.y(), what.z()));
    }
    
    // double MAD = calculate_MAD(norm_of_points);
    // cout<<"MAD: "<<MAD<<endl;

    cv::Mat labels;
    std::vector<cv::Point3f> centers;


    if(calculate_MAD(norm_of_points) > MAD_threshold)
    {
        // ind++;
        // cv::imwrite("/home/patty/alan_ws/src/alan/offb/src/alan_state_estimation/test/outliers/outlier" + to_string(ind) + ".png", frame);
        
        cv::kmeans(pts, 2, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1), 8, cv::KMEANS_PP_CENTERS, centers);

        // cout<<"center size: "<<centers.size()<<endl;

        // cout<<norm(point_wo_outlier_previous)<<endl;;
        // cout<<norm(centers[0])<<endl;
        // cout<<norm(centers[1])<<endl;

        double d0 = cv::norm(point_wo_outlier_previous - centers[1]);
        double d1 = cv::norm(point_wo_outlier_previous - centers[1]);
        
        cout<<"distance"<<endl;
        cout<<d0<<endl;
        cout<<d1<<endl;

        if(d0 < d1)
        {
            cout<<"cluster 0 is the right one"<<endl;
        }
        else
            cout<<"cluster 1 is the right one"<<endl;        
        
    }
    else
    {
        cv::Mat temp;
        
        cv::reduce(pts, temp, 01, CV_REDUCE_AVG);
        point_wo_outlier_previous = cv::Point3f(temp.at<float>(0,0), temp.at<float>(0,1), temp.at<float>(0,2));

        // cout<<temp(1)<<endl;
        // cout<<temp(2)<<endl;
    }
}

inline double alan_pose_estimation::LedNodelet::calculate_MAD(vector<double> norm_of_points)
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

void* alan_pose_estimation::LedNodelet::PubMainLoop(void* tmp)
{
    LedNodelet* pub = (LedNodelet*) tmp;

    ros::Rate loop_rate(50);
    while (ros::ok()) 
    {
        // ROS_INFO("%d,publish!", num++);
        // pub->pubpose.publish(pub->pose_estimated);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

#endif