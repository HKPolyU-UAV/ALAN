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

    double t1 = ros::Time::now().toSec(); 

    pose_w_LED_icp(frame, depth);
    
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

    // if(!LED_tracker_initiated)
    // {
    //     LED_tracking_initialize(frame, depth);
    //     LED_tracker_initiated = true;
    // }
    // else
    // {
    //     vector<Eigen::Vector2d> pts_2d_detect = LED_extract_POI(frame, depth);
    //     vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);

    //     // hungarian.solution(pts_on_body_frame_normalized, pts_3d_pcl_detect_normalized);

    // }
    
    vector<Eigen::Vector2d> pts_2d_detect = LED_extract_POI(frame, depth);
    vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);

    // for(auto what : pts_3d_pcl_detect)
    // {
    //     cout<<what<<endl<<endl;;
    // }

    reject_outlier(pts_3d_pcl_detect);
    cout<<pts_3d_pcl_detect.size()<<endl;

    //reject outlier

    //

}

void alan_pose_estimation::LedNodelet::reject_outlier(vector<Eigen::Vector3d>& pts_3d_detect)
{
    vector<double> norm_of_points;

    for(auto what :  pts_3d_detect)
    {
        norm_of_points.push_back(what.norm());
    }

    cv::kmeans()

    vector<int> v_index(norm_of_points.size());
    
    iota(v_index.begin(),v_index.end(),0); //Initializing
    sort(v_index.begin(),v_index.end(), [&](int i,int j) {return norm_of_points[i]< norm_of_points[j];} );//this is lambda

    vector<double> norm_of_points_sorted;


    for(auto what : v_index)
    {
        cout<<norm_of_points[what]<<endl;
        norm_of_points_sorted.push_back(norm_of_points[what]);
        cout<<endl;
    }
    // sort(norm_of_points.begin(), norm_of_points.end());

    // for(auto what : v_index)
    
    int index_0 = 0;
    int index_100 = norm_of_points_sorted.size();

    int index_50 = ((index_100 - index_0 + 1) + 1) / 2 - 1;
    int index_25 = ((index_50 - index_0 + 1) + 1) / 2 - 1;
    int index_75 = ((index_100 - index_50 + 1) + 1) / 2 - 1;

    double IQR = norm_of_points_sorted[index_75] - norm_of_points_sorted[index_25];

    double lower = norm_of_points_sorted[index_25] - 1.5 * IQR, 
           upper = norm_of_points_sorted[index_75] + 1.5 * IQR;

    cout<<"lower: "<<lower<<endl;
    cout<<"upper: "<<upper<<endl;

    for(auto what : v_index)
    {
        if(norm_of_points[what] < lower || norm_of_points[what] > upper)
            pts_3d_detect.erase(pts_3d_detect.begin() + what);        
    }
}



void alan_pose_estimation::LedNodelet::LED_tracking_initialize(cv::Mat& frame, cv::Mat depth)
{
    vector<Eigen::Vector2d> pts_2d_detect = LED_extract_POI(frame, depth);
    vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);
    //after above, I got:
    //pointcloud in {c}

    vector<Eigen::Vector3d> pts_3d_pcl_detect_normalized = normalization_2d(pts_3d_pcl_detect, 0, 1);
    LED_v_Detected = hungarian.solution(pts_on_body_frame_normalized, pts_3d_pcl_detect_normalized);
    pts_detected_in_corres_order = sort_the_points_in_corres_order(pts_3d_pcl_detect, hungarian.id_match);
    //now I match it with the points in {b}

    //now what?
    //save to where?


}

void alan_pose_estimation::LedNodelet::correspondence_search(vector<Eigen::Vector3d> pts_on_body_frame, vector<Eigen::Vector3d> pts_detected)
{
    // cv::kmeans()

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

Eigen::Matrix4d alan_pose_estimation::LedNodelet::icp_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_body, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera)
{
    //get SE(3) from  body to camera, i.e., camera = T * body
    //get T!

    //cloud_in = cloud_body
    //cloud_out = cloud_camera

    double t1 = ros::Time::now().toSec();


    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>::Ptr rej_ransac(new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>);

    icp.setInputSource(cloud_body);
    icp.setInputTarget(cloud_camera);
    icp.addCorrespondenceRejector(rej_ransac);
    
    pcl::PointCloud<pcl::PointXYZ> Final;
    
    icp.align(Final);

    // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    // icp.getFitnessScore() << std::endl;
    cout << icp.getFinalTransformation() << endl;  

    pcl::PointCloud<pcl::PointXYZ>::Ptr results (new pcl::PointCloud<pcl::PointXYZ>);
    *results = Final;

    pcl::CorrespondencesPtr corresps(new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
    est.setInputSource (results); //
    est.setInputTarget (cloud_camera);
    est.determineCorrespondences (*corresps, 1.0);

    for (auto& what : *corresps)
    {
        cout<<what.index_match<<endl;               
    }


    pcl::IndicesPtr match_id = icp.getIndices();


    // for (auto& what : Final)
    // {
    //     cout<<what<<endl;;
    // }

    Eigen::Matrix4d icp_pose =  icp.getFinalTransformation().cast<double>();

    double t2 = ros::Time::now().toSec();
    cout << endl << "Hz: " << 1 / (t2-t1) <<endl;    

    return icp_pose;
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

void alan_pose_estimation::LedNodelet::solveicp_svd(vector<Eigen::Vector3d> pts_3d_camera, vector<Eigen::Vector3d> pts_3d_body, Eigen::Matrix3d& R, Eigen::Vector3d& t, vector<correspondence::matchid> corres)
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



#endif

