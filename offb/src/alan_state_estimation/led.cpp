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

    pose_w_LED_pnp(frame, depth);
    
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

void alan_pose_estimation::LedNodelet::pose_w_LED_pnp(cv::Mat& frame, cv::Mat depth)
{
    //vector<Eigen::Vector3d> pts_3d_LED_camera =    
    // cv::threshold(frame, frame, )
    vector<Eigen::Vector2d> pts_2d_detect = LED_extract_POI(frame, depth);

    //reject outlier

    //

}

void alan_pose_estimation::LedNodelet::LED_tracking_initialize(cv::Mat& frame, cv::Mat depth)
{
    vector<Eigen::Vector2d> pts_2d_detect = LED_extract_POI(frame, depth);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);
    
    // cout<<"before outlier rejection:"<<pts_3d_pcl_detect.size()<<endl;
    // reject_outlier(pts_3d_pcl_detect);
    // cout<<"after outlier rejection: "<<pts_3d_pcl_detect.size()<<endl;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr alan_pose_estimation::LedNodelet::pointcloud_generate(vector<Eigen::Vector2d> pts_2d_detected, cv::Mat depthimage)
{
    //get 9 pixels around the point of interest

    int no_pixels = 25;
    int POI_width = (sqrt(no_pixels) - 1 ) / 2;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointclouds;

    int x_pixel, y_pixel;
    Eigen::Vector3d temp;
    pcl::PointXYZ temp_pcl;

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

        temp_pcl.x = temp.x();
        temp_pcl.y = temp.y();
        temp_pcl.z = temp.z();

        pointclouds->push_back(temp_pcl);
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

    //contour method

    // cv::Mat gray;
    // cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    // cv::blur(gray, gray, cv::Size(3,3));
    
    // cv::Mat canny;
    // cv::Canny(gray, canny, 100, 200);
    // vector<vector<cv::Point> > contours;
    // vector<cv::Vec4i> hierarchy;
    // cv::findContours(canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    // cv::Mat drawing = cv::Mat::zeros(canny.size(), CV_8UC3);
        
    // for( size_t i = 0; i< contours.size(); i++ )
    // {
    //     drawContours( drawing, contours, (int)i, CV_RGB(255, 255, 0), 0.5, cv::LINE_8, hierarchy, 0 );
    // }
    
    // cv::cvtColor(drawing, drawing, cv::COLOR_RGB2GRAY);
    // cv::threshold(drawing, drawing, 100, 255, cv::THRESH_BINARY);
    
    // cv::Mat final;// = drawing;

    // cv::bitwise_and(depth_mask_src, drawing, final);

    // cv::imshow("led first", frame);
    // cv::waitKey(1000/60);

    // cv::cvtColor(frame, frame, cv::COLOR_RGB2GRAY);
    // cv::threshold(frame, frame, 240, 255, cv::THRESH_BINARY);
    // cv::imshow("test", frame);
    // cv::waitKey(20);

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
    

    cv::String blob_size = to_string(keypoints_rgb_d.size());
    cv::putText(im_with_keypoints, blob_size, cv::Point(20,40), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));

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


#endif

