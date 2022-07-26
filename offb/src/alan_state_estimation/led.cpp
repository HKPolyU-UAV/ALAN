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


    double t2 = ros::Time::now().toSec();
    pcl::IndicesPtr match_id = icp.getIndices();


    // for (auto& what : Final)
    // {
    //     cout<<what<<endl;;
    // }
    cout << endl << "Hz: " << 1 / (t2-t1) <<endl;

    

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
    cv::imshow("keypoints", frame );
	cv::waitKey(20);

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

vector<alan_pose_estimation::Match> alan_pose_estimation::LedNodelet::solution(vector<cv::Point> measured, vector<cv::Point> previous )
{
    id_match.clear();
    cv::Point temp;
    vector<cv::Point> detected_pts, previous_pts;

    for(auto o : measured)
    {
        temp = cv::Point (o.x, o.y);
        detected_pts.push_back(temp);
    }

    for(auto o : previous)
    {
        temp = cv::Point (o.x, o.y);
        previous_pts.push_back(temp);
    }

    cost_generate(detected_pts, previous_pts);
    copy = cost;

    bool done = false;
    step = 1;

    while (!done)
    {
        //cout << endl << step << endl << endl;
        switch (step)
        {
            case 1:
                stp1(step);
                break;
            case 2:
                stp2(step);
                break;
            case 3:
                stp3(step);
                break;
            case 4:
                stp4(step);
                break;
            case 5:
                stp5(step);
                break;
            case 6:
                stp6(step);
                break;
            case 7:
                stp7();
                done = true;
    //            cout<<"bye"<<endl;
            break;
        }
    }

    return id_match;
}

void alan_pose_estimation::LedNodelet::cost_generate(vector<cv::Point> detected, vector<cv::Point> previous)
{
    if(detected.size() == previous.size())
    {
        cost.setZero(detected.size(), previous.size());
        mask.setZero(detected.size(), previous.size());
        cover_row = vector<int>(detected.size(),0);
        cover_col = vector<int>(detected.size(),0);
        path.setZero(detected.size()*2,2);
    }
    else if (detected.size() < previous.size())
    {
        cost.setZero(previous.size(), previous.size());
        mask.setZero(previous.size(), previous.size());
        cover_row = vector<int>(previous.size(),0);
        cover_col = vector<int>(previous.size(),0);
        path.setZero(previous.size()*2,2);
    }
    else if (detected.size() > previous.size())
    {
        cost.setZero(detected.size(), detected.size());
        mask.setZero(detected.size(), detected.size());
        cover_row = vector<int>(detected.size(),0);
        cover_col = vector<int>(detected.size(),0);
        path.setZero(detected.size()*2,2);
    }

    for (int i=0;i<detected.size();i++)
    {
        for (int j=0;j<previous.size();j++)
        {
            cost (i,j) = cv::norm ( detected[i] - previous[j] );
        }
    }
}

inline void alan_pose_estimation::LedNodelet::stp1(int& step)
{
    double minval;
    Eigen::MatrixXd minvals;

    minvals = cost.rowwise().minCoeff();
    for (int i = 0; i < cost.rows(); i++)
    {
        minval = minvals(i, 0);
        for (int j = 0; j < cost.cols(); j++)
        {
            cost(i, j) = cost(i, j) - minval;
        }
    }
    step = 2;
}

inline void alan_pose_estimation::LedNodelet::stp2(int &step)
{
    for (int r = 0; r < cost.rows(); r++)
    {
        for (int c = 0; c < cost.cols(); c++)
        {
            if (cost(r, c) == 0 && cover_row[r] == 0 && cover_col[c] == 0)
            {
                mask(r, c) = 1;
                cover_row[r] = 1;
                cover_col[c] = 1;
            }
        }
    }
    for (int r = 0; r < cost.rows(); r++)
        cover_row[r] = 0;
    for (int c = 0; c < cost.cols(); c++)
        cover_col[c] = 0;
    step = 3;
}

inline void alan_pose_estimation::LedNodelet::stp3(int &step)
{
    int count = 0;
    for (int r = 0; r < cost.rows(); r++)
        for (int c = 0; c < cost.cols(); c++)
            if (mask(r, c) == 1)
                cover_col[c] = 1;
    for (int c = 0; c < cost.cols(); c++)
        if (cover_col[c] == 1)
            count += 1;
    if (count == cost.cols() )
        step = 7;
    else
        step = 4;
}

inline void alan_pose_estimation::LedNodelet::stp4(int &step)
{
    int row = -1;
    int col = -1;
    bool done;
    done = false;

    while (!done)
    {
        find_a_zero(row, col);
        if (row == -1)
        {
            done = true;
            step = 6;
        }
        else
        {
            mask(row, col) = 2;
            if (star_in_row(row))
            {
                find_star_in_row(row, col);
                cover_row[row] = 1;
                cover_col[col] = 0;
            }
            else
            {
                done = true;
                step = 5;
                path_row_0 = row;
                path_col_0 = col;
            }
        }
    }
}

inline void alan_pose_estimation::LedNodelet::stp5(int &step)
{
    bool done;
    int row = -1;
    int col = -1;

    path_count = 1;
    path(path_count - 1, 0) = path_row_0;
    path(path_count - 1, 1) = path_col_0;
    done = false;
    while (!done)
    {
        find_star_in_col(path(path_count - 1, 1), row);
        if (row > -1)
        {
            path_count += 1;
            path(path_count - 1, 0) = row;
            path(path_count - 1, 1) = path(path_count - 2, 1);
        }
        else
            done = true;
        if (!done)
        {
            find_prime_in_row(path(path_count - 1, 0), col);
            path_count += 1;
            path(path_count - 1, 0) = path(path_count - 2, 0);
            path(path_count - 1, 1) = col;
        }
    }
    augment_path();
    clear_covers();
    erase_primes();
    step = 3;
}

inline void alan_pose_estimation::LedNodelet::stp6(int &step)
{
    double minval = DBL_MAX;
    find_min(minval);
    for (int r = 0; r < cost.rows(); r++)
        for (int c = 0; c < cost.cols(); c++)
        {
            if (cover_row[r] == 1)
                cost(r, c) += minval;
            if (cover_col[c] == 0)
                cost(r, c) -= minval;
        }
    //cout<<minval<<endl;
    step = 4;
}

inline void alan_pose_estimation::LedNodelet::stp7()
{
    for(int r = 0; r<cost.rows(); r++)
    {
        for (int c = 0; c<cost.cols();c++)
        {
            if(mask(r,c) == 1 && copy(r,c) <= 100 /*&& copy(r,c) != 0*/   )
            {
                Match temp = {c,false};
                id_match.push_back(temp);
            }
            else if(mask(r,c) == 1 && copy(r,c) > 100 /*|| copy(r,c) == 0   )*/)
            {
                Match temp = {c,true};
                id_match.push_back(temp);
            }

        }
    }
}

inline void alan_pose_estimation::LedNodelet::find_a_zero(int &row, int &col)
{
    int r = 0;
    int c;
    bool done;
    row = -1;
    col = -1;
    done = false;

    while (!done)
    {
        c = 0;
        while (true)
        {
            if (cost(r, c) == 0 && cover_row[r] == 0 && cover_col[c] == 0)
            {
                row = r;
                col = c;
                done = true;
            }
            c += 1;
            if (c >= cost.cols() || done)
                break;
        }
        r += 1;
        if (r >= cost.rows())
            done = true;
    }
}

inline bool alan_pose_estimation::LedNodelet::star_in_row(int row)
{
    bool temp = false;
    for (int c = 0; c < cost.cols(); c++)
        if (mask(row, c) == 1)
        {
            temp = true;
            break;
        }
    return temp;
}

inline void alan_pose_estimation::LedNodelet::find_star_in_row(int row, int &col)
{
    col = -1;
    for (int c = 0; c < cost.cols(); c++)
    {
        if (mask(row, c) == 1)
            col = c;
    }
}

inline void alan_pose_estimation::LedNodelet::find_min(double &minval)
{
    for (int r = 0; r < cost.rows(); r++)
        for (int c = 0; c < cost.cols(); c++)
            if (cover_row[r] == 0 && cover_col[c] == 0)
                if (minval > cost(r, c))
                    minval = cost(r, c);
}

inline void alan_pose_estimation::LedNodelet::find_star_in_col(int col, int &row)
{
    row = -1;
    for (int i = 0; i < cost.rows(); i++)
        if (mask(i, col) == 1)
            row = i;
}

inline void alan_pose_estimation::LedNodelet::find_prime_in_row(int row, int &col)
{
    for (int j = 0; j < cost.cols(); j++)
        if (mask(row, j) == 2)
            col = j;
}

inline void alan_pose_estimation::LedNodelet::augment_path()
{
    for (int p = 0; p < path_count; p++)
    {
        for (int p = 0; p < path_count; p++)
        {
            int i = path(p, 0);
            int j = path(p, 1);
            if (mask(i, j) == 1)
                mask(i, j) = 0;
            else
                mask(i, j) = 1;
        }

    }
}

inline void alan_pose_estimation::LedNodelet::clear_covers()
{
    for (int r = 0; r < cost.rows(); r++)
        cover_row[r] = 0;
    for (int c = 0; c < cost.cols(); c++)
        cover_col[c] = 0;
}

inline void alan_pose_estimation::LedNodelet::erase_primes()
{
    for (int r = 0; r < cost.rows(); r++)
        for (int c = 0; c < cost.cols(); c++)
            if (mask(r, c) == 2)
                mask(r, c) = 0;
}


#endif

