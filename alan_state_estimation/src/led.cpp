#include "include/led.h"

void alan::LedNodelet::camera_callback(const sensor_msgs::CompressedImageConstPtr & rgbmsg, const sensor_msgs::ImageConstPtr & depthmsg)
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
        display = frame.clone();
        frame1 = frame.clone();
        hsv = frame.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    double t1 = ros::Time::now().toSec(); 

    time_predicted = rgbmsg->header.stamp.toSec();

    solve_pose_w_LED(frame, depth);
    
    double t2 = ros::Time::now().toSec();

    uavpose_pub.publish(uav_pose_estimated);

    set_image_to_publish(t2, t1, rgbmsg);

    // cout<<global_counter<<endl;
    frame0 = frame1.clone();
} 

void alan::LedNodelet::solve_pose_w_LED(cv::Mat& frame, cv::Mat depth)
{
    Sophus::SE3d pose;
    vector<correspondence::matchid> corres;

    if(!LED_tracker_initiated_or_tracked)        
    {
        LED_tracker_initiated_or_tracked = LED_tracking_initialize(frame, depth);

        if(LED_tracker_initiated_or_tracked)
        {
            cout<<"initialized!"<<endl;

            pose_current = pose_predicted;
            // cout<<pose_current<<endl;
            time_current = time_predicted;
            global_counter++;
            cout<<"exit initializer..."<<endl;
        }
        else
        {
            cout<<"try to initialize tracker..."<<endl;
        }

    }
    else
    {
        if(global_counter >= 2)
        {
            set_pose_predict();
            // pause();
            
            recursive_filtering(frame, depth);
            //time step >= 2
            pose_previous = pose_current;
            pose_current = pose_predicted;
            time_previous = time_current;
            time_current = time_predicted;

            global_counter++;
        }
        else
        {
            recursive_filtering(frame, depth);
            //time step = 0, 1
            pose_previous = pose_current;
            pose_current = pose_predicted;
            time_previous = time_current;
            time_current = time_predicted;

            global_counter++;
        }                
        
    }
}

void alan::LedNodelet::recursive_filtering(cv::Mat& frame, cv::Mat depth)
{
    // cout<<"recursive_filtering"<<endl;
    vector<Eigen::Vector2d> pts_2d_detect = LED_extract_POI(frame, depth);
    //what is this for?
    //to extract POI point of interest        

    if(!pts_2d_detect.empty())
    {
        vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);
        //what is this for?
        //to get 3d coordinates in body frame, so that 
        //outlier rejection could be performed

        reject_outlier(pts_3d_pcl_detect, pts_2d_detect);
        get_final_POI(pts_2d_detect);

        detect_no = pts_2d_detect.size();

        bool stop = false;

        cout<<"detection pt size:..."<<pts_2d_detect.size()<<endl;

        if(pts_2d_detect.size() >= 4)
        {            
            correspondence_search_kmeans(pts_3d_pcl_detect, pts_2d_detect);
            // cout<<"correspond:..."<<pts_2d_detect.size()<<endl;

            vector<Eigen::Vector3d> pts_on_body_frame_in_corres_order;
            vector<Eigen::Vector2d> pts_detected_in_corres_order;            

            for(int i = 0; i < corres_global.size(); i++)
            {            
                if(corres_global[i].detected_ornot)
                {   
                    pts_detected_in_corres_order.push_back(corres_global[i].pts_2d_correspond);             
                    pts_on_body_frame_in_corres_order.push_back(pts_on_body_frame[i]);
                    corres_global[i].detected_ornot = false;//reset for next time step
                }        
            }

            detect_no = pts_on_body_frame_in_corres_order.size();

            if(detect_no < 4)
            {
                corres_global.clear();
                if(reinitialization(pts_2d_detect, depth))
                {
                    cout<<"reinitialization successful!"<<endl;

                }
                else
                {
                    cout<<"reinitialization failed!"<<endl;
                    cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/wrong/here" + to_string(i) + ".png", display);
                    i++;
                    LED_tracker_initiated_or_tracked = false;
                }                                
            }
            else
            {
                cout<<"processed "<<pts_on_body_frame_in_corres_order.size()<<" points this time for pnp"<<endl;

                Eigen::Matrix3d R;
                Eigen::Vector3d t;           

                solve_pnp_initial_pose(pts_detected_in_corres_order, pts_on_body_frame_in_corres_order, R, t);        
                
                pose_global_sophus = Sophus::SE3d(R, t);

                if(pts_on_body_frame_in_corres_order.size() == pts_detected_in_corres_order.size())
                        optimize(pose_global_sophus, pts_on_body_frame_in_corres_order, pts_detected_in_corres_order);//pose, body_frame_pts, pts_2d_detect

                double reproject_error = 0;
                for(int i = 0 ; i < pts_on_body_frame_in_corres_order.size(); i++)
                {
                    //auto what : pts_on_body_frame_in_corres_order
                    Eigen::Vector2d reproject = reproject_3D_2D(
                        pts_on_body_frame_in_corres_order[i], 
                        pose_global_sophus
                    ); 
                    double temp_error = (reproject - pts_detected_in_corres_order[i]).norm();
                    //can save as reprojection for next frame
                    reproject_error = reproject_error + temp_error;
                    
                    cv::circle(display, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(0,255,0),-1);
                }

                BA_error = reproject_error;

                pose_predicted.block(0,0,3,3) = pose_global_sophus.rotationMatrix();
                pose_predicted(0,3) = pose_global_sophus.translation().x();
                pose_predicted(1,3) = pose_global_sophus.translation().y();
                pose_predicted(2,3) = pose_global_sophus.translation().z();
                pose_predicted(3,3) = 1.0;

                map_SE3_to_pose(pose_global_sophus);
            }            
            // pause();
        }    
        else
        {
            corres_global.clear();
            // pose_predicted = pose_current;
            LED_tracker_initiated_or_tracked = false;
            // cout<<"less than 4...!"<<endl;
            // cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/lessthan4" + to_string(i) + ".png", frame_input);
            // i++;
            // LED_tracker_initiated_or_tracked = false;
            // pose_global = pose_predicted;
            // global_counter = 0;
        }

    }
                                
}

inline Eigen::Vector2d alan::LedNodelet::reproject_3D_2D(Eigen::Vector3d P, Sophus::SE3d pose)
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

void alan::LedNodelet::solve_pnp_initial_pose(vector<Eigen::Vector2d> pts_2d, vector<Eigen::Vector3d> body_frame_pts, Eigen::Matrix3d& R, Eigen::Vector3d& t)
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

}

void alan::LedNodelet::optimize(Sophus::SE3d& pose, vector<Eigen::Vector3d> pts_3d_exists, vector<Eigen::Vector2d> pts_2d_detected)
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


        pose = Sophus::SE3d::exp(dx) * pose;

        lastcost = cost;

        if(dx.norm() < converge_threshold)        
            break;
    }

    cout<<"BA: "<<lastcost<<endl;
    BA_error = lastcost;

    // cout<<"gone thru: "<<i<<" th, end optimize"<<endl<<endl;;;

}

void alan::LedNodelet::solveJacobian(Eigen::Matrix<double, 2, 6>& Jacob, Sophus::SE3d pose, Eigen::Vector3d point_3d)
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

vector<Eigen::Vector2d> alan::LedNodelet::LED_extract_POI(cv::Mat& frame, cv::Mat depth)
{    
    cv::Mat depth_mask_src = depth.clone(), depth_mask_dst1, depth_mask_dst2;

    cv::threshold(depth_mask_src, depth_mask_dst1, LANDING_DISTANCE * 1000, 50000, cv::THRESH_BINARY_INV); //filter out far depths

    cv::threshold(depth_mask_src, depth_mask_dst2, 0.5, 50000, cv::THRESH_BINARY); //filter out zeros

    cv::bitwise_and(depth_mask_dst1, depth_mask_dst2, depth_mask_src);
    
    depth_mask_src.convertTo(depth_mask_src, CV_8U);

    
    cv::Mat d_img = depth;
    int size = d_img.cols * d_img.rows;

    for(int i=0; i < size; i++)
    {
        if(isnan(d_img.at<ushort>(i)))
        {
            d_img.at<ushort>(i) = 0;
        }
        if(d_img.at<ushort>(i) > 10000|| d_img.at<ushort>(i) < 100)
        {
            d_img.at<ushort>(i) = 0;
        }
    }
   

    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    cv::threshold(frame, frame, BINARY_THRES, 255, cv::THRESH_BINARY);
    // cout<<"first blob frame type:..."<<frame.type()<<endl;
    frame_initial_thresholded = frame.clone();
    //255


    //detect frame after filter out background
    cv::bitwise_and(depth_mask_src, frame, frame); //filter out with depth information

    // vector<cv::Point> nonzeros;

    // frame_input = frame.clone();
    // cv::cvtColor(frame_input, frame_input, cv::COLOR_GRAY2RGB);

    // cv::Canny( frame_input, frame_input, 50, 100 );
    // vector<vector<cv::Point> > contours;
    // vector<cv::Vec4i> hierarchy;
    // cv::findContours( frame_input, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
    // cv::Mat drawing = cv::Mat::zeros( frame_input.size(), CV_8UC3 );
    // for( size_t i = 0; i< contours.size(); i++ )
    // {
    //     cv::Scalar color = CV_RGB(0,255,0);
    //     drawContours( drawing, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0 );
    // }
    // cv::imshow( "Contours", drawing );
    // cv::waitKey(0);
    // ros::shutdown();

    // Blob method
    vector<cv::KeyPoint> keypoints_rgb_d, keypoints_rgb;
	cv::SimpleBlobDetector::Params params;

	params.filterByArea = false;
    params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByConvexity = false;
	params.filterByInertia = false;
    params.minDistBetweenBlobs = 0.01;
    // params.

	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    
    detector->detect(frame, keypoints_rgb_d);
	cv::drawKeypoints( frame, keypoints_rgb_d, im_with_keypoints,CV_RGB(255,0,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    // cv::imshow("blob", im_with_keypoints);
    // cv::waitKey(10);
    // cout<<keypoints_rgb_d.size()<<endl;
    
    detect_no = keypoints_rgb_d.size();
    
    blobs_for_initialize = keypoints_rgb_d;

    vector<Eigen::Vector2d> POI;
    for(auto what : keypoints_rgb_d)
    {
        POI.push_back(Eigen::Vector2d(what.pt.x, what.pt.y));
    }

    return POI;
}

vector<Eigen::Vector3d> alan::LedNodelet::pointcloud_generate(vector<Eigen::Vector2d> pts_2d_detected, cv::Mat depthimage)
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

        depth_avg_of_all = depth_avg_of_all + z_depth;

        temp.x() = x_pixel;
        temp.y() = y_pixel;
        temp.z() = 1;

        temp = z_depth * cameraMat.inverse() * temp;

        
        pointclouds.push_back(temp);
    }

    depth_avg_of_all = depth_avg_of_all / pointclouds.size();

    // cout<<"average depth:..."<<depth_avg_of_all / pointclouds.size()<<endl;

    return pointclouds;
}

void alan::LedNodelet::get_final_POI(vector<Eigen::Vector2d>& pts_2d_detected)
{
    double x_min = INFINITY, y_min = INFINITY;
    double x_max = -INFINITY, y_max = -INFINITY;

    for(auto what : pts_2d_detected)
    {
        if(what.x() < x_min)
            x_min = what.x();
        
        if(what.y() < y_min)
            y_min = what.y();
        
        if(what.x() > x_max)
            x_max = what.x();

        if(what.y() > y_max)
            y_max = what.y();

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
    // cv::bitwise_and(frame_initial_thresholded, ROI_mask, final_ROI);
    // cv::cvtColor(final_ROI, final_ROI, cv::COLOR_GRAY2RGB);
    
    // cv::cvtColor(frame_initial_thresholded, frame_initial_thresholded, cv::COLOR_GRAY2RGB);


    // cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/wrong/finalthres_" + to_string(i) + ".png", final_ROI);
    

    // Blob method
    vector<cv::KeyPoint> keypoints_rgb_d, keypoints_rgb;
	cv::SimpleBlobDetector::Params params;

	params.filterByArea = false;
    params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByConvexity = false;
	params.filterByInertia = false;
    params.minDistBetweenBlobs = 0.01;
    // params.

	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    
    // cout<<"second blob frame type:..."<<final_ROI.type()<<endl;
    detector->detect(final_ROI, keypoints_rgb_d);
	cv::drawKeypoints( final_ROI, keypoints_rgb_d, im_with_keypoints,CV_RGB(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    // cout<<"final size of blob detecions..."<<keypoints_rgb_d.size()<<endl;
    

    // cv::imshow("blob", im_with_keypoints);
    // cv::waitKey(0);
    
    detect_no = keypoints_rgb_d.size();
    
    blobs_for_initialize = keypoints_rgb_d;

    vector<cv::Point2f> POI_pts;
    vector<cv::Point2f> centers;
    cv::Mat labels;

    for(auto what : keypoints_rgb_d)
    {
        POI_pts.emplace_back(cv::Point2f(what.pt.x, what.pt.y));
    }

    int no_cluster = POI_pts.size();


    if(no_cluster > LED_no)
        no_cluster = LED_no;
    else
        no_cluster = no_cluster;

    cout<<"hi..."<<no_cluster<<endl;

    if(no_cluster < 4)
    {
        cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/wrong/final_ROI_gan"+ to_string(i)+ ".png", final_ROI);
        i++;
    }
        


    cv::kmeans(POI_pts, no_cluster, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1), 8, cv::KMEANS_PP_CENTERS, centers);
    // cout<<"hi..."<<no_cluster<<endl;

    pts_2d_detected.clear();
    // cout<<"final blobs size:..."<<centers.size()<<endl;
    for(int i = 0; i < centers.size(); i++)
    {
        pts_2d_detected.emplace_back(centers[i].x, centers[i].y);
        // cv::circle(im_with_keypoints, centers[i], 2.5, CV_RGB(255,0,0),-1);
    }
    frame_input = im_with_keypoints.clone();
    // cv::imshow("final blob!", im_with_keypoints);
    // cv::waitKey(0);
    // ros::shutdown();

}

bool alan::LedNodelet::LED_tracking_initialize(cv::Mat& frame, cv::Mat depth)
{
    hsv = frame.clone();

    double t0 = ros::Time::now().toSec();

    vector<Eigen::Vector2d> pts_2d_detect = LED_extract_POI(frame, depth); 
      
    vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);

    cout<<hsv.channels()<<endl;
    //after above, I got:
    //pointcloud in {c}
    vector<double> norm_of_x_points, norm_of_y_points, norm_of_z_points;

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
        // cout<<"can initialize!"<<endl;

        int i = 0;

        //hsv detect feature
        cv::cvtColor(hsv, hsv, CV_RGB2HSV);
        vector<bool> g_or_r; //g = true

        vector<int> corres_g;
        vector<int> corres_r;

        // cout<<blobs_for_initialize.size()<<endl;

        for(int i = 0 ; i < blobs_for_initialize.size(); i++)
        {
            cv::Point hsv_vertice1 = cv::Point(pts_2d_detect[i].x() - 2 * blobs_for_initialize[i].size,
                                               pts_2d_detect[i].y() - 2 * blobs_for_initialize[i].size);
            cv::Point hsv_vertice2 = cv::Point(pts_2d_detect[i].x() + 2 * blobs_for_initialize[i].size,
                                               pts_2d_detect[i].y() + 2 * blobs_for_initialize[i].size);

            
            cv::Rect letsgethsv(hsv_vertice1, hsv_vertice2);

            cv::Mat ROI(hsv, letsgethsv);

            int size = ROI.cols * ROI.rows;
            // cout<<"size: "<<size<<endl;
            
            double accu = 0;

            cv::Vec3b hsv_value;

            for(int i = 0; i < ROI.rows; i++)
            {
                for(int j = 0; j < ROI.cols; j++)
                {
                    hsv_value = ROI.at<cv::Vec3b>(i, j);
                    // cout<<hsv_value<<endl;
                    if(hsv_value[0] == 0)                    
                        size = size - 1;                
                    else
                        accu = accu + hsv_value[0];
                }
            }

            // cout<<"size: "<<size<<endl;
            // cout<<"hue: "<<accu/size<<endl;     

            if(accu/size < 100)
                corres_g.push_back(i);
            else   
                corres_r.push_back(i);
        }

        vector<int> corres(LED_no);

        // cout<<corres_g.size()<<endl;
        // cout<<corres_r.size()<<endl;
        // cout<<"`````````````"<<endl;

        if(corres_g.size() != 3 || corres_r.size() != 3)
        {
            // cout<<"hue not found continue!"<<endl;
            return false;
        }

        vector<int> final_corres;
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


                vector<Eigen::Vector2d> pts_2d_detect_temp;   

                // cout<<endl<<corres.size()<<endl;    

                for(auto what : corres)
                {
                    pts_2d_detect_temp.push_back(pts_2d_detect[what]);
                }
                                                        
                solve_pnp_initial_pose(pts_2d_detect_temp, pts_on_body_frame, R, t);
                
                pose_global_sophus = Sophus::SE3d(R, t);

                Eigen::Vector2d reproject, error; 
                double e = 0;

                for(int i = 0 ; i < pts_on_body_frame.size(); i++)
                {
                    reproject = reproject_3D_2D(pts_on_body_frame[i], pose_global_sophus);  
                    error = pts_2d_detect_temp[i] - reproject;
                    e = e + error.norm();
                }

                if(e < error_total)
                {                    
                    error_total = e;
                    final_corres = corres;
                    

                    if(error_total < 5)
                        break;
                }                        
            } while (next_permutation(corres_r.begin(), corres_r.end()));

        } while(next_permutation(corres_g.begin(), corres_g.end()));

        

        

        cout<<"final error :" <<error_total<<endl;
        BA_error = error_total;


        correspondence::matchid corres_temp;
        
        vector<Eigen::Vector2d> pts_2d_detect_correct_order;
        
        for(auto what : final_corres)
        {
            corres_temp.detected_indices = what;
            corres_temp.detected_ornot = true;
            corres_temp.pts_3d_correspond = pts_3d_pcl_detect[what];            
            corres_temp.pts_2d_correspond = pts_2d_detect[what];

            pts_2d_detect_correct_order.push_back(pts_2d_detect[what]);
            
            // cout<<corres_temp.pts_2d_correspond<<endl;
            // cout<<"i :"<< what<<endl;                        

            corres_global.push_back(corres_temp);
        }        
        double t1 = ros::Time::now().toSec();

        optimize(pose_global_sophus, pts_on_body_frame, pts_2d_detect_correct_order);

        // pose_glo
        pose_predicted.block(0,0,3,3) = pose_global_sophus.rotationMatrix();
        pose_predicted(0,3) = pose_global_sophus.translation().x();
        pose_predicted(1,3) = pose_global_sophus.translation().y();
        pose_predicted(2,3) = pose_global_sophus.translation().z();
        pose_predicted(3,3) = 1.0;

        // cout<<"initialization time:..."<< 1 / (t1 - t0)<<endl;

        return true;
    }
    else
        return false;

}

bool alan::LedNodelet::reinitialization(vector<Eigen::Vector2d> pts_2d_detect, cv::Mat depth)
{
    cout<<"reinitialization"<<endl;
    vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);
    vector<double> norm_of_x_points, norm_of_y_points, norm_of_z_points;

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
        // cout<<"can initialize!"<<endl;

        int i = 0;

        //hsv detect feature
        cv::cvtColor(hsv, hsv, CV_RGB2HSV);
        vector<bool> g_or_r; //g = true

        vector<int> corres_g;
        vector<int> corres_r;

        // cout<<blobs_for_initialize.size()<<endl;

        for(int i = 0 ; i < blobs_for_initialize.size(); i++)
        {
            cv::Point hsv_vertice1 = cv::Point(pts_2d_detect[i].x() - 2 * blobs_for_initialize[i].size,
                                               pts_2d_detect[i].y() - 2 * blobs_for_initialize[i].size);
            cv::Point hsv_vertice2 = cv::Point(pts_2d_detect[i].x() + 2 * blobs_for_initialize[i].size,
                                               pts_2d_detect[i].y() + 2 * blobs_for_initialize[i].size);

            
            cv::Rect letsgethsv(hsv_vertice1, hsv_vertice2);

            cv::Mat ROI(hsv, letsgethsv);

            int size = ROI.cols * ROI.rows;
            // cout<<"size: "<<size<<endl;
            
            double accu = 0;

            cv::Vec3b hsv_value;

            for(int i = 0; i < ROI.rows; i++)
            {
                for(int j = 0; j < ROI.cols; j++)
                {
                    hsv_value = ROI.at<cv::Vec3b>(i, j);
                    // cout<<hsv_value<<endl;
                    if(hsv_value[0] == 0)                    
                        size = size - 1;                
                    else
                        accu = accu + hsv_value[0];
                }
            }

            // cout<<"size: "<<size<<endl;
            // cout<<"hue: "<<accu/size<<endl;     

            if(accu/size < 100)
                corres_g.push_back(i);
            else   
                corres_r.push_back(i);
        }

        vector<int> corres(LED_no);

        // cout<<corres_g.size()<<endl;
        // cout<<corres_r.size()<<endl;
        // cout<<"`````````````"<<endl;

        if(corres_g.size() != 3 || corres_r.size() != 3)
        {
            // cout<<"hue not found continue!"<<endl;
            return false;
        }

        vector<int> final_corres;
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


                vector<Eigen::Vector2d> pts_2d_detect_temp;   

                cout<<"corres:..."<<corres.size()<<endl;    

                for(auto what : corres)
                {
                    cout<<what<<endl;
                    cout<<pts_2d_detect[what]<<endl;
                    pts_2d_detect_temp.push_back(pts_2d_detect[what]);
                }
                cout<<endl;
                                                        
                solve_pnp_initial_pose(pts_2d_detect_temp, pts_on_body_frame, R, t);
                
                pose_global_sophus = Sophus::SE3d(R, t);

                Eigen::Vector2d reproject, error; 
                double e = 0;

                for(int i = 0 ; i < pts_on_body_frame.size(); i++)
                {
                    reproject = reproject_3D_2D(pts_on_body_frame[i], pose_global_sophus);  
                    error = pts_2d_detect_temp[i] - reproject;
                    e = e + error.norm();
                }

                if(e < error_total)
                {                    
                    error_total = e;
                    final_corres = corres;
                    

                    if(error_total < 5)
                        break;
                }                        
            } while (next_permutation(corres_r.begin(), corres_r.end()));

        } while(next_permutation(corres_g.begin(), corres_g.end()));

        

        // cout<<"final error :" <<error_total<<endl;


        correspondence::matchid corres_temp;
        
        vector<Eigen::Vector2d> pts_2d_detect_correct_order;
        
        for(auto what : final_corres)
        {
            corres_temp.detected_indices = what;
            corres_temp.detected_ornot = true;
            corres_temp.pts_3d_correspond = pts_3d_pcl_detect[what];            
            corres_temp.pts_2d_correspond = pts_2d_detect[what];

            pts_2d_detect_correct_order.push_back(pts_2d_detect[what]);
            
            // cout<<corres_temp.pts_2d_correspond<<endl;
            // cout<<"i :"<< what<<endl;                        

            corres_global.push_back(corres_temp);
        }        
        double t1 = ros::Time::now().toSec();

        // pose_glo
        pose_predicted.block(0,0,3,3) = pose_global_sophus.rotationMatrix();
        pose_predicted(0,3) = pose_global_sophus.translation().x();
        pose_predicted(1,3) = pose_global_sophus.translation().y();
        pose_predicted(2,3) = pose_global_sophus.translation().z();
        pose_predicted(3,3) = 1.0;

        // cout<<"initialization time:..."<< 1 / (t1 - t0)<<endl;
        
        return true;
    }
    else
    {
        
        return false;
    }
        

}

void alan::LedNodelet::correspondence_search(vector<Eigen::Vector3d> pts_3d_detected, vector<Eigen::Vector2d> pts_2d_detected)
{
    vector<Eigen::Vector2d> pts_reproject_current;
    vector<Eigen::Vector2d> pts_reproject_predict;

    Eigen::Vector2d reproject_temp;

    // cout<<"fix size: "<<pts_on_body_frame.size()<<endl;
    double e0;
    
    for(auto what : pts_on_body_frame)
    {
        // cout<<pose_global<<endl;
        // global_pose_
        reproject_temp = reproject_3D_2D(
            what, 
            Sophus::SE3d(pose_current)
        );

        //pose_global_sophus
        // cv::circle(display, cv::Point(reproject_temp(0), reproject_temp(1)), 2.5, CV_RGB(255,0,0),-1);
        // cv::imshow("display", display);
        // cv::waitKey(10);
        pts_reproject_current.push_back(reproject_temp);

        reproject_temp = reproject_3D_2D(
            what, 
            Sophus::SE3d(pose_previous)
        );
        // cv::circle(display, cv::Point(reproject_temp(0), reproject_temp(1)), 2.5, CV_RGB(255,0,0),-1);
        pts_reproject_predict.push_back(reproject_temp);
    }

    hungarian1.solution(pts_reproject_current, pts_2d_detected);
    hungarian2.solution(pts_reproject_predict, pts_2d_detected);
    
    bool stop = false;
    double xcoord;
    xcoord = -INFINITY;

    for(int i = 0; i < corres_global.size(); i++)
    {
        
        if(hungarian1.id_match[i].detected_ornot)
        {
            corres_global[i].detected_ornot = hungarian1.id_match[i].detected_ornot;
            corres_global[i].detected_indices = hungarian1.id_match[i].detected_indices;



            corres_global[i].pts_3d_correspond = pts_3d_detected[hungarian1.id_match[i].detected_indices];
            corres_global[i].pts_2d_correspond = pts_2d_detected[hungarian1.id_match[i].detected_indices];


            if(corres_global[i].pts_2d_correspond.x() < xcoord)
            {
                // cout<<corres_global[i].pts_2d_correspond.x()<<endl;
                // cout<<xcoord<<endl;
                // stop = true;
            }
            xcoord = corres_global[i].pts_2d_correspond.x();

        }
        else
        {
            corres_global[i].detected_ornot = hungarian1.id_match[i].detected_ornot;
        }

    }

    if(stop)
    {
        cout<<"wrong correspondencesss!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        // cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/wrong/" + to_string(i) + ".png", frame_input);
        cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/wrong/repro_" + to_string(i) + ".png", display);
        // cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/wrong/frame0_" + to_string(i) + ".png", frame0);
        // cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/wrong/frame1_" + to_string(i) + ".png", frame1);
        cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/wrong/final_previous_" + to_string(i) + ".png", final_frame);
        cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/wrong/" + to_string(i) + ".png", im_with_keypoints);
        // i++;
        // if(reinitialization(pts_2d_detected, pts_3d_detected))
        // {
        //     global_counter = 1;

        // }
        // else
        // {
        //     LED_tracker_initiated_or_tracked = false;
        //     global_counter = 0;

        // }

        

        // ros::shutdown();

        // cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/wrong/" + to_string(i) + ".png", frame_input);
        // i++;

        // cv::imwrite("/home/patty/alan_ws/src/alan/offb/src/alan_state_estimation/test/outliers/outlier" + to_string(ind) + ".png", frame);

        //use hue again!
        //also consider points less than 6 points
        
        // cout<<pts_reproject.size()<<endl;
        // cout<<pts_2d_detected.size()<<endl;
        
        // for(auto what : pts_reproject)
        // {
        //     cout<<what<<endl;
        // }
        // cout<<"here!--------------------"<<endl;
            
        // for(auto what : pts_2d_detected)
        // {
        //     cout<<what<<endl;            
        // }

        // cout<<"shutdown"<<endl;            
        // ros::shutdown();
    }

}

void alan::LedNodelet::correspondence_search_kmeans(vector<Eigen::Vector3d> pts_3d_detected, vector<Eigen::Vector2d> pts_2d_detected)
{
    vector<cv::Point2f> pts;
    Eigen::Vector2d reproject_temp;  

    cout<<"enter here!"<<endl;  

    for(auto what : pts_on_body_frame)
    {
        // cout<<pose_global<<endl;
        // global_pose_
        reproject_temp = reproject_3D_2D(
            what, 
            Sophus::SE3d(pose_current)
        );

        //pose_global_sophus
        // cv::circle(display, cv::Point(reproject_temp(0), reproject_temp(1)), 2.5, CV_RGB(255,0,0),-1);
        // reproject_temp = reproject_3D_2D(
        //     what, 
        //     Sophus::SE3d(pose_previous)
        // );
        // cv::circle(display, cv::Point(reproject_temp(0), reproject_temp(1)), 2.5, CV_RGB(0,0,255),-1);
        
        
        
        // cv::imshow("display", display);
        // cv::waitKey(10);
        // cout<<reproject_temp<<"------"<<endl<<endl;
        pts.emplace_back(cv::Point2f(reproject_temp.x(), reproject_temp.y()));
    }
    //first emplace back pts_on_body_frame in 2D at this frame
    //in preset order

    for(auto what : pts_2d_detected)
    {
        // cout<<what<<endl<<"------"<<endl;;
        pts.emplace_back(cv::Point2f(what.x(), what.y()));
    }

    
    vector<cv::Point2f> centers;
    cv::Mat labels;

    double t0 = ros::Time::now().toSec();
    
    cout<<pts.size()<<endl;

    cv::kmeans(pts, LED_no, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1), 8, cv::KMEANS_PP_CENTERS, centers);
    
    for( i = 0; i < pts.size(); i++ )
    {
        int clusterIdx = labels.at<int>(i);
        cout<<clusterIdx<<endl;
        // cv::Point ipt = pts[i];
        // circle( display, ipt, 2, colorTab[clusterIdx], cv::FILLED, cv::LINE_AA );
    }
    
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
                    cout<<i<<k<<LED_no<<endl;
                    error_count++;
                    break;
                }  
                else
                {
                    cout<<clusterIdx<<clusterIdx_detected<<endl;
                    corres_global[i].detected_ornot = true;
                    corres_global[i].pts_2d_correspond = pts_2d_detected[k - LED_no];
                }                                                    
                cv::Point ipt = pts[i];
                // circle( display, ipt, 1, colorTab[clusterIdx], cv::FILLED, cv::LINE_AA );

                ipt = pts[k];
                // circle( display, ipt, 1, colorTab[clusterIdx], cv::FILLED, cv::LINE_AA );
            }
        }
    }


    double t1 = ros::Time::now().toSec();



    cout<<"fps: "<<1/(t1-t0)<<endl;
    // cv::imshow("kmeans", display);
    // cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/wrong/lala"+ to_string(i)+ ".png", display);
    // cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/wrong/input"+ to_string(i)+ ".png", frame_input);
    // cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/wrong/final_last"+ to_string(i)+ ".png", final_frame);
    // cv::waitKey(0);
    // ros::shutdown();

}

void alan::LedNodelet::set_pose_predict()
{
    // cout<<"set_pose_predict"<<endl;
    double delta_time = (time_predicted - time_current) / (time_current - time_previous);


    // Eigen::Matrix4d 
    Eigen::VectorXd delta;
    delta.resize(6);
    delta = logarithmMap(pose_previous.inverse() * pose_current);
    
    Eigen::VectorXd delta_hat;
    delta_hat.resize(6);
    delta_hat = delta * delta_time;    

    
    

    pose_predicted = pose_current * exponentialMap(delta_hat);
    
    // cout<<endl;
    // cout<<delta_hat<<endl;
    // cout<<time_predicted<<endl;
    // cout<<time_current<<endl;
    // cout<<time_previous<<endl;
    // cout<<time_predicted - time_current<<endl;
    // cout<<time_current - time_previous<<endl;
    // cout<<pose_previous<<endl;
    // cout<<pose_current<<endl;
    // cout<<pose_predicted<<endl;
}

//outlier rejection
    //in outlier rejection, we first calculate the MAD (mean average deviation)
    //to see whether there exists some outlier or not.
    //then, we try to do clustering with k-means algorithm.
    //as we are processing 3D points, at most time, 
    //the LED blobs should be close enough, 
    //while others being at some other coordinates that are pretty far away
    //hence, we set the clustering no. as 2.
    //we then calcullate the distance between the centroid of the cluster to the
    //center at previous time step(pcl_center_point_wo_outlier_previous)
    //and determine which cluster is the one that we want
void alan::LedNodelet::reject_outlier(vector<Eigen::Vector3d>& pts_3d_detect, vector<Eigen::Vector2d>& pts_2d_detect)
{
    // cout<<
    int n = pts_3d_detect.size();

    vector<cv::Point3f> pts;
    vector<double> norm_of_x_points;
    vector<double> norm_of_y_points;
    vector<double> norm_of_z_points;

    for(auto what :  pts_3d_detect)
    {
        norm_of_x_points.push_back(what.x());
        norm_of_y_points.push_back(what.y());
        norm_of_z_points.push_back(what.z());
        pts.push_back(cv::Point3f(what.x(), what.y(), what.z()));
    }
    
    // double MAD = calculate_MAD(norm_of_points);
    // cout<<"MAD: "<<MAD<<endl;

    cv::Mat labels;
    std::vector<cv::Point3f> centers;

    
    if(calculate_MAD(norm_of_x_points) > MAD_x_threshold  
        || calculate_MAD(norm_of_y_points) > MAD_y_threshold
        || calculate_MAD(norm_of_z_points) > MAD_z_threshold)
    {
        // ind++;m
        // cv::imwrite("/home/patty/alan_ws/src/alan/offb/src/alan_state_estimation/test/outliers/outlier" + to_string(ind) + ".png", frame);
        
        cv::kmeans(pts, 2, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1), 8, cv::KMEANS_PP_CENTERS, centers);

        // cout<<"center size: "<<centers.size()<<endl;

        // cout<<norm(pcl_center_point_wo_outlier_previous)<<endl;;
        // cout<<norm(centers[0])<<endl;
        // cout<<norm(centers[1])<<endl;

        double d0 = cv::norm(pcl_center_point_wo_outlier_previous - centers[0]);
        double d1 = cv::norm(pcl_center_point_wo_outlier_previous - centers[1]);
        
        // cout<<"distance"<<endl;
        // cout<<d0<<endl;
        // cout<<d1<<endl;
        // cout<<"label: "<<labels<<endl;
        // cout<<"before erasing: "<<pts_2d_detect.size()<<endl;

        vector<Eigen::Vector2d> pts_2d_result;
        vector<Eigen::Vector3d> pts_3d_result;

        if(d0 < d1) //then get index with 0
        {           
            // cout<<"cluster 0 is the right one"<<endl;
            for(int i = 0; i < labels.rows; i++)
            {
                cout<<labels.at<int>(0,i)<<endl;
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
            // cout<<"cluster 1 is the right one"<<endl;
            for(int i = 0; i < labels.rows; i++)
            {
                // cout<<labels.at<int>(0,i)<<endl;

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

        // cout<<"after erasing: "<<pts_2d_detect.size()<<endl;
        // cout<<"after erasing: "<<pts_3d_detect.size()<<endl;
    }
    else
    {
        cv::Mat temp;
        
        cv::reduce(pts, temp, 01, CV_REDUCE_AVG);
        pcl_center_point_wo_outlier_previous = cv::Point3f(temp.at<float>(0,0), temp.at<float>(0,1), temp.at<float>(0,2));

        // cout<<temp(1)<<endl;
        // cout<<temp(2)<<endl;
    }

}

inline double alan::LedNodelet::calculate_MAD(vector<double> norm_of_points)
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

void alan::LedNodelet::map_SE3_to_pose(Sophus::SE3d pose)
{
    uav_pose_estimated.pose.position.x = pose.translation().x();
    uav_pose_estimated.pose.position.y = pose.translation().y();
    uav_pose_estimated.pose.position.z = pose.translation().z();

    Eigen::Quaterniond q = Eigen::Quaterniond(pose.rotationMatrix());
    uav_pose_estimated.pose.orientation.w = q.w();
    uav_pose_estimated.pose.orientation.x = q.x();
    uav_pose_estimated.pose.orientation.y = q.y();
    uav_pose_estimated.pose.orientation.z = q.z();
}

void alan::LedNodelet::set_image_to_publish(double t2, double t1, const sensor_msgs::CompressedImageConstPtr & rgbmsg)
{
    
    char hz[40];
    char fps[10] = " fps";
    sprintf(hz, "%.2f", 1 / (t2 - t1));
    strcat(hz, fps);

    char BA[40] = "BA: ";
    char BA_error_display[10];
    sprintf(BA_error_display, "%.2f", BA_error);
    strcat(BA, BA_error_display);
    
    cv::putText(display, hz, cv::Point(20,40), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));  
    cv::putText(display, to_string(detect_no), cv::Point(720,460), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));
    cv::putText(display, BA, cv::Point(720,60), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));
    cv::putText(display, to_string(depth_avg_of_all), cv::Point(20,460), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));

    cv::Mat imageoutput = display.clone();
    cv_bridge::CvImage for_visual;
    for_visual.header = rgbmsg->header;
    for_visual.encoding = sensor_msgs::image_encodings::BGR8;
    for_visual.image = imageoutput;
    this->pubimage.publish(for_visual.toImageMsg());
    final_frame = display;


    cv_bridge::CvImage for_visual_input;
    for_visual_input.header = rgbmsg->header;
    for_visual_input.encoding = sensor_msgs::image_encodings::BGR8;
    for_visual_input.image = frame_input;
    this->pubimage_input.publish(for_visual_input.toImageMsg());    

}

Eigen::VectorXd alan::LedNodelet::logarithmMap(Eigen::Matrix4d trans)
{
    Eigen::VectorXd xi;
    xi.resize(6);

    Eigen::Matrix3d R = trans.block<3, 3>(0, 0);
    Eigen::Vector3d t = trans.block<3, 1>(0, 3);
    Eigen::Vector3d w, upsilon;
    Eigen::Matrix3d w_hat;
    Eigen::Matrix3d A_inv;
    double phi = 0;
    double w_norm;

    // Calculate w_hat
    if (R.isApprox(Eigen::Matrix3d::Identity(), 1e-10) == 1)
    {
        // phi has already been set to 0;
        w_hat.setZero();
    }
    else
    {
        double temp = (R.trace() - 1) / 2;
        // Force phi to be either 1 or -1 if necessary. Floating point errors can cause problems resulting in this not happening
        if (temp > 1)
        {
        temp = 1;
        }
        else if (temp < -1)
        {
        temp = -1;
        }

        phi = acos(temp);
        if (phi == 0)
        {
        w_hat.setZero();
        }
        else
        {
        w_hat = (R - R.transpose()) / (2 * sin(phi)) * phi;
        }
    }

    // Extract w from skew symmetrix matrix of w
    w << w_hat(2, 1), w_hat(0, 2), w_hat(1, 0);

    w_norm = w.norm();

    // Calculate upsilon
    if (t.isApproxToConstant(0, 1e-10) == 1)
    {
        A_inv.setZero();
    }
    else if (w_norm == 0 || sin(w_norm) == 0)
    {
        A_inv.setIdentity();
    }
    else
    {
        A_inv = Eigen::Matrix3d::Identity() - w_hat / 2
            + (2 * sin(w_norm) - w_norm * (1 + cos(w_norm))) / (2 * w_norm * w_norm * sin(w_norm)) * w_hat * w_hat;
    }

    upsilon = A_inv * t;

    // Compose twist coordinates vector
    xi.head<3>() = upsilon;
    xi.tail<3>() = w;

    return xi;
}

Eigen::Matrix4d alan::LedNodelet::exponentialMap(Eigen::VectorXd& twist)
{
    Eigen::Vector3d upsilon = twist.head<3>();
    Eigen::Vector3d omega = twist.tail<3>();

    double theta = omega.norm();
    double theta_squared = theta * theta;

    Eigen::Matrix3d Omega = skewSymmetricMatrix(omega);
    Eigen::Matrix3d Omega_squared = Omega * Omega;
    Eigen::Matrix3d rotation;
    Eigen::Matrix3d V;

    if (theta == 0)
    {
        rotation = Eigen::Matrix3d::Identity();
        V.setIdentity();
    }
    else
    {
        rotation = Eigen::Matrix3d::Identity() + Omega / theta * sin(theta)
            + Omega_squared / theta_squared * (1 - cos(theta));
        V = (Eigen::Matrix3d::Identity() + (1 - cos(theta)) / (theta_squared) * Omega
            + (theta - sin(theta)) / (theta_squared * theta) * Omega_squared);
    }

    Eigen::Matrix4d transform;

    transform.setIdentity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = V * upsilon;

    return transform;
}

Eigen::Matrix3d alan::LedNodelet::skewSymmetricMatrix(Eigen::Vector3d w)
{
    Eigen::Matrix3d Omega;
    Omega << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

    return Omega;
}