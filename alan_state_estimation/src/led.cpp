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
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    double t1 = ros::Time::now().toSec(); 

    solve_pose_w_LED(frame, depth);
    
    double t2 = ros::Time::now().toSec();

    uavpose_pub.publish(uav_pose_estimated);


    char hz[40];
    char fps[10] = " fps";
    sprintf(hz, "%.2f", 1 / (t2 - t1));
    strcat(hz, fps);

    char BA[40] = "BA: ";
    char BA_error_display[10];
    sprintf(BA_error_display, "%.2f", BA_error);
    strcat(BA, BA_error_display);
    

    cv::putText(display, hz, cv::Point(20,40), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));  
    cv::putText(display, to_string(detect_no), cv::Point(700,60), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));
    cv::putText(display, BA, cv::Point(720,60), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));

    cv::Mat imageoutput = display.clone();
    cv_bridge::CvImage for_visual;
    for_visual.header = rgbmsg->header;
    for_visual.encoding = sensor_msgs::image_encodings::BGR8;
    for_visual.image = imageoutput;
    
    this->pubimage.publish(for_visual.toImageMsg());


    cv_bridge::CvImage for_visual_input;
    for_visual_input.header = rgbmsg->header;
    for_visual_input.encoding = sensor_msgs::image_encodings::BGR8;

    cv::cvtColor(frame_input, frame_input, cv::COLOR_GRAY2RGB);

    for_visual_input.image = frame_input;

    this->pubimage_input.publish(for_visual_input.toImageMsg());    
} 

void alan::LedNodelet::solve_pose_w_LED(cv::Mat& frame, cv::Mat depth)
{
    Sophus::SE3d pose;
    vector<correspondence::matchid> corres;


    if(!LED_tracker_initiated)        
    {
        LED_tracker_initiated = LED_tracking_initialize(frame, depth);

        if(LED_tracker_initiated)
        {
            cout<<"initialized!"<<endl;
        }

    }
    else
    {
        recursive_filtering(frame, depth);
        // map_SE3_to_pose(pose);
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

    // cout<<"PnP: "<<R<<endl<<endl;;

    // cout<<"PnP: "<<t<<endl<<endl;;

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

    cout<<"gone thru: "<<i<<" th, end optimize"<<endl<<endl;;;

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


    // cv::Mat adjMap;
    // d_img.convertTo(adjMap,CV_8UC1, 255 / (10000.0), 0);
    // cv::applyColorMap(adjMap, adjMap, cv::COLORMAP_RAINBOW);
    // for(int i=0; i<size; i++)
    // {
    //     if(d_img.at<ushort>(i)==0)
    //     {
    //         cv::Vec3b color = adjMap.at<cv::Vec3b>(i);
    //         color[0]=255;
    //         color[1]=255;
    //         color[2]=255;
    //         adjMap.at<cv::Vec3b>(i)=color;
    //     }
    // }

    // cv::imshow("depppth", adjMap);
    // cv::waitKey(20);    

    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    cv::threshold(frame, frame, BINARY_THRES, 255, cv::THRESH_BINARY);
    

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


    //detect frame after filter out background
    cv::bitwise_and(depth_mask_src, frame, frame); //filter out with depth information

    vector<cv::Point> nonzeros;

    frame_input = frame.clone();
    
    detector->detect(frame, keypoints_rgb_d);
	cv::drawKeypoints( frame, keypoints_rgb_d, im_with_keypoints,CV_RGB(255,0,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    
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

        temp.x() = x_pixel;
        temp.y() = y_pixel;
        temp.z() = 1;

        temp = z_depth * cameraMat.inverse() * temp;

        
        pointclouds.push_back(temp);
    }

    return pointclouds;
}


bool alan::LedNodelet::LED_tracking_initialize(cv::Mat& frame, cv::Mat depth)
{
    cout<<"try to initialize tracker..."<<endl;

    cv::Mat hsv = frame.clone();

    vector<Eigen::Vector2d> pts_2d_detect = LED_extract_POI(frame, depth); 
      
    vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);

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
        cout<<"can initialize!"<<endl;

        int i = 0;

        //hsv detect feature
        cv::cvtColor(hsv, hsv, CV_RGB2HSV);
        vector<bool> g_or_r; //g = true

        vector<int> corres_g;
        vector<int> corres_r;

        cout<<blobs_for_initialize.size()<<endl;

        for(int i = 0 ; i < blobs_for_initialize.size(); i++)
        {
            cv::Point hsv_vertice1 = cv::Point(pts_2d_detect[i].x() - 2 * blobs_for_initialize[i].size,
                                               pts_2d_detect[i].y() - 2 * blobs_for_initialize[i].size);
            cv::Point hsv_vertice2 = cv::Point(pts_2d_detect[i].x() + 2 * blobs_for_initialize[i].size,
                                               pts_2d_detect[i].y() + 2 * blobs_for_initialize[i].size);

            
            cv::Rect letsgethsv(hsv_vertice1, hsv_vertice2);

            cv::Mat ROI(hsv, letsgethsv);

            int size = ROI.cols * ROI.rows;
            cout<<"size: "<<size<<endl;
            
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

            cout<<"size: "<<size<<endl;
            cout<<"hue: "<<accu/size<<endl;     

            if(accu/size < 100)
                corres_g.push_back(i);
            else   
                corres_r.push_back(i);
        }

        vector<int> corres(LED_no);

        cout<<corres_g.size()<<endl;
        cout<<corres_r.size()<<endl;
        cout<<"`````````````"<<endl;

        if(corres_g.size() != 3 || corres_r.size() != 3)
        {
            cout<<"hue not found continue!"<<endl;
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

                cout<<endl<<corres.size()<<endl;    

                for(auto what : corres)
                {
                    pts_2d_detect_temp.push_back(pts_2d_detect[what]);
                }
                                                        
                solve_pnp_initial_pose(pts_2d_detect_temp, pts_on_body_frame, R, t);
                
                pose = Sophus::SE3d(R, t);

                Eigen::Vector2d reproject, error; 
                double e = 0;

                for(int i = 0 ; i < pts_on_body_frame.size(); i++)
                {
                    reproject = reproject_3D_2D(pts_on_body_frame[i], pose);  
                    error = pts_2d_detect_temp[i] - reproject;
                    e = e + error.norm();
                }

                if(e < error_total)
                {                    
                    error_total = e;
                    final_corres = corres;
                    pose_global = pose;

                    if(error_total < 5)
                        break;
                }                        
            } while (next_permutation(corres_r.begin(), corres_r.end()));

        } while(next_permutation(corres_g.begin(), corres_g.end()));

        

        cout<<"final error :" <<error_total<<endl;


        correspondence::matchid corres_temp;
        
        vector<Eigen::Vector2d> pts_2d_detect_correct_order;
        
        for(auto what : final_corres)
        {
            corres_temp.detected_indices = what;
            corres_temp.detected_ornot = true;
            corres_temp.pts_3d_correspond = pts_3d_pcl_detect[what];            
            corres_temp.pts_2d_correspond = pts_2d_detect[what];

            pts_2d_detect_correct_order.push_back(pts_2d_detect[what]);
            cout<<corres_temp.pts_2d_correspond<<endl;
            cout<<"i :"<< what<<endl;            

            

            corres_global.push_back(corres_temp);
        }

        vector<Eigen::Vector2d> config_temp = pts_2d_normlization(pts_2d_detect_correct_order);

        for(auto what : config_temp)
        {
            pts_obj_configuration.push_back(what);
        }

        return true;
    }
    else
        return false;

}

vector<Eigen::Vector2d> alan::LedNodelet::pts_2d_normlization(vector<Eigen::Vector2d> pts_2d)
{
    int n = pts_2d.size();

    vector<double> pts_xs;
    vector<double> pts_ys;

    for(int i = 0; i < n; i++)
    {
        pts_xs.push_back(pts_2d[i].x());
        pts_ys.push_back(pts_2d[i].y());
    }

    double x_avg = accumulate(pts_xs.begin(), pts_xs.end(), 0.0) / pts_xs.size();
    double y_avg = accumulate(pts_ys.begin(), pts_ys.end(), 0.0) / pts_ys.size();


    vector<Eigen::Vector2d> pts_2d_results;

    for(int i = 0; i < n; i++)    
    {
        pts_2d_results.push_back(Eigen::Vector2d(pts_2d[i].x() - x_avg, pts_2d[i].y() - y_avg));
    }

    return pts_2d_results;

}

void alan::LedNodelet::recursive_filtering(cv::Mat& frame, cv::Mat depth)
{
    vector<Eigen::Vector2d> pts_2d_detect = LED_extract_POI(frame, depth);
    vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);


    if(!pts_2d_detect.empty())
    {
        reject_outlier(pts_3d_pcl_detect, pts_2d_detect);
        detect_no = pts_2d_detect.size();

        bool stop = false;

        if(detect_no > 6)
        {
            for(auto what : pts_2d_detect)
                cout<<what<<endl;

            // pause();
        }

        if(pts_2d_detect.size() >= 4)
        {
            correspondence_search_test(pts_3d_pcl_detect, pts_2d_detect);

            vector<Eigen::Vector3d> pts_on_body_frame_in_corres_order;
            vector<Eigen::Vector2d> pts_detected_in_corres_order;            

            for(int i = 0; i < corres_global.size(); i++)
            {            
                if(corres_global[i].detected_ornot)
                {                
                    pts_on_body_frame_in_corres_order.push_back(pts_on_body_frame[i]);
                    pts_detected_in_corres_order.push_back(corres_global[i].pts_2d_correspond);
                    
                }        
            }

            detect_no = pts_on_body_frame_in_corres_order.size();

            cout<<"processed "<<pts_on_body_frame_in_corres_order.size()<<" points this time for pnp"<<endl;

            Eigen::Matrix3d R;
            Eigen::Vector3d t;           

            solve_pnp_initial_pose(pts_detected_in_corres_order, pts_on_body_frame_in_corres_order, R, t);        

            pose_global = Sophus::SE3d(R, t);

            if(pts_on_body_frame_in_corres_order.size() == pts_detected_in_corres_order.size())
                    optimize(pose_global, pts_on_body_frame_in_corres_order, pts_detected_in_corres_order);//pose, body_frame_pts, pts_2d_detect

            for(auto what : pts_on_body_frame_in_corres_order)
            {
                Eigen::Vector2d reproject = reproject_3D_2D(what, pose_global);  
                cv::circle(display, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(255,0,0),-1);
            }


            map_SE3_to_pose(pose_global);

            // pause();

        }    

    }
    
                             
}

void alan::LedNodelet::correspondence_search(vector<Eigen::Vector3d> pts_3d_detected, vector<Eigen::Vector2d> pts_2d_detected)
{
    
    //use uzh method!!!!









    
    vector<Eigen::Vector2d> pts_previous;

    for(auto what : this->corres_global)
    {
        pts_previous.push_back(what.pts_2d_correspond);
        // cout<<what.pts_2d_correspond<<endl;
    }
    cout<<"------------------------"<<endl;

    vector<Eigen::Vector2d> pts_previous_normalized, pts_2d_detected_normalized;

    

    pts_previous_normalized = pts_obj_configuration;
    pts_2d_detected_normalized = pts_2d_normlization(pts_2d_detected);

    hungarian.solution(pts_previous_normalized, pts_2d_detected_normalized);

    

    for(auto what :  hungarian.id_match)
        cout<<what.detected_indices<<"  " <<what.detected_ornot<<"     "<<endl;;
        ;
    
    // cout<<endl<<endl;


    bool stop = false;

    double xcoord;
    xcoord = -INFINITY;
    
    for(int i = 0; i < corres_global.size(); i++) // go through every indices in the {L} #0, 1, 2... that indicate the indices in {D}
    {        
        if(hungarian.id_match[i].detected_ornot)
        {
            // cout<<xcoord<<endl;
            // cout<<"ture!"<<endl;
            corres_global[i].detected_ornot = hungarian.id_match[i].detected_ornot;
            corres_global[i].detected_indices = hungarian.id_match[i].detected_indices;
            corres_global[i].pts_3d_correspond = pts_3d_detected[hungarian.id_match[i].detected_indices];
            corres_global[i].pts_2d_correspond = pts_2d_detected[hungarian.id_match[i].detected_indices];
            // cout<<corres_global[i].pts_2d_correspond<<endl;
            // cout<<"end"<<endl;

            if(corres_global[i].pts_2d_correspond.x() < xcoord)
            {
                // cout<<corres_global[i].pts_2d_correspond.x()<<endl;
                // cout<<xcoord<<endl;
                stop = true;
            }
            xcoord = corres_global[i].pts_2d_correspond.x();
            
        }
        else
        {
            // cout<<"false!"<<endl;
            corres_global[i].detected_ornot = hungarian.id_match[i].detected_ornot;
        }
            

        //we need to do more to save the non-detected,
        //save the pts_detected_previous in the same format as global_corres            
            
    }

    if(stop)
    {
        cout<<"wrong correspondencesss!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        cout<<pts_previous_normalized.size()<<endl;
        cout<<pts_2d_detected_normalized.size()<<endl;
        // cv::imwrite("/home/patty/alan_ws/src/alan/offb/src/alan_state_estimation/test/" + to_string(i) + ".png", frame);
        // i++;
        // for(auto what : pts_previous_normalized)
        //     cout<<what<<endl;
        for(auto what : pts_2d_detected_normalized)
            cout<<what<<endl;

        // ros::shutdown();
    }
        

}

void alan::LedNodelet::correspondence_search_test(vector<Eigen::Vector3d> pts_3d_detected, vector<Eigen::Vector2d> pts_2d_detected)
{
    vector<Eigen::Vector2d> pts_reproject;

    Eigen::Vector2d reproject_temp;

    cout<<"fix size: "<<pts_on_body_frame.size()<<endl;
    
    for(auto what : pts_on_body_frame)
    {
        reproject_temp = reproject_3D_2D(what, pose_global);
        cv::circle(display, cv::Point(reproject_temp(0), reproject_temp(1)), 2.5, CV_RGB(0,255,0),-1);

        pts_reproject.push_back(reproject_temp);
    }

    hungarian.solution(pts_reproject, pts_2d_detected);
    
    bool stop = false;
    double xcoord;
    xcoord = -INFINITY;

    for(int i = 0; i < corres_global.size(); i++)
    {
        if(hungarian.id_match[i].detected_ornot)
        {
            corres_global[i].detected_ornot = hungarian.id_match[i].detected_ornot;
            corres_global[i].detected_indices = hungarian.id_match[i].detected_indices;
            corres_global[i].pts_3d_correspond = pts_3d_detected[hungarian.id_match[i].detected_indices];
            corres_global[i].pts_2d_correspond = pts_2d_detected[hungarian.id_match[i].detected_indices];


            if(corres_global[i].pts_2d_correspond.x() < xcoord)
            {
                // cout<<corres_global[i].pts_2d_correspond.x()<<endl;
                // cout<<xcoord<<endl;
                stop = true;
            }
            xcoord = corres_global[i].pts_2d_correspond.x();

        }
        else
        {
            corres_global[i].detected_ornot = hungarian.id_match[i].detected_ornot;
        }

    }

    if(stop)
    {
        cout<<"wrong correspondencesss!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        //use hue again!
        //also consider points less than 6 points






        
        cout<<pts_reproject.size()<<endl;
        cout<<pts_2d_detected.size()<<endl;
        cv::imwrite("/home/patty/alan_ws/src/alan/offb/src/alan_state_estimation/test/wrongframes/" + to_string(i) + ".png", display);
        i++;
        for(auto what : pts_reproject)
        {
            cout<<what<<endl;
        }
        cout<<"here!--------------------"<<endl;
            
        for(auto what : pts_2d_detected)
        {
            cout<<what<<endl;            
        }

        cout<<"shutdown"<<endl;            
        // ros::shutdown();
    }



}


//outlier rejection
void alan::LedNodelet::reject_outlier(vector<Eigen::Vector3d>& pts_3d_detect, vector<Eigen::Vector2d>& pts_2d_detect)
{
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
                cout<<labels.at<int>(0,i)<<endl;

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

void* alan::LedNodelet::PubMainLoop(void* tmp)
{
    LedNodelet* pub = (LedNodelet*) tmp;

    ros::Rate loop_rate(50);
    while (ros::ok()) 
    {
        // ROS_INFO("%d,publish!", num++);
        pub->uavpose_pub.publish(pub->uav_pose_estimated);

        ros::spinOnce();
        loop_rate.sleep();
    }

    void* result;

    return result;
}


void alan::LedNodelet::solveicp_svd(vector<Eigen::Vector3d> pts_3d_camera, vector<Eigen::Vector3d> pts_3d_body, Eigen::Matrix3d& R, Eigen::Vector3d& t)
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


Eigen::Vector3d alan::LedNodelet::get_CoM(vector<Eigen::Vector3d> pts_3d)
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

void alan::LedNodelet::use_pnp_instead(cv::Mat frame, vector<Eigen::Vector2d> pts_2d_detect, vector<Eigen::Vector3d> pts_3d_detect, Sophus::SE3d& pose)
{
    Eigen::Vector3d t;
    Eigen::Matrix3d R;

    solve_pnp_initial_pose(pts_2d_detect, pts_3d_detect, R, t);
    
    //generate noise to validate BA
    Sophus::SE3d pose_;
    pose_ = Sophus::SE3d(R,t);

    optimize(pose_, pts_3d_detect, pts_2d_detect);

    pose = pose_;

}