// bool LED_pts_measurement(
//                 cv::Mat& frame, 
//                 cv::Mat& depth, 
//                 std::vector<Eigen::Vector2d>& pts_2d_detected
//             );

//             bool alan::LedNodelet::LED_pts_measurement(
//     cv::Mat& frame, 
//     cv::Mat& depth, 
//     std::vector<Eigen::Vector2d>& pts_2d_detect
// )
// {
//     pts_2d_detect = LED_extract_POI(frame, depth);
//     // to extract POI point of interest  

//     if(pts_2d_detect.empty())
//     {
//         LED_tracker_initiated_or_tracked = false;
//         return false;
//     }

//     reject_outlier(pts_2d_detect, depth);

//     bool sufficient_pts = get_final_POI(pts_2d_detect);
//     // the former retrieved POI could be noisy,
//     // so get final POI

//     std::get<0>(corres_global_current) = pts_2d_detect.size();
//     std::cout<<"measurement size: "<<pts_2d_detect.size()<<std::endl;
    
//     return sufficient_pts;
// }

//             bool get_final_POI(std::vector<Eigen::Vector2d>& pts_2d_detected);


// bool alan::LedNodelet::get_final_POI(std::vector<Eigen::Vector2d>& pts_2d_detected)
// {
//     double x_min = INFINITY, y_min = INFINITY;
//     double x_max = -INFINITY, y_max = -INFINITY;

//     for(auto what : pts_2d_detected)
//     {
//         x_min = (what.x() < x_min ? what.x() : x_min);        
//         y_min = (what.y() < y_min ? what.y() : y_min);        
//         x_max = (what.x() > x_max ? what.x() : x_max);
//         y_max = (what.y() > y_max ? what.y() : y_max);

//     }
    
//     double image_ROI_width = (x_max - x_min) * 1.5;
//     double image_ROI_height = (y_max - y_min) * 1.5;

//     cv::Rect rect_ROI(x_min, y_min, image_ROI_width, image_ROI_height);

//     cv::Mat ROI_mask = cv::Mat::zeros(
//         frame_initial_thresholded.size(),
//         frame_initial_thresholded.type()
//     );

//     cv::rectangle(ROI_mask, rect_ROI, CV_RGB(255, 255, 255), -1, 8, 0);

//     cv::Mat final_ROI;
//     frame_initial_thresholded.copyTo(final_ROI, ROI_mask);
//     // cv::imshow();

//     // Blob method
//     std::vector<cv::KeyPoint> keypoints_rgb_d, keypoints_rgb;
// 	cv::SimpleBlobDetector::Params params;

// 	params.filterByArea = false;
//     params.filterByColor = false;
// 	params.filterByCircularity = false;
// 	params.filterByConvexity = false;
// 	params.filterByInertia = false;
//     params.minDistBetweenBlobs = 0.01;

// 	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    

//     detector->detect(final_ROI, keypoints_rgb_d);
// 	cv::drawKeypoints( final_ROI, keypoints_rgb_d, im_with_keypoints,CV_RGB(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
   
//     // cv::imshow("final_ROI", final_ROI);
//     // cv::waitKey(10);
        
//     blobs_for_initialize = keypoints_rgb_d;

//     std::vector<cv::Point2f> POI_pts;
//     std::vector<cv::Point2f> centers;
//     cv::Mat labels;

//     for(auto what : keypoints_rgb_d)
//     {
//         POI_pts.emplace_back(cv::Point2f(what.pt.x, what.pt.y));
//     }

//     int no_cluster = (POI_pts.size() > LED_no ? LED_no : POI_pts.size());

//     if(no_cluster < 4)
//     {
//         // cv::imwrite("/home/patty/alan_ws/src/alan/alan_state_estimation/src/test/wrong/final_ROI_gan"+ std::to_string(i)+ ".png", final_ROI);
//         // i++;
//         return false;
//     }

//     cv::kmeans(POI_pts, no_cluster, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1), 8, cv::KMEANS_PP_CENTERS, centers);

//     pts_2d_detected.clear();
//     for(int i = 0; i < centers.size(); i++)
//     {
//         pts_2d_detected.emplace_back(centers[i].x, centers[i].y);
//     }

//     frame_input = im_with_keypoints.clone();

//     // cout<<"final POI:..."<<pts_2d_detected.size()<<endl;
//     if(pts_2d_detected.size() < 4)
//         return false;
//     else
//         return true;
// }



        // //reinitialization
        //     bool reinitialization(std::vector<Eigen::Vector2d> pts_2d_detect, cv::Mat depth);
// bool alan::LedNodelet::reinitialization(std::vector<Eigen::Vector2d> pts_2d_detect, cv::Mat depth)
// {
//     std::get<1>(corres_global_current).clear();
//     std::vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);

//     std::vector<double> norm_of_x_points, norm_of_y_points, norm_of_z_points;

//     for(auto what :  pts_3d_pcl_detect)
//     {
//         norm_of_x_points.push_back(what.x());
//         norm_of_y_points.push_back(what.y());
//         norm_of_z_points.push_back(what.z());
//     }

//     if(pts_3d_pcl_detect.size() == LED_no  //we got LED_no
//         && calculate_MAD(norm_of_x_points) < MAD_x_threshold //no outlier
//         && calculate_MAD(norm_of_y_points) < MAD_y_threshold 
//         && calculate_MAD(norm_of_z_points) < MAD_z_threshold) 
//     {
//         Sophus::SE3d pose;

//         int i = 0;

//         //hsv detect feature
//         cv::cvtColor(hsv, hsv, CV_RGB2HSV);
//         std::vector<bool> g_or_r; //g = true

//         std::vector<int> corres_g;
//         std::vector<int> corres_r;

//         for(int i = 0 ; i < pts_2d_detect.size(); i++)
//         {
//             cv::Point hsv_vertice1 = cv::Point(pts_2d_detect[i].x() - 2 * min_blob_size,
//                                                pts_2d_detect[i].y() - 2 * min_blob_size);
//             cv::Point hsv_vertice2 = cv::Point(pts_2d_detect[i].x() + 2 * min_blob_size,
//                                                pts_2d_detect[i].y() + 2 * min_blob_size);

//             cv::Rect letsgethsv(hsv_vertice1, hsv_vertice2);

//             cv::Mat ROI(hsv, letsgethsv);

//             int size = ROI.cols * ROI.rows;
            
//             double accu = 0;

//             cv::Vec3b hsv_value;

//             for(int i = 0; i < ROI.rows; i++)
//             {
//                 for(int j = 0; j < ROI.cols; j++)
//                 {
//                     hsv_value = ROI.at<cv::Vec3b>(i, j);
                    
//                     if(hsv_value[0] == 0)                    
//                         size = size - 1;                
//                     else
//                         accu = accu + hsv_value[0];
//                 }
//             }  

//             if(accu/size < 100)
//                 corres_g.push_back(i);
//             else   
//                 corres_r.push_back(i);
//         }

//         std::vector<int> corres(LED_no);

//         if(corres_g.size() != 3 || corres_r.size() != 3)
//         {
//             return false;
//         }

//         std::vector<int> final_corres;
//         double error_total = INFINITY;

//         do
//         {
//             do
//             {
//                 corres.clear();
//                 for(auto what : corres_g)
//                     corres.push_back(what);
//                 for(auto what : corres_r)
//                     corres.push_back(what);

//                 std::vector<Eigen::Vector2d> pts_2d_detect_temp;                   

//                 for(auto what : corres)
//                 {
//                     pts_2d_detect_temp.push_back(pts_2d_detect[what]);
//                 }
                                                                        
//                 solve_pnp_initial_pose(pts_2d_detect_temp, pts_on_body_frame);
                
//                 pose_global_sophus = pose_epnp_sophus;

//                 double e = get_reprojection_error(
//                     pts_on_body_frame,
//                     pts_2d_detect_temp,
//                     pose_global_sophus,
//                     false
//                 );

//                 if(e < error_total)
//                 {                    
//                     error_total = e;
//                     final_corres = corres;                   

//                     if(error_total < 5)
//                         break;
//                 }                        
//             } while (next_permutation(corres_r.begin(), corres_r.end()));

//         } while(next_permutation(corres_g.begin(), corres_g.end()));

//         BA_error = error_total;

//         correspondence::matchid corres_temp;
        
//         pts_2d_detect_correct_order.clear();
        
//         for(auto what : final_corres)
//         {
//             corres_temp.detected_indices = what;
//             corres_temp.detected_ornot = true;
//             corres_temp.pts_3d_correspond = pts_3d_pcl_detect[what];            
//             corres_temp.pts_2d_correspond = pts_2d_detect[what];

//             pts_2d_detect_correct_order.push_back(pts_2d_detect[what]);                     

//             std::get<1>(corres_global_current).push_back(corres_temp);
//         }        

//         camOptimize(
//             pose_global_sophus, 
//             pts_on_body_frame, 
//             pts_2d_detect_correct_order,
//             BA_error
//         );
        
//         detect_no = 6;

//         return true;
//     }
//     else        
//         return false;
// }


//depth compensation
            //objects
            // Eigen::Vector3d led_3d_posi_in_camera_frame_depth;




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

// void alan::LedNodelet::reject_outlier(std::vector<Eigen::Vector2d>& pts_2d_detect, cv::Mat depth)
// {
//     std::vector<Eigen::Vector3d> pts_3d_detect = pointcloud_generate(pts_2d_detect, depth);
//     //what is this for?
//     //to get 3d coordinates in body frame, so that 
//     //outlier rejection could be performed
//     int n = pts_3d_detect.size();

//     std::vector<cv::Point3f> pts;
//     std::vector<double> norm_of_x_points;
//     std::vector<double> norm_of_y_points;
//     std::vector<double> norm_of_z_points;

//     for(auto what :  pts_3d_detect)
//     {
//         norm_of_x_points.push_back(what.x());
//         norm_of_y_points.push_back(what.y());
//         norm_of_z_points.push_back(what.z());

//         pts.push_back(cv::Point3f(what.x(), what.y(), what.z()));
//     }

//     cv::Mat labels;
//     std::vector<cv::Point3f> centers;

    
//     if(calculate_MAD(norm_of_x_points) > MAD_x_threshold  
//         || calculate_MAD(norm_of_y_points) > MAD_y_threshold
//         || calculate_MAD(norm_of_z_points) > MAD_z_threshold)
//     {   
//         // cout<<"got some rejection to do"<<endl;
//         cv::kmeans(pts, 2, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1), 8, cv::KMEANS_PP_CENTERS, centers);

//         double d0 = cv::norm(pcl_center_point_wo_outlier_previous - centers[0]);
//         double d1 = cv::norm(pcl_center_point_wo_outlier_previous - centers[1]);

//         std::vector<Eigen::Vector2d> pts_2d_result;
//         std::vector<Eigen::Vector3d> pts_3d_result;

//         if(d0 < d1) //then get index with 0
//         {           
//             for(int i = 0; i < labels.rows; i++)
//             {
//                 if(labels.at<int>(0,i) == 0)
//                 {
//                     pts_2d_result.push_back(pts_2d_detect[i]);
//                     pts_3d_result.push_back(pts_3d_detect[i]); 
//                 }                    
//             }            
//             pcl_center_point_wo_outlier_previous = centers[0];
//         }
//         else
//         {
//             for(int i = 0; i < labels.rows; i++)
//             {

//                 if(labels.at<int>(0,i) == 1)
//                 {                    
//                     pts_2d_result.push_back(pts_2d_detect[i]);
//                     pts_3d_result.push_back(pts_3d_detect[i]);                    
//                 }
//             }
//             pcl_center_point_wo_outlier_previous = centers[1];
//         }
            
//         pts_2d_detect.clear();
//         pts_2d_detect = pts_2d_result;

//         pts_3d_detect.clear();
//         pts_3d_detect = pts_3d_result;

//     }
//     else
//     {
//         cv::Mat temp;
        
//         cv::reduce(pts, temp, 01, CV_REDUCE_AVG);
//         pcl_center_point_wo_outlier_previous = cv::Point3f(temp.at<float>(0,0), temp.at<float>(0,1), temp.at<float>(0,2));

//     }

    
//     led_3d_posi_in_camera_frame_depth = Eigen::Vector3d(
//         pcl_center_point_wo_outlier_previous.x,
//         pcl_center_point_wo_outlier_previous.y,
//         pcl_center_point_wo_outlier_previous.z
//     );

//     // cout<<"reject outlier"<<endl;
//     // cout<<led_3d_posi_in_camera_frame_depth.z()<<endl;


// }