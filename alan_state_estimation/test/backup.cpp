//pcl icp
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

// double t1 = ros::Time::now().toSec();
// // Fill in the CloudIn data
// for (auto& point : *cloud_in)
// {
//     point.x = 1024 * rand() / (RAND_MAX + 1.0f);
//     point.y = 1024 * rand() / (RAND_MAX + 1.0f);
//     point.z = 1024 * rand() / (RAND_MAX + 1.0f);
// }

// int temp_i = cloud_in->size() - 1;

// for (int i = temp_i; i >=0  ; i--)
// {
//     cout<<i<<endl;
//     cloud_out->push_back(cloud_in->points.at(i));
//     cout<<"lala"<<endl;
//     cout<<i<<endl<<endl;
// }                
// // *cloud_out = *cloud_in;

// // int i = 0;
// // i--;
// // cout<<i<<endl<<endl;;


// pcl::PointXYZ lala;
// lala.x = 0;
// lala.y = 0;
// lala.z = 0;

// cloud_in->emplace_back(lala);

// std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;
    
// for (auto& point : *cloud_in)
//     std::cout << point << std::endl;
    
// cout<<endl;


// //////////////////////////////////////////////////////////////////////////////////////


// for (auto& point : *cloud_out)
//     point.x += 0.7f;


// std::cout << "Saved " << cloud_out->size () << " data points to output:" << std::endl;
    
// for (auto& point : *cloud_out)
//     std::cout << point << std::endl;

// //////////////////////////////////////////////////////////////////////////////////////

// pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>::Ptr rej_ransac(new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>);

// pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
// icp.setInputSource(cloud_in);
// icp.setInputTarget(cloud_out);
// icp.addCorrespondenceRejector(rej_ransac);

// pcl::PointCloud<pcl::PointXYZ> Final;
// // icp.settr

// icp.align(Final);

// // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
// // icp.getFitnessScore() << std::endl;
// std::cout << icp.getFinalTransformation() << std::endl;  

// cout<<"hi"<<endl;

// pcl::PointCloud<pcl::PointXYZ>::Ptr results (new pcl::PointCloud<pcl::PointXYZ>);
// *results = Final;
// cout<<"hi"<<endl;

// pcl::CorrespondencesPtr corresps(new pcl::Correspondences);
// pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
// est.setInputSource (results);
// est.setInputTarget (cloud_out);
// est.determineCorrespondences (*corresps, 1.0);

// for (auto& what : *corresps)
// {
//     cout<<what.index_match<<endl;               
// }


// double t2 = ros::Time::now().toSec();
// pcl::IndicesPtr match_id = icp.getIndices();


// // for (auto& what : Final)
// // {
// //     cout<<what<<endl;;
// // }
// cout << endl << "Hz: " << 1 / (t2-t1) <<endl;


//munkres test
                // Eigen::Vector3d a(0, 1, 2), b(10, 8, 7), c(20, 21, 20), d(40, 38, 41);
                // Eigen::Vector3d a_(-1, 0.1, 2), b_(9, 9, 9), c_(19, 19, 20), d_(40, 37, 41);

                // vector<Eigen::Vector3d> first;
                // first.push_back(a);
                // first.push_back(b);
                // first.push_back(c);
                // first.push_back(d);

                // vector<Eigen::Vector3d> second;
                // second.push_back(c_);
                // // second.push_back(d_);
                // second.push_back(a_);
                // second.push_back(b_);

                // correspondence::munkres lala;   
                
                // double t1 = ros::Time::now().toSec();          

                // LED_v_Detected = lala.solution(first, second);
                
                // for(auto what : lala.solution(first, second))
                // {
                //     cout<<what.detected_indices<<"   ";
                //     cout<<what.detected_ornot<<endl;
                // }
                
                // ;

                // // for(auto what : normalization_2d(first, 1, 2))
                // // {
                // //     cout << what << endl << endl;
                // // }




                // double t2 = ros::Time::now().toSec();
                // cout<<1/(t2-t1)<<" fps"<<endl;

//contour
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



    
    

    // cout<<pts.size()<<endl;

    // cout<<normsssss<<endl;


   
    // cout<<"out"<<endl;
    // vector<int> v_index(norm_of_points.size());
    
    // iota(v_index.begin(),v_index.end(),0); //Initializing
    // sort(v_index.begin(),v_index.end(), [&](int i,int j) {return norm_of_points[i]< norm_of_points[j];} );//this is lambda

    // vector<double> norm_of_points_sorted;


    // for(auto what : v_index)
    // {
    //     cout<<norm_of_points[what]<<endl;
    //     norm_of_points_sorted.push_back(norm_of_points[what]);
    //     cout<<endl;
    // }
    // // sort(norm_of_points.begin(), norm_of_points.end());

    // // for(auto what : v_index)
    
    // int index_0 = 0;
    // int index_100 = norm_of_points_sorted.size();

    // int index_50 = ((index_100 - index_0 + 1) + 1) / 2 - 1;
    // int index_25 = ((index_50 - index_0 + 1) + 1) / 2 - 1;
    // int index_75 = ((index_100 - index_50 + 1) + 1) / 2 - 1;

    // double IQR = norm_of_points_sorted[index_75] - norm_of_points_sorted[index_25];

    // double lower = norm_of_points_sorted[index_25] - 1.5 * IQR, 
    //        upper = norm_of_points_sorted[index_75] + 1.5 * IQR;

    // cout<<"lower: "<<lower<<endl;
    // cout<<"upper: "<<upper<<endl;

    // for(auto what : v_index)
    // {
    //     if(norm_of_points[what] < lower || norm_of_points[what] > upper)
    //         pts_3d_detect.erase(pts_3d_detect.begin() + what);        
    // }          



    
                // // x = [-0.36926538, -0.35783651, -0.30663395, -0.37761885, -0.28259838, -0.32332534]
                // // y = [-0.17193949, -0.17355335,  -0.17994796,  -0.1793365, -0.19169508,  -0.20557153]
                // // z = [0.71600002, 0.71799999, 0.72549999, 0.68800002,0.70550001, 0.727]
            
                // //test initialization
                // vector<Eigen::Vector2d> pts_2d_detect;
                // vector<Eigen::Vector3d> pts_3d_detect;

                // XmlRpc::XmlRpcValue pts_2d_list, pts_3d_list;
                
                // nh.getParam("/alan_pose/pts_2d_list", pts_2d_list); 
                // nh.getParam("/alan_pose/pts_3d_list", pts_3d_list); 
                // cout<<"hi"<<endl;

                // cout<<pts_2d_list.size()<<endl;
                // cout<<pts_3d_list.size()<<endl;


                // for(int i = 0; i < pts_2d_list.size(); i++)
                // {
                //     Eigen::Vector2d temp1(pts_2d_list[i]["x"], pts_2d_list[i]["y"]);                
                //     Eigen::Vector3d temp2(pts_3d_list[i]["x"], pts_3d_list[i]["y"], pts_3d_list[i]["z"]);

                //     pts_2d_detect.push_back(temp1);
                //     pts_3d_detect.push_back(temp2);

                // }   

                // //now I have pts_2d_detect pts_3d_detect and pts_on_body_frame;        
                

                // double t1 = ros::Time::now().toSec();
                // int i = 0;
                
                // vector<int> test;
                
                // for(int i = 0; i < 6; i++)
                // {
                //     test.push_back(i);
                // }

                // double error_total = INFINITY;
                
                // Eigen::Matrix3d R;
                // Eigen::Vector3d t;
                // vector<int> final_permutation;

                // do 
                // {       
                //     vector<Eigen::Vector2d> pts_2d_detect_temp;       
                //     vector<Eigen::Vector3d> pts_3d_detect_temp;     

                //     for(auto what : test)
                //     {
                //         pts_2d_detect_temp.push_back(pts_2d_detect[what]);
                //         pts_3d_detect_temp.push_back(pts_3d_detect[what]);
                //     }
                                                            
                //     get_initial_pose(pts_2d_detect_temp, pts_on_body_frame, R, t);
                //     // solveicp_svd(pts_3d_detect_temp, pts_on_body_frame, R, t);
                    
                //     Sophus::SE3d pose(R, t);

                //     Eigen::Vector2d reproject, error; 
                //     double e = 0;

                //     for(int i = 0 ; i < pts_on_body_frame.size(); i++)
                //     {
                //         reproject = reproject_3D_2D(pts_on_body_frame[i], pose);  
                //         error = pts_2d_detect_temp[i] - reproject;
                //         e = e + error.norm();
                //     }

                //     if(e < error_total)
                //     {                    
                //         error_total = e;
                //         final_permutation = test;
                //         if(error_total < 5)
                //             break;
                //     }
                    
                // }
                // while(next_permutation(test.begin(), test.end()));

                // // vector<Eigen::Vector3d> pts_3d_detect_temp;
                // // pts_3d_detect_temp.push_back(pts_3d_detect[4]);
                // // pts_3d_detect_temp.push_back(pts_3d_detect[2]);
                // // pts_3d_detect_temp.push_back(pts_3d_detect[1]);
                // // pts_3d_detect_temp.push_back(pts_3d_detect[5]);
                // // pts_3d_detect_temp.push_back(pts_3d_detect[0]);
                // // pts_3d_detect_temp.push_back(pts_3d_detect[3]);

                // // vector<Eigen::Vector2d> pts_2d_detect_temp;
                // // pts_2d_detect_temp.push_back(pts_2d_detect[0]);
                // // pts_2d_detect_temp.push_back(pts_2d_detect[1]);
                // // pts_2d_detect_temp.push_back(pts_2d_detect[2]);
                // // pts_2d_detect_temp.push_back(pts_2d_detect[3]);
                // // pts_2d_detect_temp.push_back(pts_2d_detect[4]);
                // // pts_2d_detect_temp.push_back(pts_2d_detect[5]);

                // // solveicp_svd(pts_3d_detect_temp, pts_on_body_frame, R, t);

                // // Sophus::SE3d pose(R, t);

                // // Eigen::Vector2d reproject, error; 
                // // double e = 0;

                // // for(int i = 0 ; i < pts_on_body_frame.size(); i++)
                // // {
                // //     reproject = reproject_3D_2D(pts_on_body_frame[i], pose);  
                // //     error = pts_2d_detect[final_permutation[i]] - reproject;
                // //     e = e + error.norm();
                // // }
                // // cout<<e<<endl;

                // cout<<"final: "<<error_total<<endl;
                // for(auto what : final_permutation)
                //     cout<<what;
                // cout<<endl;
                // double t2 = ros::Time::now().toSec();

                // cout<<"Hz: "<< 1 / (t2-t1) <<endl;


 // x = [-0.36926538, -0.35783651, -0.30663395, -0.37761885, -0.28259838, -0.32332534]
                // y = [-0.17193949, -0.17355335,  -0.17994796,  -0.1793365, -0.19169508,  -0.20557153]
                // z = [0.71600002, 0.71799999, 0.72549999, 0.68800002,0.70550001, 0.727]
            
                //test initialization
                // vector<Eigen::Vector2d> pts_2d_detect;
                // vector<Eigen::Vector3d> pts_3d_detect;
                // vector<Eigen::Vector2d> pts_2d_detect_previous;

                // XmlRpc::XmlRpcValue pts_2d_list, pts_3d_list, pts_2d_list_previous;
                
                // nh.getParam("/alan_pose/pts_2d_list", pts_2d_list); 
                // nh.getParam("/alan_pose/pts_3d_list", pts_3d_list); 
                // nh.getParam("/alan_pose/pts_2d_list_previous", pts_2d_list_previous);            


                // for(int i = 0; i < pts_2d_list.size(); i++)
                // {
                //     Eigen::Vector2d temp1(pts_2d_list[i]["x"], pts_2d_list[i]["y"]);                                                        
                //     pts_2d_detect.push_back(temp1);                    
                // }   

                // for(int i = 0; i < pts_3d_list.size(); i++)
                // {
                //     Eigen::Vector3d temp2(pts_3d_list[i]["x"], pts_3d_list[i]["y"], pts_3d_list[i]["z"]);
                //     pts_3d_detect.push_back(temp2);
                // }

                // for(int i = 0; i < pts_2d_list_previous.size(); i++)
                // {
                //     Eigen::Vector2d temp3(pts_2d_list_previous[i]["x"], 
                //                             pts_2d_list_previous[i]["y"]);
                //     pts_2d_detect_previous.push_back(temp3);
                // }


                // for(auto what : pts_2d_detect_previous)
                // {
                //     correspondence::matchid temp;
                //     cout<<what<<endl;
                //     temp.pts_2d_correspond = what;
                //     corres_global.push_back(temp);                
                // }

                // cout<<corres_global.size()<<endl;

                // correspondence_search(pts_3d_detect, pts_2d_detect);
                
                // cout<<"hiiiiiiiiiiiii"<<endl;

                // for(auto what : corres_global)
                // {
                //     cout<<what.detected_indices<<" ";
                //     cout<<what.detected_ornot<<endl;
                //     // cout<<what.pts_3d_correspond<<endl;
                //     // cout<<what.pts_2d_correspond<<endl;
                //     cout<<"end..................."<<endl;
                // }

                //now I have pts_2d_detect pts_3d_detect and pts_on_body_frame;        
                
                // x = [ 202 221 231 265 272 285]
                // y = [ 248 260 262 246 268 262]

                // x = [ 121 200 154 194 143 211]
                // y = [ 227 256 245 194 242 250]

                // vector<Eigen::Vector2d> test1;
                // vector<Eigen::Vector2d> test2;

                // test1.push_back(Eigen::Vector2d(202, 248));
                // test1.push_back(Eigen::Vector2d(221, 260));
                // test1.push_back(Eigen::Vector2d(231, 262));
                // test1.push_back(Eigen::Vector2d(265, 246));
                // test1.push_back(Eigen::Vector2d(272, 268));
                // test1.push_back(Eigen::Vector2d(285, 262));

                // test2.push_back(Eigen::Vector2d(121, 227));
                // test2.push_back(Eigen::Vector2d(143, 242));
                // test2.push_back(Eigen::Vector2d(154, 245));
                // test2.push_back(Eigen::Vector2d(194, 194));
                // test2.push_back(Eigen::Vector2d(200, 256));
                // test2.push_back(Eigen::Vector2d(221, 250));

                // vector<Eigen::Vector2d> final1;
                // vector<Eigen::Vector2d> final2;

                // final1 = pts_2d_normlization(test1);
                // final2 = pts_2d_normlization(test2);


                // hungarian.solution(final1, final2);

                // for(auto what : hungarian.id_match)
                // {
                //     cout<<what.detected_indices<<endl;
                // }                


// Eigen::Vector2d a(0, 1), b(10, 8), c(20, 21), d(40, 38);
                // Eigen::Vector2d a_(-1, 0.1), b_(9, 9), c_(19, 19), d_(40, 37);

                // vector<Eigen::Vector2d> first;
                // first.push_back(a);
                // first.push_back(b);
                // first.push_back(c);
                // first.push_back(d);

                // vector<Eigen::Vector2d> second;
                // second.push_back(c_);
                // // second.push_back(d_);
                // second.push_back(a_);
                // second.push_back(b_);

                // correspondence::munkres lala;   
                
                // double t1 = ros::Time::now().toSec();          

                // LED_v_Detected = lala.solution(first, second);
                
                // for(auto what : lala.solution(first, second))
                // {
                //     cout<<what.detected_indices<<"   ";
                //     cout<<what.detected_ornot<<endl;
                // }
                
                // ;

                // for(auto what : normalization_2d(first, 1, 2))
                // {
                //     cout << what << endl << endl;
                // }
// iota(corres.begin(), corres.end(), 0); //initiate corresponding

        
        


        // do 
        // {       
        //     vector<Eigen::Vector2d> pts_2d_detect_temp;       

        //     for(auto what : corres)
        //     {
        //         pts_2d_detect_temp.push_back(pts_2d_detect[what]);
        //     }
                                                    
        //     solve_pnp_initial_pose(pts_2d_detect_temp, pts_on_body_frame, R, t);
            
        //     pose = Sophus::SE3d(R, t);

        //     Eigen::Vector2d reproject, error; 
        //     double e = 0;

        //     for(int i = 0 ; i < pts_on_body_frame.size(); i++)
        //     {
        //         reproject = reproject_3D_2D(pts_on_body_frame[i], pose);  
        //         error = pts_2d_detect_temp[i] - reproject;
        //         e = e + error.norm();
        //     }

        //     if(e < error_total)
        //     {                    
        //         error_total = e;
        //         final_corres = corres;
        //         pose_global = pose;
        //         if(error_total < 5)
        //             break;
        //     }
            
        // }
        // while(next_permutation(corres.begin(), corres.end())); //for all permutations