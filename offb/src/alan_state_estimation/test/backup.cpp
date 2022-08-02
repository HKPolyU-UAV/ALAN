pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

double t1 = ros::Time::now().toSec();
// Fill in the CloudIn data
for (auto& point : *cloud_in)
{
    point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    point.z = 1024 * rand() / (RAND_MAX + 1.0f);
}

int temp_i = cloud_in->size() - 1;

for (int i = temp_i; i >=0  ; i--)
{
    cout<<i<<endl;
    cloud_out->push_back(cloud_in->points.at(i));
    cout<<"lala"<<endl;
    cout<<i<<endl<<endl;
}                
// *cloud_out = *cloud_in;

// int i = 0;
// i--;
// cout<<i<<endl<<endl;;


pcl::PointXYZ lala;
lala.x = 0;
lala.y = 0;
lala.z = 0;

cloud_in->emplace_back(lala);

std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;
    
for (auto& point : *cloud_in)
    std::cout << point << std::endl;
    
cout<<endl;


//////////////////////////////////////////////////////////////////////////////////////


for (auto& point : *cloud_out)
    point.x += 0.7f;


std::cout << "Saved " << cloud_out->size () << " data points to output:" << std::endl;
    
for (auto& point : *cloud_out)
    std::cout << point << std::endl;

//////////////////////////////////////////////////////////////////////////////////////

pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>::Ptr rej_ransac(new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>);

pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
icp.setInputSource(cloud_in);
icp.setInputTarget(cloud_out);
icp.addCorrespondenceRejector(rej_ransac);

pcl::PointCloud<pcl::PointXYZ> Final;
// icp.settr

icp.align(Final);

// std::cout << "has converged:" << icp.hasConverged() << " score: " <<
// icp.getFitnessScore() << std::endl;
std::cout << icp.getFinalTransformation() << std::endl;  

cout<<"hi"<<endl;

pcl::PointCloud<pcl::PointXYZ>::Ptr results (new pcl::PointCloud<pcl::PointXYZ>);
*results = Final;
cout<<"hi"<<endl;

pcl::CorrespondencesPtr corresps(new pcl::Correspondences);
pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
est.setInputSource (results);
est.setInputTarget (cloud_out);
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



                Eigen::Vector3d a(0, 1, 2), b(10, 8, 7), c(20, 21, 20), d(40, 38, 41);
                Eigen::Vector3d a_(-1, 0.1, 2), b_(9, 9, 9), c_(19, 19, 20), d_(40, 37, 41);

                vector<Eigen::Vector3d> first;
                first.push_back(a);
                first.push_back(b);
                first.push_back(c);
                first.push_back(d);

                vector<Eigen::Vector3d> second;
                second.push_back(c_);
                second.push_back(d_);
                second.push_back(a_);
                second.push_back(b_);

                correspondence::munkres lala;   
                
                double t1 = ros::Time::now().toSec();          

                LED_v_Detected = lala.solution(first, second);
                
                for(auto what : lala.solution(first, second))
                {
                    cout<<what.detected_indices<<endl;
                }
                
                ;

                // for(auto what : normalization_2d(first, 1, 2))
                // {
                //     cout << what << endl << endl;
                // }


                vector<Eigen::Vector3d> test1 = normalization_2d(first, 1, 2), test2 = normalization_2d(second, 1, 2);

                for(auto what : lala.solution(test1, test2))
                {
                    cout<<what.detected_indices<<endl;
                }


                double t2 = ros::Time::now().toSec();
                cout<<1/(t2-t1)<<" fps"<<endl;