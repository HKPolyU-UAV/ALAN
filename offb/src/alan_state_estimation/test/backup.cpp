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