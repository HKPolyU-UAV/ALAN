#include <sstream>
#include <cmath>
#include <string>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"

#include "include/run_yolo.h"
#include "include/dbscan.hpp"
#include "include/roshandle.hpp"

static double eps = 0.75;
static double minPts = 4;
static vector<Eigen::Vector3d> pcl_list;
static vector<pts> pcl_result;

int main(int argc, char** argv)
{
    cout<<"Object detection..."<<endl;

    ros::init(argc, argv, "dbscan");
    SubAndPub subandpub_ob;
    

    while(ros::ok())
    {   
        ros::spinOnce();
        // // rviz_visual.publish(edge_points);
        // cout<<subandpub_ob.depthmat.size<<endl;
        // cout<<subandpub_ob.depthmat.channels()<<endl<<endl;;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr testtt;
        // sensor_msgs::PointCloud2 tt;

        // if(subandpub_ob.output.data.size() != 0)
        //     cout << subandpub_ob.output.data[0] << endl;
        cout<<subandpub_ob.depthmat.size<<endl;
        
        
    }
    ros::spin();
    return 0;
}





