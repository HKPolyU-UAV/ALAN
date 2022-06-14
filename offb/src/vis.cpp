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

#include "include/essential.h"
#include "include/dbscan.hpp"


static double eps = 0.75;
static double minPts = 4;
static vector<Eigen::Vector3d> pcl_list;
static vector<pts> pcl_result;


void readfile(string filelocation)
{
    fstream file(filelocation, ios::in);
    string line, value;    
    if(file.is_open())
    {    
        Eigen::Vector3d temp_vector;

        while(getline(file, line))
		{
            stringstream str(line);
            int index = 0;
            while(getline(str, value, ','))
            {
                double temp = stod(value);
                if(index<3)
                {
                    temp_vector(index) = temp;
                }
                index ++ ;
            }
            pcl_list.push_back(temp_vector);                                            
		}
    }
}

void writefile(string filelocation)
{
    ofstream save(filelocation, ios::app);
    for(const auto stuff : pcl_result)
    {
        save << stuff.x << " " << stuff.y << " "  << stuff.z << " " << " id: " << stuff.clid <<endl;        
    }
    save.close();
}

void run_algo()
{
    dbscan cluster(eps, minPts);
    pcl_result = cluster.execute(pcl_list);
}

int main(int argc, char** argv)
{
    cout<<"Object detection..."<<endl;

    ros::init(argc, argv, "dbscan");
    ros::NodeHandle nh;
    ros::Publisher rviz_visual 
        = nh.advertise <visualization_msgs::Marker>("gt_points",10);


    readfile("/home/patrick/dbscan/src/include/data.dat");
    
    run_algo();

    // writefile("/home/patrick/dbscan/src/include/result.dat");

    visualization_msgs::Marker edge_points;


    edge_points.header.frame_id = "map";
    edge_points.header.stamp = ros::Time::now();
    edge_points.ns = "GT_points";
    edge_points.id = 0;
    edge_points.action = visualization_msgs::Marker::ADD;
    edge_points.pose.orientation.w = 1.0;
    edge_points.type = visualization_msgs::Marker::SPHERE_LIST;
    edge_points.scale.x = edge_points.scale.y = edge_points.scale.z = 0.05;
    std_msgs::ColorRGBA color_for_edge;
    edge_points.color.a=1;
    edge_points.color.g=0;
    edge_points.color.r=1;
    edge_points.color.b=0;

    for(auto pts : pcl_result)
    {
        std_msgs::ColorRGBA color;
        int temp_color = -1;
        geometry_msgs::Point pt;
        pt.x = pts.x;
        pt.y = pts.y;
        pt.z = pts.z;
        temp_color = pts.clid;
        
        // switch(temp_color)
        // {
        //     case 1:
        //         color.a = 1;
        //         color.r = 1;
        //         color.g = 0;
        //         color.b = 0;
        //         break;
        //     case 2:
        //         color.a = 1;
        //         color.r = 0;
        //         color.g = 1;
        //         color.b = 0;
        //         break;
        //     case 3:
        //         color.a = 1;
        //         color.r = 0;
        //         color.g = 0;
        //         color.b = 1;
        //         break;                
        //     case 4:
        //         color.a = 1;
        //         color.r = 1;
        //         color.g = 1;
        //         color.b = 0;
        //         break;
        //     case 5:
        //         color.a = 1;
        //         color.r = 0;
        //         color.g = 1;
        //         color.b = 1;
        //         break;
        //     case 6:
        //         color.a = 1;
        //         color.r = 1;
        //         color.g = 0;
        //         color.b = 1;
        //         break;
        //     case 7:
        //         color.a = 1;
        //         color.r = 1;
        //         color.g = 1;
        //         color.b = 1;
        //         break;
        //     default:
        //         color.a = 1;
        //         color.r = 0;
        //         color.g = 0;
        //         color.b = 0;
            
        // }

        edge_points.points.push_back(pt);      
        // edge_points.colors.push_back(color);
    }

    while(ros::ok())
    {   
        ros::spinOnce();
        rviz_visual.publish(edge_points);
    }
    ros::spin();
    return 0;
}




