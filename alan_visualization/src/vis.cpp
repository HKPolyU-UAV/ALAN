#include <iostream>
#include "include/essential.h"
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_geometry/polyhedron.h>

#include "alan_visualization/PolyhedronArray.h"
#include "alan_landing_planning/Traj.h"


// #include <decomp_ros_utils/data_ros_utils.h>
// decomp_ros_msg
using namespace std;

static decomp_ros_msgs::PolyhedronArray sfc_pub_vis_object_polyh;
static decomp_ros_msgs::Polyhedron sfc_pub_vis_object_tangent;
static ros::Publisher poly_pub;

void sfc_msg_sub(const alan_visualization::Polyhedron::ConstPtr & msg)
{
    geometry_msgs::Point temp_sfc_p, temp_sfc_n;

    sfc_pub_vis_object_polyh.polyhedrons.clear();   
    sfc_pub_vis_object_polyh.header.frame_id = "map";


    for(auto what : msg->PolyhedronTangentArray)
    {
        sfc_pub_vis_object_tangent.points.clear();
        sfc_pub_vis_object_tangent.normals.clear();

        temp_sfc_p.x = what.pt.X;
        temp_sfc_p.y = what.pt.Y;
        temp_sfc_p.z = what.pt.Z;

        temp_sfc_n.x = what.n.X;
        temp_sfc_n.y = what.n.Y;
        temp_sfc_n.z = what.n.Z;

        sfc_pub_vis_object_tangent.points.push_back(temp_sfc_p);
        sfc_pub_vis_object_tangent.normals.push_back(temp_sfc_n);
        sfc_pub_vis_object_polyh.polyhedrons.push_back(sfc_pub_vis_object_tangent);
        
    }

    poly_pub.publish(sfc_pub_vis_object_polyh);

}

int main(int argc, char** argv)
{
    Polyhedron3D polyh_msg;
    // polyh_msg.
    

	ros::init(argc, argv, "lala");
    ros::NodeHandle nh;

    ros::Subscriber sfc_sub = nh.subscribe<alan_visualization::Polyhedron>("/alan/sfc", 1, sfc_msg_sub);

    poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);
    
    

    ros::spin();

	

	return 0;
}









// Polyhedron3D ros_to_polyhedron(const decomp_ros_msgs::Polyhedron& msg){
//   Polyhedron3D poly;
//   for(unsigned int i = 0; i < msg.points.size(); i++){
//     Vec3f pt(msg.points[i].x,
//              msg.points[i].y,
//              msg.points[i].z);
//     Vec3f n(msg.normals[i].x,
//             msg.normals[i].y,
//             msg.normals[i].z);
//     poly.add(Hyperplane3D(pt, n));
//   }
//   return poly;
// }

// vec_E<Polyhedron3D> ros_to_polyhedron_array(const decomp_ros_msgs::PolyhedronArray& msg) {
//   vec_E<Polyhedron3D> polys(msg.polyhedrons.size());

//   for(size_t i = 0; i < msg.polyhedrons.size(); i++)
//     polys[i] = ros_to_polyhedron(msg.polyhedrons[i]);

//   return polys;
// }

// decomp_ros_msgs::Polyhedron polyhedron_to_ros(const Polyhedron2D& poly){
//   decomp_ros_msgs::Polyhedron msg;
//   for (const auto &p : poly.hyperplanes()) {
//     geometry_msgs::Point pt, n;
//     pt.x = p.p_(0);
//     pt.y = p.p_(1);
//     pt.z = 0;
//     n.x = p.n_(0);
//     n.y = p.n_(1);
//     n.z = 0;
//     msg.points.push_back(pt);
//     msg.normals.push_back(n);
//   }

//   geometry_msgs::Point pt1, n1;
//   pt1.x = 0, pt1.y = 0, pt1.z = 0.01;
//   n1.x = 0, n1.y = 0, n1.z = 1;
//   msg.points.push_back(pt1);
//   msg.normals.push_back(n1);

//   geometry_msgs::Point pt2, n2;
//   pt2.x = 0, pt2.y = 0, pt2.z = -0.01;
//   n2.x = 0, n2.y = 0, n2.z = -1;
//   msg.points.push_back(pt2);
//   msg.normals.push_back(n2);

//   return msg;
// }

// decomp_ros_msgs::Polyhedron polyhedron_to_ros(const Polyhedron3D& poly)
// {
//   decomp_ros_msgs::Polyhedron msg;
//   for (const auto &p : poly.hyperplanes()) {
//     geometry_msgs::Point pt, n;
//     pt.x = p.p_(0);
//     pt.y = p.p_(1);
//     pt.z = p.p_(2);
//     n.x = p.n_(0);
//     n.y = p.n_(1);
//     n.z = p.n_(2);
//     msg.points.push_back(pt);
//     msg.normals.push_back(n);
//   }

//   return msg;
// }

// template <int Dim>
// decomp_ros_msgs::PolyhedronArray polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>>& vs){
//   decomp_ros_msgs::PolyhedronArray msg;
//   for (const auto &v : vs)
//     msg.polyhedrons.push_back(polyhedron_to_ros(v));
//   return msg;
// }

// decomp_ros_msgs::Polyhedron msg;
//     decomp_ros_msgs::PolyhedronArray poly_msg;

//     geometry_msgs::Point pt, n;


//     pt.x = 1;
//     pt.y = 1;
//     pt.z = 1;
//     n.x = 1;
//     n.y = 1;
//     n.z = 0;

//     msg.points.push_back(pt);
//     msg.normals.push_back(n);

//     pt.x = 0;
//     pt.y = 0;
//     pt.z = 1;

//     n.x = -1;
//     n.y = -1;
//     n.z = 0;

//     msg.points.push_back(pt);
//     msg.normals.push_back(n);

//     pt.x = 0;
//     pt.y = 0;
//     pt.z = 4;
//     n.x = 0;
//     n.y = 0;
//     n.z = 1;

//     msg.points.push_back(pt);
//     msg.normals.push_back(n);

//     pt.x = 0;
//     pt.y = 0;
//     pt.z = -1;
//     n.x = 0;
//     n.y = 0;
//     n.z = -1;

//     msg.points.push_back(pt);
//     msg.normals.push_back(n);

//     pt.x = 2;
//     pt.y = -2;
//     pt.z = -1;
//     n.x = 1;
//     n.y = -1;
//     n.z = 0;

//     msg.points.push_back(pt);
//     msg.normals.push_back(n);

//     pt.x = -2;
//     pt.y = 2;
//     pt.z = -1;
//     n.x = -1;
//     n.y = 1;
//     n.z = 0;

//     msg.points.push_back(pt);
//     msg.normals.push_back(n);


//     poly_msg.polyhedrons.push_back(msg);
//     poly_msg.header.frame_id = "map";
//     poly_pub.publish(poly_msg);