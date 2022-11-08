#include <iostream>
#include "include/bezier_lib/traj_gen.h"
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_geometry/polyhedron.h>

// #include <decomp_ros_utils/data_ros_utils.h>
// decomp_ros_msg
using namespace std;

Polyhedron3D ros_to_polyhedron(const decomp_ros_msgs::Polyhedron& msg){
  Polyhedron3D poly;
  for(unsigned int i = 0; i < msg.points.size(); i++){
    Vec3f pt(msg.points[i].x,
             msg.points[i].y,
             msg.points[i].z);
    Vec3f n(msg.normals[i].x,
            msg.normals[i].y,
            msg.normals[i].z);
    poly.add(Hyperplane3D(pt, n));
  }
  return poly;
}

vec_E<Polyhedron3D> ros_to_polyhedron_array(const decomp_ros_msgs::PolyhedronArray& msg) {
  vec_E<Polyhedron3D> polys(msg.polyhedrons.size());

  for(size_t i = 0; i < msg.polyhedrons.size(); i++)
    polys[i] = ros_to_polyhedron(msg.polyhedrons[i]);

  return polys;
}

decomp_ros_msgs::Polyhedron polyhedron_to_ros(const Polyhedron2D& poly){
  decomp_ros_msgs::Polyhedron msg;
  for (const auto &p : poly.hyperplanes()) {
    geometry_msgs::Point pt, n;
    pt.x = p.p_(0);
    pt.y = p.p_(1);
    pt.z = 0;
    n.x = p.n_(0);
    n.y = p.n_(1);
    n.z = 0;
    msg.points.push_back(pt);
    msg.normals.push_back(n);
  }

  geometry_msgs::Point pt1, n1;
  pt1.x = 0, pt1.y = 0, pt1.z = 0.01;
  n1.x = 0, n1.y = 0, n1.z = 1;
  msg.points.push_back(pt1);
  msg.normals.push_back(n1);

  geometry_msgs::Point pt2, n2;
  pt2.x = 0, pt2.y = 0, pt2.z = -0.01;
  n2.x = 0, n2.y = 0, n2.z = -1;
  msg.points.push_back(pt2);
  msg.normals.push_back(n2);

  return msg;
}

decomp_ros_msgs::Polyhedron polyhedron_to_ros(const Polyhedron3D& poly)
{
  decomp_ros_msgs::Polyhedron msg;
  for (const auto &p : poly.hyperplanes()) {
    geometry_msgs::Point pt, n;
    pt.x = p.p_(0);
    pt.y = p.p_(1);
    pt.z = p.p_(2);
    n.x = p.n_(0);
    n.y = p.n_(1);
    n.z = p.n_(2);
    msg.points.push_back(pt);
    msg.normals.push_back(n);
  }

  return msg;
}

template <int Dim>
decomp_ros_msgs::PolyhedronArray polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>>& vs){
  decomp_ros_msgs::PolyhedronArray msg;
  for (const auto &v : vs)
    msg.polyhedrons.push_back(polyhedron_to_ros(v));
  return msg;
}

int main(int argc, char** argv)
{
    Polyhedron3D polyh_msg;
    // polyh_msg.
    

	ros::init(argc, argv, "lala");
    ros::NodeHandle nh;

    ros::Publisher poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);
    
    decomp_ros_msgs::PolyhedronArray poly_msg;
    poly_msg.header.frame_id = "map";
    poly_pub.publish(poly_msg);

	
	int n_order = 6;
    int m = 1;
    int d_order = 3;

    vector<double> s;

    for(int i = 0; i < m; i++)
        s.push_back(1);
    
    alan_traj::endpt_cond start;
    start.p_ = 50;
    start.v_ = 0;
    start.a_ = 0;
    start.j_ = 0;

    alan_traj::endpt_cond end;
    end.p_ = 60;
    end.v_ = 0;
    end.a_ = 0;
    end.j_ = 0;

    vector<alan_traj::corridor> cube_list;
    alan_traj::corridor cube;

    cube.x_max = 100;
    cube.x_min = 0;//0;
    //
    cube_list.push_back(cube);

    // cube.x_max = 150;
    // cube.x_min = 50;//-OsqpEigen::INFTY;//50;
    // cube_list.push_back(cube);

    // cube.x_max = 230;
    // cube.x_min = 130;//-OsqpEigen::INFTY;//130;
    // cube_list.push_back(cube);
    
    // cube.x_max = 300;
    // cube.x_min = 200;//-OsqpEigen::INFTY;//200;
    // cube_list.push_back(cube);
    
    // cube.x_max = 330;
    // cube.x_min = 230;//-OsqpEigen::INFTY;//230;
    // cube_list.push_back(cube);

    alan_traj::dynamic_constraints d_constraints;
    d_constraints.v_max =  150;
    d_constraints.v_min = -150;//OsqpEigen::INFTY;//-150;
    d_constraints.a_max =  200;
    d_constraints.a_min = -200;//OsqpEigen::INFTY;//-200;
    d_constraints.j_max =  400;
    d_constraints.j_min = -400;//OsqpEigen::INFTY;//-400;

    alan_traj::bezier_info b_info;
    alan_traj::bezier_constraints b_constraints;

    b_info.n_order = n_order;
    b_info.m = m;
    b_info.d_order = d_order;
    b_info.s = s;

    b_constraints.start = start;
    b_constraints.end = end;
    b_constraints.cube_list = cube_list;
    b_constraints.d_constraints = d_constraints;

    double t0 = ros::Time::now().toSec();

	alan_traj::traj_gen traj(b_info, b_constraints);

	traj.solve_opt();

    double t1 = ros::Time::now().toSec();

    cout<<"fps: "<<1/(t1-t0)<<endl;

	return 0;
}