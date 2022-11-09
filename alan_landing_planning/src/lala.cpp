
// This executable is only for algorithm validation, 
// DO NOT run it on a real physical UAV platform!

// Created on: Nov 9 2022
// Author: Li-yu LO pattylo

#include <iostream>
#include "include/bezier_lib/traj_gen.h"

#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_geometry/polyhedron.h>

#include "alan_visualization/Polyhedron.h"
#include "alan_visualization/Tangent.h"
#include <mavros_msgs/AttitudeTarget.h>

// #include <decomp_ros_utils/data_ros_utils.h>
// decomp_ros_msg
using namespace std;


int main(int argc, char** argv)
{
    Polyhedron3D polyh_msg;
    // polyh_msg.
    

    ros::init(argc, argv, "lala");
    ros::NodeHandle nh;

    ros::Publisher poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("/polyhedron_array", 1, true);
    ros::Publisher polyh_pub = nh.advertise<alan_visualization::Polyhedron>("/alan_visualization/polyhedron", 1, true);

    // test.n.X

    decomp_ros_msgs::PolyhedronArray poly_msg;
    
    poly_msg.header.frame_id = "map";
    poly_pub.publish(poly_msg);

    alan_traj::bezier_info b_traj_info;
    
    b_traj_info.dimension = 2;
    b_traj_info.n_order = 7;
    b_traj_info.m = 5;
    b_traj_info.d_order = 3;

    for(int i = 0; i < b_traj_info.m; i++)
        b_traj_info.s.push_back(1);

    //

    //equality constraint:
    alan_traj::endpt start_2d;
    start_2d.posi(0) = 50;
    start_2d.velo(0) = 0;
    start_2d.accl(0) = 0;
    start_2d.jerk(0) = 0;

    start_2d.posi(1) = 50;
    start_2d.velo(1) = 0;
    start_2d.accl(1)= 0;
    start_2d.jerk(1) = 0;

    alan_traj::endpt end_2d;
    end_2d.posi(0) = 280;
    end_2d.velo(0) = 0;
    end_2d.accl(0) = 0;
    end_2d.jerk(0) = 0;

    end_2d.posi(1) = 0;
    end_2d.velo(1) = 0;
    end_2d.accl(1) = 0;
    end_2d.jerk(1) = 0;



    //inequality constraints:

    //kinematic constraints

    vector<alan_visualization::Polyhedron> corridor;

    alan_visualization::Polyhedron temp_polyh;
    alan_visualization::Tangent temp_tangent;



    //1st corridor: 4 plane
    temp_polyh.PolyhedronTangentArray.clear();

    temp_tangent.pt.X = 50;
    temp_tangent.pt.Y = 150;

    temp_tangent.n.X = 1;
    temp_tangent.n.Y = 1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

    
    temp_tangent.pt.X = 50;
    temp_tangent.pt.Y = -50;

    temp_tangent.n.X = -1;
    temp_tangent.n.Y = -1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


    temp_tangent.pt.X = 150;
    temp_tangent.pt.Y = 50;

    temp_tangent.n.X = 1;
    temp_tangent.n.Y = -1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


    temp_tangent.pt.X = -50;
    temp_tangent.pt.Y = 50;

    temp_tangent.n.X = -1;
    temp_tangent.n.Y = 1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

    corridor.push_back(temp_polyh);

    //2nd corridor: 4 plane

    temp_polyh.PolyhedronTangentArray.clear();
    
    temp_tangent.pt.X = 100;
    temp_tangent.pt.Y = 220;

    temp_tangent.n.X = 1;
    temp_tangent.n.Y = 1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

    
    temp_tangent.pt.X = 100;
    temp_tangent.pt.Y = 20;

    temp_tangent.n.X = -1;
    temp_tangent.n.Y = -1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


    temp_tangent.pt.X = 200;
    temp_tangent.pt.Y = 120;

    temp_tangent.n.X = 1;
    temp_tangent.n.Y = -1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


    temp_tangent.pt.X = 0;
    temp_tangent.pt.Y = 120;

    temp_tangent.n.X = -1;
    temp_tangent.n.Y = 1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

    corridor.push_back(temp_polyh);

    //3rd corridor: 4 plane

    temp_polyh.PolyhedronTangentArray.clear();
    
    temp_tangent.pt.X = 180;
    temp_tangent.pt.Y = 250;

    temp_tangent.n.X = 1;
    temp_tangent.n.Y = 1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

    
    temp_tangent.pt.X = 180;
    temp_tangent.pt.Y = 50;

    temp_tangent.n.X = -1;
    temp_tangent.n.Y = -1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


    temp_tangent.pt.X = 280;
    temp_tangent.pt.Y = 150;

    temp_tangent.n.X = 1;
    temp_tangent.n.Y = -1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


    temp_tangent.pt.X = 80;
    temp_tangent.pt.Y = 150;

    temp_tangent.n.X = -1;
    temp_tangent.n.Y = 1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

    corridor.push_back(temp_polyh);

    //4th corridor: 4 plane

    temp_polyh.PolyhedronTangentArray.clear();
    
    temp_tangent.pt.X = 250;
    temp_tangent.pt.Y = 180;

    temp_tangent.n.X = 1;
    temp_tangent.n.Y = 1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

    
    temp_tangent.pt.X = 250;
    temp_tangent.pt.Y = -20;

    temp_tangent.n.X = -1;
    temp_tangent.n.Y = -1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


    temp_tangent.pt.X = 350;
    temp_tangent.pt.Y = 80;

    temp_tangent.n.X = 1;
    temp_tangent.n.Y = -1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


    temp_tangent.pt.X = 150;
    temp_tangent.pt.Y = 80;

    temp_tangent.n.X = -1;
    temp_tangent.n.Y = 1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

    corridor.push_back(temp_polyh);

    //5th corridor: 4 plane

    temp_polyh.PolyhedronTangentArray.clear();
    
    temp_tangent.pt.X = 280;
    temp_tangent.pt.Y = 100;

    temp_tangent.n.X = 1;
    temp_tangent.n.Y = 1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

    
    temp_tangent.pt.X = 280;
    temp_tangent.pt.Y = -100;

    temp_tangent.n.X = -1;
    temp_tangent.n.Y = -1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


    temp_tangent.pt.X = 380;
    temp_tangent.pt.Y = 0;

    temp_tangent.n.X = 1;
    temp_tangent.n.Y = -1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


    temp_tangent.pt.X = 180;
    temp_tangent.pt.Y = 0;

    temp_tangent.n.X = -1;
    temp_tangent.n.Y = 1;

    temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

    corridor.push_back(temp_polyh);

    cout<<corridor.size()<<endl;
    for(auto what : corridor)
    {
        cout<<what.PolyhedronTangentArray.size()<<endl;
    }

    ROS_INFO("corridor all set");

    //dynamic constraint
    alan_traj::dynamic_constraints d_constraints;
    d_constraints.v_max.x =  150;
    d_constraints.v_min.x = -150;//OsqpEigen::INFTY;//-150;
    d_constraints.a_max.x =  200;
    d_constraints.a_min.x = -200;//OsqpEigen::INFTY;//-200;
    d_constraints.j_max.x =  400;
    d_constraints.j_min.x = -400;//OsqpEigen::INFTY;//-400;

    d_constraints.v_max.y =  150;
    d_constraints.v_min.y = -150;//OsqpEigen::INFTY;//-150;
    d_constraints.a_max.y =  200;
    d_constraints.a_min.y = -200;//OsqpEigen::INFTY;//-200;
    d_constraints.j_max.y =  400;
    d_constraints.j_min.y = -400;//OsqpEigen::INFTY;//-400;


    alan_traj::bezier_constraints b_constraints;

    b_constraints.start = start_2d;
    b_constraints.end = end_2d;

    b_constraints.sfc_list = corridor;
    b_constraints.d_constraints = d_constraints;

    b_constraints.corridor_type = "POLYH";


    


   


    double t0 = ros::Time::now().toSec();

	alan_traj::traj_gen traj(b_traj_info, b_constraints);

	traj.solve_opt();

    double t1 = ros::Time::now().toSec();

    cout<<"fps: "<<1/(t1-t0)<<endl;

	return 0;
}