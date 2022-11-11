
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

    // mavros_msgs::AttitudeTarget lala;
    // lala.orientation.
    

    ros::init(argc, argv, "lala");
    ros::NodeHandle nh;

    ros::Publisher poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("/polyhedron_array", 1, true);
    ros::Publisher polyh_pub = nh.advertise<alan_visualization::Polyhedron>("/alan_visualization/polyhedron", 1, true);

    // test.n.X

    decomp_ros_msgs::PolyhedronArray poly_msg;
    
    poly_msg.header.frame_id = "map";
    poly_pub.publish(poly_msg);

    alan_traj::bezier_info b_info;
    
    b_info.axis_dim = 2;
    b_info.n_order = 7;
    b_info.m = 5;
    b_info.d_order = 3;

    for(int i = 0; i < b_info.m; i++)
        b_info.s.push_back(1);

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

    // cout<<corridor.size()<<endl;
    // for(auto what : corridor)
    // {
    //     cout<<what.PolyhedronTangentArray.size()<<endl;
    // }

    ROS_INFO("corridor all set");

    //dynamic constraint
    alan_traj::dynamic_constraints d_constraints;
    d_constraints.v_max(0) =  150;
    d_constraints.v_min(0) = -150;//OsqpEigen::INFTY;//-150;
    d_constraints.a_max(0) =  200;
    d_constraints.a_min(0) = -200;//OsqpEigen::INFTY;//-200;
    d_constraints.j_max(0) =  400;
    d_constraints.j_min(0) = -400;//OsqpEigen::INFTY;//-400;

    d_constraints.v_max(1) =  150;
    d_constraints.v_min(1) = -150;//OsqpEigen::INFTY;//-150;
    d_constraints.a_max(1) =  200;
    d_constraints.a_min(1) = -200;//OsqpEigen::INFTY;//-200;
    d_constraints.j_max(1) =  400;
    d_constraints.j_min(1) = -400;//OsqpEigen::INFTY;//-400;


    alan_traj::bezier_constraints b_constraints;

    b_constraints.start = start_2d;
    b_constraints.end = end_2d;

    b_constraints.sfc_list = corridor;
    b_constraints.d_constraints = d_constraints;

    b_constraints.corridor_type = "POLYH";

    double t0 = ros::Time::now().toSec();

	alan_traj::traj_gen traj(b_info, b_constraints);

    traj.solve_opt();




























    //////////////////////////////////////////////////////

    alan_traj::endpt b_traj_start;
    b_traj_start.posi(0) = 50;
    b_traj_start.velo(0) = 0;
    b_traj_start.accl(0) = 0;
    b_traj_start.jerk(0) = 0;

    b_traj_start.posi(1) = 50;
    b_traj_start.velo(1) = 0;
    b_traj_start.accl(1) = 0;
    b_traj_start.jerk(1) = 0;
    

    alan_traj::endpt b_traj_end;
    b_traj_end.posi(0) = 280;
    b_traj_end.velo(0) = 0;
    b_traj_end.accl(0) = 0;
    b_traj_end.jerk(0) = 0;

    b_traj_end.posi(1) = 0;
    b_traj_end.velo(1) = 0;
    b_traj_end.accl(1) = 0;
    b_traj_end.jerk(1) = 0;

    vector<alan_traj::corridor> cube_list;
    alan_traj::corridor cube;

    cube.p_max(0) = 100;
    cube.p_max(1) = 100;//0;

    cube.p_min(0) = 0;
    cube.p_min(1) = 0;//0;
    //
    cube_list.push_back(cube);

    cube.p_max(0) = 150;
    cube.p_max(1) = 170;//-OsqpEigen::INFTY;//50;

    cube.p_min(0) = 50;
    cube.p_min(1) = 70;

    cube_list.push_back(cube);

    cube.p_max(0) = 230;
    cube.p_max(1) = 200;//-OsqpEigen::INFTY;//130;

    cube.p_min(0) = 130;
    cube.p_min(1) = 100;

    cube_list.push_back(cube);
    
    cube.p_max(0) = 300;
    cube.p_max(1) = 130;//-OsqpEigen::INFTY;//200;

    cube.p_min(0) = 200;
    cube.p_min(1) = 30;

    cube_list.push_back(cube);
    
    cube.p_max(0) = 330;
    cube.p_max(1) = 50;//-OsqpEigen::INFTY;//230;

    cube.p_min(0) = 230;
    cube.p_min(1) = -50;

    cube_list.push_back(cube);

    alan_traj::dynamic_constraints b_traj_d_constraints;
    b_traj_d_constraints.v_max(0) =  150;
    b_traj_d_constraints.v_min(0) = -150;//OsqpEigen::INFTY;//-150;
    b_traj_d_constraints.a_max(0) =  200;
    b_traj_d_constraints.a_min(0) = -200;//OsqpEigen::INFTY;//-200;
    b_traj_d_constraints.j_max(0) =  400;
    b_traj_d_constraints.j_min(0) = -400;//OsqpEigen::INFTY;//-400;

    b_traj_d_constraints.v_max(1) =  150;
    b_traj_d_constraints.v_min(1) = -150;//OsqpEigen::INFTY;//-150;
    b_traj_d_constraints.a_max(1) =  200;
    b_traj_d_constraints.a_min(1) = -200;//OsqpEigen::INFTY;//-200;
    b_traj_d_constraints.j_max(1) =  400;
    b_traj_d_constraints.j_min(1) = -400;//OsqpEigen::INFTY;//-400;

    alan_traj::bezier_info b_traj_b_info;
    alan_traj::bezier_constraints b_traj_b_constraints;

    vector<double> s;

    for(int i = 0; i < 5; i++)
        s.push_back(1);

    b_traj_b_info.axis_dim = 2;
    b_traj_b_info.n_order = 7;
    b_traj_b_info.m = 5;
    b_traj_b_info.d_order = 4;
    b_traj_b_info.s = s;

    b_traj_b_constraints.start = b_traj_start;
    b_traj_b_constraints.end = b_traj_end;
    b_traj_b_constraints.cube_list = cube_list;
    b_traj_b_constraints.d_constraints = b_traj_d_constraints;
    b_traj_b_constraints.corridor_type = "CUBE";

    double t2 = ros::Time::now().toSec();

    cout<<"here enter~~"<<endl;

	alan_traj::traj_gen b_traj_traj(b_traj_b_info, b_traj_b_constraints);

	b_traj_traj.solve_opt();

    double t3 = ros::Time::now().toSec();


	

    double t1 = ros::Time::now().toSec();

    cout<<"fps: "<<1/(t3-t2)<<endl;

	return 0;
}