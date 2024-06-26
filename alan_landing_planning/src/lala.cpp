/*
    This file is part of ALan - the non-robocentric dynamic landing system for quadrotor

    ALan is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ALan is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ALan.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file osqpsolver.h
 * \date 01/10/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief executable for algorithm tests
 */

#include <iostream>
#include "include/bezier_lib/traj_gen.h"

#include "include/bezier_lib/traj_sampling.h"



#include "alan_visualization/PolyhedronArray.h"
#include "alan_visualization/Tangent.h"

#include <mavros_msgs/AttitudeTarget.h>

#include <visualization_msgs/Marker.h>


// #include <decomp_ros_utils/data_ros_utils.h>
// decomp_ros_msg
using namespace std;

static alan_visualization::PolyhedronArray land_traj_constraint;
static bool land_traj_constraint_initiated = false;
static vector<alan_visualization::Polyhedron> corridors;
static alan_landing_planning::TrajArray optiTrajArray;
static alan_landing_planning::Traj optiTraj;
static Eigen::VectorXd optiCtrl;

static visualization_msgs::Marker ctrl_points;



void set_btraj_inequality_kinematic()
{
    int temp_size_i = 0;

    // cout<<"we got this many corridor:..."<<land_traj_constraint.a_series_of_Corridor.size()<<endl;    
    corridors.clear();

    alan_visualization::Polyhedron temp_poly;


    for(int i = 0; i < land_traj_constraint.a_series_of_Corridor.size(); i++)
    {
        temp_size_i = land_traj_constraint.a_series_of_Corridor[i].PolyhedronTangentArray.size();
        
        temp_poly.PolyhedronTangentArray.clear();

        for(int j = 0; j < temp_size_i; j++)
        {
            temp_poly.PolyhedronTangentArray.emplace_back(
                land_traj_constraint.a_series_of_Corridor[i].PolyhedronTangentArray[j]
            );
        }

        corridors.emplace_back(temp_poly);
    }

    // btraj_constraints.sfc_list = corridors;
    // btraj_constraints.corridor_type = "POLYH";

}

void sfcMsgCallback(const alan_visualization::PolyhedronArray::ConstPtr& sfc)
{
    land_traj_constraint.a_series_of_Corridor.clear();
    land_traj_constraint = *sfc;
    
    land_traj_constraint_initiated = true;
    // cout<<"hiii"<<endl;
    
}

void setCtrlVis()
{
    geometry_msgs::Point posi_temp;
    ctrl_points.points.clear();

    int no_of_ctrl = optiCtrl.size() / 3;

    for(int i = 0; i < no_of_ctrl; i++)
    {
        posi_temp.x = optiCtrl(i);
        posi_temp.y = optiCtrl(i + no_of_ctrl);
        posi_temp.z = optiCtrl(i + no_of_ctrl * 2);
        ctrl_points.points.push_back(posi_temp);

    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lala");
    ros::NodeHandle nh;

    // ros::Publisher poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("/polyhedron_array", 1, true);
    // ros::Publisher polyh_pub = nh.advertise<alan_visualization::Polyhedron>("/alan_visualization/polyhedron", 1, true);
    ros::Publisher ctrl_vis_pub = nh.advertise <visualization_msgs::Marker>("/gt_points/ctrl", 1, true);
    ros::Publisher traj_pub = nh.advertise<alan_landing_planning::Traj>("/alan_visualization/traj", 1, true);
    ros::Publisher trajArray_pub = nh.advertise<alan_landing_planning::TrajArray>("/alan_visualization/trajArray", 1, true);


    ros::Subscriber sfc_sub = nh.subscribe<alan_visualization::PolyhedronArray>
            ("/alan_state_estimation/msgsync/polyhedron_array", 1, sfcMsgCallback);

    
    ros::Rate rosrate(50);


    ctrl_points.header.frame_id = "world";
    ctrl_points.header.stamp = ros::Time::now();
    ctrl_points.ns = "GT_points";

    ctrl_points.id = 0;
    ctrl_points.action = visualization_msgs::Marker::ADD;
    ctrl_points.pose.orientation.w = 1.0;
    ctrl_points.type = visualization_msgs::Marker::SPHERE_LIST;
    ctrl_points.scale.x = ctrl_points.scale.y = ctrl_points.scale.z = 0.04;
    std_msgs::ColorRGBA color_for_edge;
    ctrl_points.color.a=1;
    ctrl_points.color.g=0;
    ctrl_points.color.r=1;
    ctrl_points.color.b=0;

    int i = 0;

    while(ros::ok())
    {
        
        ros::spinOnce();
        rosrate.sleep();

        // cout<<i<<endl;

        i++;
        if(land_traj_constraint_initiated && i > 10)
        {
            break;
        }
    }

    // cout<<"hey we got these..."<<corridors.size()<<endl;
    // cout<<"we are out"<<endl;

    Eigen::Vector3d virtual_car_velo;
    virtual_car_velo.setZero();


    double t00 = ros::Time::now().toSec();

    alan_traj::bezier_info b_info;
    
    b_info.axis_dim = 3;
    b_info.n_order = 7;
    b_info.m = 2;
    b_info.d_order = 3;


//inequality constraints:
    
    //kinematic constraints
    set_btraj_inequality_kinematic();

    cout<<corridors.size()<<endl;
    ROS_INFO("corridor all set");

    //dynamic constraint
    alan_traj::dynamic_constraints d_constraints;
    double vel_temp = 4;
    double acc_temp = 8;
    d_constraints.v_max(0) =  vel_temp;
    d_constraints.v_min(0) = -vel_temp;//OsqpEigen::INFTY;//-150;
    d_constraints.a_max(0) =  acc_temp;
    d_constraints.a_min(0) = -acc_temp;//OsqpEigen::INFTY;//-200;

    d_constraints.v_max(1) =  vel_temp;
    d_constraints.v_min(1) = -vel_temp;//OsqpEigen::INFTY;//-150;
    d_constraints.a_max(1) =  acc_temp;
    d_constraints.a_min(1) = -acc_temp;//OsqpEigen::INFTY;//-200;

    d_constraints.v_max(2) =  vel_temp;
    d_constraints.v_min(2) = -vel_temp;//OsqpEigen::INFTY;//-150;
    d_constraints.a_max(2) =  acc_temp;
    d_constraints.a_min(2) = -acc_temp;//OsqpEigen::INFTY;//-200;


    alan_traj::bezier_constraints b_constraints;


    b_constraints.sfc_list = corridors;
    b_constraints.d_constraints = d_constraints;
    b_constraints.corridor_type = "POLYH";


//sampling..
    alan_traj::traj_sampling btraj_sampling(
        b_info, 
        b_constraints, 
        50, 
        "/home/patty/alan_ws/src/alan/alan_landing_planning/src/log/"
    );

    // for visualization
    // posi_start: -1.6 0.0 1.8
    // end_start :  0.0 0.0 0.1
    // time max...1.16726 2.75

    vector<double> time_sample;
    time_sample.emplace_back(1.16726); //0.894425 //1.16726   1.91667
    time_sample.emplace_back(2.75);//2.0 //2.75
    
    int sample_size_root = 32;
    double tick0 = ros::Time::now().toSec();
    btraj_sampling.set_prerequisite(time_sample, sample_size_root, sample_size_root);
    double tock0 = ros::Time::now().toSec();
    
    cout<<"set pre-requisite:"<<endl;
    cout<<(tock0 - tick0)<<endl;
    cout<<1/(tock0 - tick0)<<endl<<endl;;


    Eigen::Vector3d posi_start(
            -1.6,
            0.0,
            1.8
        );
    Eigen::Vector3d posi_end(
            0.0,
            0.0,
            0.1
        );

    Eigen::Vector3d velo_constraint(0,0,0);

    double tick1 = ros::Time::now().toSec();

    btraj_sampling.updateBoundary(posi_start, posi_end, velo_constraint);
    btraj_sampling.optSamples();

    optiTraj = btraj_sampling.getOptiTraj();
    optiTrajArray = btraj_sampling.getOptiTrajSamples();
    optiCtrl = btraj_sampling.getOptiCtrl();
    setCtrlVis();
    

    double tock1 = ros::Time::now().toSec();

    


    cout<<endl<<endl;
    cout<<"now online..."<<endl;
    // btraj_sampling.
    Eigen::Vector3d posi_current(-1.6,0.0,1.8);
    Eigen::Vector3d posi_goal(0,0,0.1);
    double tick2 = ros::Time::now().toSec();
    alan_landing_planning::Traj traj_execute_final_in_B = btraj_sampling.opt_traj_online(
        posi_current,
        posi_goal
    );
    double tock2 = ros::Time::now().toSec();

    cout<<"online update:"<<endl;
    cout<<"ms: "<<(tock2 - tick2) * 1000<<endl;
    cout<<"fps: "<<1 / (tock2 - tick2)<<endl;


    while(ros::ok())
    {
        traj_pub.publish(optiTraj);
        trajArray_pub.publish(optiTrajArray);
        ctrl_vis_pub.publish(ctrl_points);
        
        rosrate.sleep();
    }


	return 0;
}



// double t0_matrix = ros::Time::now().toSec();
//     // for(int i = 0; i < 100; i++)
//     alan_traj::traj_gen traj(b_info, b_constraints, 50, "/home/patty/alan_ws/src/alan/alan_landing_planning/src/log/");
//     double t1_matrix = ros::Time::now().toSec();


//     double t0_opt = ros::Time::now().toSec();
//     traj.solve_opt(50);
//     double t1_opt = ros::Time::now().toSec();
    

//     // alan_landing_planning::Traj optiTraj = traj.getOptiTraj();

//     double t01 = ros::Time::now().toSec();

//     cout<<"overall\n";
//     cout<<"ms: "<<(t01-t00)<<endl;
//     cout<<"fps: "<<1/(t01-t00)<<endl<<endl;

//     cout<<"set matrices\n";
//     cout<<"ms: "<<(t1_matrix-t0_matrix)<<endl;
//     cout<<"fps: "<<1/(t1_matrix-t0_matrix)<<endl<<endl;

//     cout<<"optimize\n";
//     cout<<"ms: "<<(t1_opt-t0_opt)<<endl;
//     cout<<"fps: "<<1/(t1_opt-t0_opt)<<endl<<endl;






















// //1st corridor: 4 plane
//     temp_polyh.PolyhedronTangentArray.clear();

//     temp_tangent.pt.X = 50;
//     temp_tangent.pt.Y = 150;

//     temp_tangent.n.X = 1;
//     temp_tangent.n.Y = 1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

    
//     temp_tangent.pt.X = 50;
//     temp_tangent.pt.Y = -50;

//     temp_tangent.n.X = -1;
//     temp_tangent.n.Y = -1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


//     temp_tangent.pt.X = 150;
//     temp_tangent.pt.Y = 50;

//     temp_tangent.n.X = 1;
//     temp_tangent.n.Y = -1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


//     temp_tangent.pt.X = -50;
//     temp_tangent.pt.Y = 50;

//     temp_tangent.n.X = -1;
//     temp_tangent.n.Y = 1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

//     corridor.push_back(temp_polyh);

//     //2nd corridor: 4 plane

//     temp_polyh.PolyhedronTangentArray.clear();
    
//     temp_tangent.pt.X = 100;
//     temp_tangent.pt.Y = 220;

//     temp_tangent.n.X = 1;
//     temp_tangent.n.Y = 1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

    
//     temp_tangent.pt.X = 100;
//     temp_tangent.pt.Y = 20;

//     temp_tangent.n.X = -1;
//     temp_tangent.n.Y = -1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


//     temp_tangent.pt.X = 200;
//     temp_tangent.pt.Y = 120;

//     temp_tangent.n.X = 1;
//     temp_tangent.n.Y = -1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


//     temp_tangent.pt.X = 0;
//     temp_tangent.pt.Y = 120;

//     temp_tangent.n.X = -1;
//     temp_tangent.n.Y = 1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

//     corridor.push_back(temp_polyh);

//     //3rd corridor: 4 plane

//     temp_polyh.PolyhedronTangentArray.clear();
    
//     temp_tangent.pt.X = 180;
//     temp_tangent.pt.Y = 250;

//     temp_tangent.n.X = 1;
//     temp_tangent.n.Y = 1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

    
//     temp_tangent.pt.X = 180;
//     temp_tangent.pt.Y = 50;

//     temp_tangent.n.X = -1;
//     temp_tangent.n.Y = -1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


//     temp_tangent.pt.X = 280;
//     temp_tangent.pt.Y = 150;

//     temp_tangent.n.X = 1;
//     temp_tangent.n.Y = -1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


//     temp_tangent.pt.X = 80;
//     temp_tangent.pt.Y = 150;

//     temp_tangent.n.X = -1;
//     temp_tangent.n.Y = 1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

//     corridor.push_back(temp_polyh);

//     //4th corridor: 4 plane

//     temp_polyh.PolyhedronTangentArray.clear();
    
//     temp_tangent.pt.X = 250;
//     temp_tangent.pt.Y = 180;

//     temp_tangent.n.X = 1;
//     temp_tangent.n.Y = 1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

    
//     temp_tangent.pt.X = 250;
//     temp_tangent.pt.Y = -20;

//     temp_tangent.n.X = -1;
//     temp_tangent.n.Y = -1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


//     temp_tangent.pt.X = 350;
//     temp_tangent.pt.Y = 80;

//     temp_tangent.n.X = 1;
//     temp_tangent.n.Y = -1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


//     temp_tangent.pt.X = 150;
//     temp_tangent.pt.Y = 80;

//     temp_tangent.n.X = -1;
//     temp_tangent.n.Y = 1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

//     corridor.push_back(temp_polyh);

//     //5th corridor: 4 plane

//     temp_polyh.PolyhedronTangentArray.clear();
    
//     temp_tangent.pt.X = 280;
//     temp_tangent.pt.Y = 100;

//     temp_tangent.n.X = 1;
//     temp_tangent.n.Y = 1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

    
//     temp_tangent.pt.X = 280;
//     temp_tangent.pt.Y = -100;

//     temp_tangent.n.X = -1;
//     temp_tangent.n.Y = -1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


//     temp_tangent.pt.X = 380;
//     temp_tangent.pt.Y = 0;

//     temp_tangent.n.X = 1;
//     temp_tangent.n.Y = -1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);


//     temp_tangent.pt.X = 180;
//     temp_tangent.pt.Y = 0;

//     temp_tangent.n.X = -1;
//     temp_tangent.n.Y = 1;

//     temp_polyh.PolyhedronTangentArray.push_back(temp_tangent);

//     corridor.push_back(temp_polyh);














//earliest example
// //////////////////////////////////////////////////////

    // alan_traj::endpt b_traj_start;
    // b_traj_start.posi(0) = 50;
    // b_traj_start.velo(0) = 0;
    // b_traj_start.accl(0) = 0;
    // b_traj_start.jerk(0) = 0;

    // b_traj_start.posi(1) = 50;
    // b_traj_start.velo(1) = 0;
    // b_traj_start.accl(1) = 0;
    // b_traj_start.jerk(1) = 0;
    

    // alan_traj::endpt b_traj_end;
    // b_traj_end.posi(0) = 280;
    // b_traj_end.velo(0) = 0;
    // b_traj_end.accl(0) = 0;
    // b_traj_end.jerk(0) = 0;

    // b_traj_end.posi(1) = 0;
    // b_traj_end.velo(1) = 0;
    // b_traj_end.accl(1) = 0;
    // b_traj_end.jerk(1) = 0;

    // vector<alan_traj::corridor> cube_list;
    // alan_traj::corridor cube;

    // cube.p_max(0) = 100;
    // cube.p_max(1) = 100;//0;

    // cube.p_min(0) = 0;
    // cube.p_min(1) = 0;//0;
    // //
    // cube_list.push_back(cube);

    // cube.p_max(0) = 150;
    // cube.p_max(1) = 170;//-OsqpEigen::INFTY;//50;

    // cube.p_min(0) = 50;
    // cube.p_min(1) = 70;

    // cube_list.push_back(cube);

    // cube.p_max(0) = 230;
    // cube.p_max(1) = 200;//-OsqpEigen::INFTY;//130;

    // cube.p_min(0) = 130;
    // cube.p_min(1) = 100;

    // cube_list.push_back(cube);
    
    // cube.p_max(0) = 300;
    // cube.p_max(1) = 130;//-OsqpEigen::INFTY;//200;

    // cube.p_min(0) = 200;
    // cube.p_min(1) = 30;

    // cube_list.push_back(cube);
    
    // cube.p_max(0) = 330;
    // cube.p_max(1) = 50;//-OsqpEigen::INFTY;//230;

    // cube.p_min(0) = 230;
    // cube.p_min(1) = -50;

    // cube_list.push_back(cube);

    // alan_traj::dynamic_constraints b_traj_d_constraints;
    // b_traj_d_constraints.v_max(0) =  150;
    // b_traj_d_constraints.v_min(0) = -150;//OsqpEigen::INFTY;//-150;
    // b_traj_d_constraints.a_max(0) =  200;
    // b_traj_d_constraints.a_min(0) = -200;//OsqpEigen::INFTY;//-200;
    // b_traj_d_constraints.j_max(0) =  400;
    // b_traj_d_constraints.j_min(0) = -400;//OsqpEigen::INFTY;//-400;

    // b_traj_d_constraints.v_max(1) =  150;
    // b_traj_d_constraints.v_min(1) = -150;//OsqpEigen::INFTY;//-150;
    // b_traj_d_constraints.a_max(1) =  200;
    // b_traj_d_constraints.a_min(1) = -200;//OsqpEigen::INFTY;//-200;
    // b_traj_d_constraints.j_max(1) =  400;
    // b_traj_d_constraints.j_min(1) = -400;//OsqpEigen::INFTY;//-400;

    // alan_traj::bezier_info b_traj_b_info;
    // alan_traj::bezier_constraints b_traj_b_constraints;

    // vector<double> s;

    // for(int i = 0; i < 5; i++)
    //     s.push_back(1);

    // b_traj_b_info.axis_dim = 2;
    // b_traj_b_info.n_order = 7;
    // b_traj_b_info.m = 5;
    // b_traj_b_info.d_order = 4;
    // b_traj_b_info.s = s;

    // b_traj_b_constraints.start = b_traj_start;
    // b_traj_b_constraints.end = b_traj_end;
    // b_traj_b_constraints.cube_list = cube_list;
    // b_traj_b_constraints.d_constraints = b_traj_d_constraints;
    // b_traj_b_constraints.corridor_type = "CUBE";

    // double t2 = ros::Time::now().toSec();

    // cout<<"here enter~~"<<endl;

	// alan_traj::traj_gen b_traj_traj(b_traj_b_info, b_traj_b_constraints);

	// b_traj_traj.solve_opt();

    // double t3 = ros::Time::now().toSec();


	

    // double t1 = ros::Time::now().toSec();

    // cout<<"fps: "<<1/(t3-t2)<<endl;