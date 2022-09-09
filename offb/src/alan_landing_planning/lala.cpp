#include <iostream>
#include "include/camera.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
// #include "third_party/mosek/include/mosek.h"

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

// #include <IpIpoptApplication.hpp>
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
// #include <ifopt/test_vars_constr_cost.h>
// #include "include/qpsolver.h"
// #include "include/temp.hpp"
#include "include/traj_gen.hpp"

#include <nlopt.hpp>
#include <qpOASES.hpp>

using namespace std;


// int main( int argc, char** argv )
// {
//     

//     // nlopt::algorithm::

//     double t1 = ros::Time::now().toSec();

//     int n_order = 7;
//     int m = 5;
//     int d_order = 4;
//     vector<double> s;
//     for(int i = 0; i < 5; i++)
//         s.push_back(1);
    
//     endpt_cond start;
//     start.p_ = 50;
//     start.v_ = 0;
//     start.a_ = 0;
//     start.j_ = 0;

//     endpt_cond end;
//     end.p_ = 280;
//     end.v_ = 0;
//     end.a_ = 0;
//     end.j_ = 0;

//     vector<corridor> cube_list;
//     corridor cube;

//     cube.x_max = 100;
//     cube.x_min = 0;
//     cube_list.push_back(cube);

//     cube.x_max = 150;
//     cube.x_min = 50;
//     cube_list.push_back(cube);

//     cube.x_max = 230;
//     cube.x_min = 130;
//     cube_list.push_back(cube);
    
//     cube.x_max = 300;
//     cube.x_min = 200;
//     cube_list.push_back(cube);
    
//     cube.x_max = 330;
//     cube.x_min = 230;
//     cube_list.push_back(cube);

//     dynamic_constraints d_constraints;
//     d_constraints.v_max =  150;
//     d_constraints.v_min = -150;
//     d_constraints.a_max =  200;
//     d_constraints.a_min = -200;
//     d_constraints.j_max =  400;
//     d_constraints.j_min = -400;

//     bezier_info b_info;
//     bezier_constraints b_constraints;

//     b_info.n_order = n_order;
//     b_info.m = m;
//     b_info.d_order = d_order;
//     b_info.s = s;

//     b_constraints.start = start;
//     b_constraints.end = end;
//     b_constraints.cube_list = cube_list;
//     b_constraints.d_constraints = d_constraints;

//     traj_gen traj(b_info, b_constraints);

//     traj.solveqp();


//     // bernstein b_object_test(n_order, m, d_order, s, start, end, cube_list, d_constraints);

     
    
//     double t2 = ros::Time::now().toSec();

//     cout<<"fps: "<<1/(t2-t1)<<endl;

//     return 0;
// }


int main(int argc, char** argv)
{
	ros::init(argc, argv, "lala");
    ros::NodeHandle nh;
	// USING_NAMESPACE_QPOASES

	/* Setup data of first QP. */
	// qpOASES::real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };

	// cout<<H[0,0]<<endl;
	// cout<<H[0,1]<<endl;
	// cout<<H[1,0]<<endl;
	// cout<<H[1,1]<<endl;
	// cout<<*(H+3)<<endl;




	// qpOASES::real_t A[1*2] = { 1.0, 1.0 };
	// qpOASES::real_t g[2] = { 1.5, 1.0 };
	// qpOASES::real_t lb[2] = { 0.5, -2.0 };
	// qpOASES::real_t ub[2] = { 5.0, 2.0 };
	// qpOASES::real_t lbA[1] = { -1.0 };
	// qpOASES::real_t ubA[1] = { 2.0 };

	// /* Setup data of second QP. */
	// qpOASES::real_t g_new[2] = { 1.0, 1.5 };
	// qpOASES::real_t lb_new[2] = { 0.0, -1.0 };
	// qpOASES::real_t ub_new[2] = { 5.0, -0.5 };
	// qpOASES::real_t lbA_new[1] = { -2.0 };
	// qpOASES::real_t ubA_new[1] = { 1.0 };


	// /* Setting up QProblem object. */
	// qpOASES::QProblem example( 2,1 );

	// qpOASES::Options options;
	// example.setOptions( options );

	// /* Solve first QP. */
	// int nWSR = 10;
	// example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

    

	// /* Get and print solution of first QP. */
	// qpOASES::real_t xOpt[2];
	// qpOASES::real_t yOpt[2+1];
	// example.getPrimalSolution( xOpt );
	// example.getDualSolution( yOpt );
	// printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
	// 		xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );
	
	// /* Solve second QP. */
	// nWSR = 10;
	// example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );

	// /* Get and print solution of second QP. */
	// example.getPrimalSolution( xOpt );
	// example.getDualSolution( yOpt );
	// printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
	// 		xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );

	// example.printOptions();


	/*example.printProperties();*/
	ROS_DEBUG_STREAM("Hello " << "World");


	int n_order = 7;
    int m = 5;
    int d_order = 4;
    vector<double> s;
    for(int i = 0; i < 5; i++)
        s.push_back(1);
    
    endpt_cond start;
    start.p_ = 50;
    start.v_ = 0;
    start.a_ = 0;
    start.j_ = 0;

    endpt_cond end;
    end.p_ = 280;
    end.v_ = 0;
    end.a_ = 0;
    end.j_ = 0;

    vector<corridor> cube_list;
    corridor cube;

    cube.x_max = 100;
    cube.x_min = 0;
    cube_list.push_back(cube);

    cube.x_max = 150;
    cube.x_min = 50;
    cube_list.push_back(cube);

    cube.x_max = 230;
    cube.x_min = 130;
    cube_list.push_back(cube);
    
    cube.x_max = 300;
    cube.x_min = 200;
    cube_list.push_back(cube);
    
    cube.x_max = 330;
    cube.x_min = 230;
    cube_list.push_back(cube);

    dynamic_constraints d_constraints;
    d_constraints.v_max =  150;
    d_constraints.v_min = -150;
    d_constraints.a_max =  200;
    d_constraints.a_min = -200;
    d_constraints.j_max =  400;
    d_constraints.j_min = -400;

    bezier_info b_info;
    bezier_constraints b_constraints;

    b_info.n_order = n_order;
    b_info.m = m;
    b_info.d_order = d_order;
    b_info.s = s;

    b_constraints.start = start;
    b_constraints.end = end;
    b_constraints.cube_list = cube_list;
    b_constraints.d_constraints = d_constraints;

	traj_gen traj(b_info, b_constraints);

	traj.solveqp();

	

	




	return 0;
}