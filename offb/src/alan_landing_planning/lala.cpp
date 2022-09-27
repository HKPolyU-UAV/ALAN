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
using namespace ifopt;


int main(int argc, char** argv)
{

    


	ros::init(argc, argv, "lala");
    ros::NodeHandle nh;
	
	ROS_DEBUG_STREAM("Hello " << "World");


//     // 1. define the problem
//   Problem nlp;
//   nlp.AddVariableSet  (std::make_shared<ExVariables>());
//   nlp.AddConstraintSet(std::make_shared<ExConstraint>());
//   nlp.AddCostSet      (std::make_shared<ExCost>());
//   nlp.PrintCurrent();

//   // 2. choose solver and options
//   IpoptSolver ipopt;
//   ipopt.SetOption("linear_solver", "mumps");
//   ipopt.SetOption("jacobian_approximation", "exact");

//   // 3 . solve
//   ipopt.Solve(nlp);
//   Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
//   std::cout << x.transpose() << std::endl;

//   // 4. test if solution correct
//   double eps = 1e-5; //double precision
//   assert(1.0-eps < x(0) && x(0) < 1.0+eps);
//   assert(0.0-eps < x(1) && x(1) < 0.0+eps);




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

    double t0 = ros::Time::now().toSec();

	traj_gen traj(b_info, b_constraints);

	traj.solveqp();

    double t1 = ros::Time::now().toSec();

    cout<<"fps: "<<1/(t1-t0)<<endl;
























    // USING_NAMESPACE_QPOASES

	// /* Setup data of first QP. */
	// real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
	// real_t A[1*2] = { 1.0, 1.0 };
	// real_t g[2] = { 1.5, 1.0 };
	// real_t lb[2] = { 0.5, -2.0 };
	// real_t ub[2] = { 5.0, 2.0 };
	// real_t lbA[1] = { -1.0 };
	// real_t ubA[1] = { 2.0 };

	// /* Setup data of second QP. */
	// real_t g_new[2] = { 1.0, 1.5 };
	// real_t lb_new[2] = { 0.0, -1.0 };
	// real_t ub_new[2] = { 5.0, -0.5 };
	// real_t lbA_new[1] = { -2.0 };
	// real_t ubA_new[1] = { 1.0 };


	// /* Setting up QProblem object. */
	// QProblem example( 2,1 );

	// Options options;
	// example.setOptions( options );

    // cout<<"now start..."<<endl;

	// /* Solve first QP. */
	// int_t nWSR = 10;
	// example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

	// /* Get and print solution of first QP. */
	// real_t xOpt[2];
	// real_t yOpt[2+1];
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

	/*getGlobalMessageHandler()->listAllMessages();*/

	return 0;
	

	




	return 0;
}
