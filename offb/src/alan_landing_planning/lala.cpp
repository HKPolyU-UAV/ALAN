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

// #include <nlopt.hpp>

using namespace std;


int main( int argc, char** argv )
{
    ros::init(argc, argv, "kf");
    ros::NodeHandle nh;

    double t1 = ros::Time::now().toSec();

    // Eigen::MatrixXd la;
    // la.resize(3, 5);
    // la << 1, 2, 3, 4, 5,
    //       6, 7, 8, 9, 10,
    //       11, 12, 13, 14, 15;

    // cout<<la<<endl;
    // cout<<la(0,4)<<endl;

    int n_order = 7;
    int m = 4;
    int d_order = 3;
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


    bernstein b_object_test(n_order, m, d_order, s, start, end, cube_list, d_constraints);

     
    // A.resize(40,40);
    // A.setI

    

    ifopt::Problem nlp;
    // nlp.AddVariableSet  (std::make_shared<ifopt::ExVariables>());
    // nlp.AddConstraintSet(std::make_shared<ifopt::ExConstraint>());
    // nlp.AddCostSet      (std::make_shared<ifopt::ExCost>());
    
    // ifopt::Problem nlp;
    // nlp.AddVariableSet  (std::make_shared<ifopt::ExVariables>());
    // nlp.AddConstraintSet(std::make_shared<ifopt::ExConstraint>());
    // nlp.AddCostSet      (std::make_shared<ifopt::ExCost>());


    // nlp.PrintCurrent();
    cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~start"<<endl;
    
    

    // // 2. choose solver and options
    ifopt::IpoptSolver ipopt;
    // ipopt.
    ipopt.SetOption("linear_solver", "mumps");
    ipopt.SetOption("jacobian_approximation", "exact");
    // ipopt.SetOption("warm_start_init_point", "no");
    // ipopt.SetOption("least_square_init_primal", "yes");
    // ipopt.SetOption("least_square_init_duals", "yes");

    // // 3 . solve
    ipopt.Solve(nlp);
    Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
    std::cout << x.transpose() << std::endl;

    // // 4. test if solution correct
    // double eps = 1e-5; //double precision
    // // assert(1.0-eps < x(0) && x(0) < 1.0+eps);
    // // assert(0.0-eps < x(1) && x(1) < 0.0+eps);
    // cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~end"<<endl;

    double t2 = ros::Time::now().toSec();

    cout<<"fps: "<<1/(t2-t1)<<endl;

    


    return 0;
}
