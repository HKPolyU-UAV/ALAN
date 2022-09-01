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
// #include "include/test.hpp"
#include "include/traj_gen.hpp"

// #include <nlopt.hpp>

using namespace cv;
using namespace std;
Mat src_gray;
int thresh = 100;
RNG rng(12345);


int main( int argc, char** argv )
{
    ros::init(argc, argv, "kf");
    ros::NodeHandle nh;

    double t1 = ros::Time::now().toSec();

    bezier_info b_info;
    bezier_constraints bezier_constraints;

    Eigen::MatrixXd l;
    Eigen::Matrix4d k;
    Eigen::Matrix3d o;
    k.setZero();
    o.setZero();

    l=k;
    l=o;


    // traj_gen b_traj(b_info, bezier_constraints);
    // b_traj.solveqp();
    // vector<Eigen::VectorXd> polycoeff_opt = b_traj.getPolyCoeff();

    
    
    // bezier_info b_info;

    // b_info.n_order = 7;
    // b_info.m = 5;
    // b_info.d_order = 4;
    // for(int i = 0; i < 5; i++)
    // {
    //     b_info.s.push_back(1.0);
    // }

    // bezier_constraints b_constraints;


    // ifopt::Problem nlp;
    // nlp.AddVariableSet  (std::make_shared<ifopt::ExVariables>(b_info));
    // nlp.AddConstraintSet(std::make_shared<ifopt::ExConstraint>(b_info, b_constraints));
    // nlp.AddCostSet      (std::make_shared<ifopt::ExCost>());

    // ifopt::Problem nlp;
    // nlp.AddVariableSet  (std::make_shared<ifopt::ExVariables>());
    // nlp.AddConstraintSet(std::make_shared<ifopt::ExConstraint>());
    // nlp.AddCostSet      (std::make_shared<ifopt::ExCost>());


    // nlp.PrintCurrent();
    cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~start"<<endl;
    double i = -0.6;
    double j = 5;
    // int k = j^i;
    // cout<<"kkkkkkkk: "<<pow(j,i)<<endl;

    // bernstein base;
    // vector<double> s;
    // for(int i = 0; i < 5; i++)
    //     s.push_back(1.0);
    
    // base.setMQM(7, 5, 4, s);
    
    // base.setQ(7, 5, 4, s);

    // Eigen::MatrixXd result = base.getQ();
    // cout<<result<<endl;
    

    // // 2. choose solver and options
    ifopt::IpoptSolver ipopt;
    // ipopt.
    ipopt.SetOption("linear_solver", "mumps");
    ipopt.SetOption("jacobian_approximation", "exact");

    // // 3 . solve
    // ipopt.Solve(nlp);
    // Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
    // std::cout << x.transpose() << std::endl;

    // // 4. test if solution correct
    // double eps = 1e-5; //double precision
    // // assert(1.0-eps < x(0) && x(0) < 1.0+eps);
    // // assert(0.0-eps < x(1) && x(1) < 0.0+eps);
    // cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~end"<<endl;

    // double t2 = ros::Time::now().toSec();

    // cout<<"fps: "<<1/(t2-t1)<<endl;

    


    return 0;
}
