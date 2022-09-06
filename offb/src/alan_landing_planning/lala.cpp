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
#include "include/temp.hpp"
// #include "include/traj_gen.hpp"

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

    ifopt::Problem nlp;
    nlp.AddVariableSet  (std::make_shared<ifopt::ExVariables>());
    nlp.AddConstraintSet(std::make_shared<ifopt::ExConstraint>());
    nlp.AddCostSet      (std::make_shared<ifopt::ExCost>());
    
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
    ipopt.SetOption("warm_start_init_point", "no");
    ipopt.SetOption("least_square_init_primal", "yes");
    ipopt.SetOption("least_square_init_duals", "yes");

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
