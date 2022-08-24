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
#include <ifopt/test_vars_constr_cost.h>


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
    

    double t2 = ros::Time::now().toSec();

    cout<<1/(t2-t1)<<endl;

    try {
    int age = 15;
        if (age >= 18) {
            cout << "Access granted - you are old enough.";
        } else {
            throw 505;
        }
    }
    catch (...) {
    cout << "Access denied - You must be at least 18 years old.\n";
    }
    cout<<"hihi"<<endl;


    return 0;
}
