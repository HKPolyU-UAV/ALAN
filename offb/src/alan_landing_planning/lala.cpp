#include <iostream>
#include "include/camera.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include "third_party/mosek/include/mosek.h"


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
    MSKrescodee r, trmcode;

    MSKenv_t env = NULL;
    MSKtask_t task = NULL;
    
    double xx = 0.0;
    MSK_makeenv(&env, NULL); // Create environment
    MSK_maketask(env, 0, 1, &task); // Create task
    MSK_appendvars(task, 1); // 1 variable x
    MSK_putcj(task, 0, 1.0); // c_0 = 1.0
    MSK_putvarbound(task, 0, MSK_BK_RA, 2.0, 3.0); // 2.0 <= x <= 3.0
    MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE); // Minimize
    MSK_optimizetrm(task, &trmcode); // Optimize
    MSK_getxx(task, MSK_SOL_ITR, &xx); // Get solution
    printf("Solution x = %f\n", xx); // Print solution
    MSK_deletetask(&task); // Clean up task
    MSK_deleteenv(&env); // Clean up environment

    double t2 = ros::Time::now().toSec();

    cout<<1/(t2-t1)<<endl;


    return 0;
}
