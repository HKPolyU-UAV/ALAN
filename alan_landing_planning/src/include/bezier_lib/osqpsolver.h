#include <OsqpEigen/OsqpEigen.h>

#include "../tools/essential.h"




class osqpsolver
{
private:
    
    /* data */
public:
    osqpsolver(/* args */);
    void qp_opt(
        Eigen::MatrixXd _MQM, 
        Eigen::MatrixXd _A, 
        Eigen::MatrixXd _ub, 
        Eigen::MatrixXd _lb);
    ~osqpsolver();
};