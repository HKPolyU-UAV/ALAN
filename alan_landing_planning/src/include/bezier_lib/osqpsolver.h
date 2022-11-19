#ifndef QPSOLVER_H
#define QPSOLVER_H
#include <OsqpEigen/OsqpEigen.h>

#include "../tools/essential.h"

class osqpsolver
{
private:
    Eigen::VectorXd qpsol;
    
    /* data */
public:
    osqpsolver(/* args */);    
    ~osqpsolver();

    void qp_opt(
        Eigen::MatrixXd _MQM, 
        Eigen::MatrixXd _A, 
        Eigen::MatrixXd _ub, 
        Eigen::MatrixXd _lb);

    Eigen::VectorXd getQpsol(){return qpsol;}
    
};

#endif