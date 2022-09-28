#ifndef QPSOLVER_H
#define QPSOLVER_H

#include <OsqpEigen/OsqpEigen.h>
// #include <osqp.h>
#include "essential.h"

class osqpsolver
{
private:
    /* data */
public:
    osqpsolver(/* args */);
    void test(
        Eigen::MatrixXd _MQM, 
        Eigen::MatrixXd _A, 
        Eigen::MatrixXd _ub, 
        Eigen::MatrixXd _lb);
    ~osqpsolver();
};

osqpsolver::osqpsolver(/* args */)
{
}

osqpsolver::~osqpsolver()
{
}

void osqpsolver::test(
        Eigen::MatrixXd _MQM, 
        Eigen::MatrixXd _A, 
        Eigen::MatrixXd _ub, 
        Eigen::MatrixXd _lb)
{
    // Load problem data
    
    int nV = _MQM.rows();
    int nC = _A.rows();
    
    Eigen::MatrixXd H;
    H.resize(nV, nV);
    H << _MQM;

    Eigen::VectorXd g;
    g.resize(nV);
    // g << -2, -6;
    g.setZero();

    Eigen::MatrixXd A;
    A.resize(nC, nV);
    A << _A;

    Eigen::VectorXd lb, ub;
    lb.resize(nC);
    ub.resize(nC);
    lb << _lb;//-OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
    ub << _ub;//2, 2, 3;

    Eigen::SparseMatrix<double> Hessian = H.sparseView();
    Eigen::SparseMatrix<double> ALinear = A.sparseView();


    OsqpEigen::Solver qpsolver;
    
    qpsolver.settings()->setWarmStart(true);

    qpsolver.data()->setNumberOfVariables(nV);
    qpsolver.data()->setNumberOfConstraints(nC);
    
    if(!qpsolver.data()->setHessianMatrix(Hessian))
        cout<<"Hessian not set!"<<endl;

    if(!qpsolver.data()->setGradient(g))
        cout<<"gradient not set!"<<endl;
    
    if(!qpsolver.data()->setLinearConstraintsMatrix(ALinear))
        cout<<"linear matrix not set!"<<endl;
    
    if(!qpsolver.data()->setLowerBound(lb))
        cout<<"lb not set!!"<<endl;
    
    if(!qpsolver.data()->setUpperBound(ub))
        cout<<"ub not set!"<<endl;
    
    if(!qpsolver.initSolver())
        cout<<"please initialize solver!!"<<endl;
    
    if(!qpsolver.solve())
        cout<<"not yet solved"<<endl;

    Eigen::VectorXd qpsol = qpsolver.getSolution();

    cout<<"here are the solutions: "<<endl;
    cout<<qpsol<<endl;


    cout<<"wowowowow"<<endl;
    

    

}


#endif