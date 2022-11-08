#ifndef QPSOLVER_H
#define QPSOLVER_H

#include <qpOASES.hpp>
#include "essential.h"


class qpsolver
{
private:
    int _nV = 0;
    int _nC = 0;
    

public:
    qpsolver(int nV, int nC);
    
    void solve_qp(
        Eigen::MatrixXd _MQM, 
        Eigen::MatrixXd _A, 
        Eigen::MatrixXd _ub, 
        Eigen::MatrixXd _lb);
    
    ~qpsolver();
};

qpsolver::qpsolver(int nV, int nC) : _nV(nV), _nC(nC)
{
    // this->qpserver = qpOASES::QProblem(_nV, _nC);//, qpOASES::HST_SEMIDEF);
    
}

qpsolver::~qpsolver()
{
}

void qpsolver::solve_qp(
    Eigen::MatrixXd _MQM, 
    Eigen::MatrixXd _A, 
    Eigen::MatrixXd _ub, 
    Eigen::MatrixXd _lb)
{
    qpOASES::real_t *qpH;
    qpOASES::real_t *qpg;
    qpOASES::real_t *qpA;
    qpOASES::real_t *qplbA;
    qpOASES::real_t *qpubA;

    qpOASES::QProblem qpserver = qpOASES::QProblem(_nV, _nC);
    
    Eigen::LLT<Eigen::MatrixXd> llt_check(_MQM);


    // if(llt_check.info() == Eigen::NumericalIssue)//
    // //try to do cholesky decomposition
    // //as if A has A=LL^T
    // //A is Hermitian & Positive (semi-)Definite
    // {
    //     cout<<"!!!!!!Possibly non semi-positive definitie matrix!"<<endl;;
    // }   
    // else
    // {
    //     // psd_or_not = true;
    //     cout<<"!!!!!!!we got it semi-positive definte!"<<endl;
    //     // continue;
    // }

    qpH = _MQM.data();
    qpA = _A.data();

    Eigen::VectorXd _g;
    _g.resize(_nV);
    _g.setZero();
    qpg = _g.data();

    qpubA = _ub.data();
    qplbA = _lb.data();
    //////////////////////
    qpOASES::Options options;

    options.setToReliable();
    options.terminationTolerance = 1e-10;

    qpserver.setOptions(options);

    qpOASES::int_t nWSR = 4000;

    qpOASES::returnValue res = qpserver.init(qpH, qpg, qpA, NULL, NULL, qplbA, qpubA, nWSR);
    
    // cout<<qpserver.isInfeasible()<<endl;;
    cout<<"solve result: "<< res <<endl;

    if(qpserver.isInfeasible())
    {
        ROS_ERROR("cannot solve...");
    }
    else
    {
        cout<<"hi"<<endl;
        ROS_INFO("solved...");
    }
    cout<<"end"<<endl;

    qpOASES::real_t* xOpt = new qpOASES::real_t[_nV];
    qpserver.getPrimalSolution(xOpt);
    cout<<_nV<<" results: "<<endl;
    for(int i = 0; i < _nV; i++)
    {
        // cout<<"hi"<<endl;
        cout<<xOpt[i]<<" ";//<<endl;;
    }
    cout<<endl;

}

#endif