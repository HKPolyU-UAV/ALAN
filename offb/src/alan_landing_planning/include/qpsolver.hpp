#ifndef QPSOLVER_H
#define QPSOLVER_H

#include <qpOASES.hpp>
#include "essential.h"

class qpsolver
{
private:
    int _nV = 0;
    int _nC = 0;
    qpOASES::real_t *qpH;
    qpOASES::real_t *qpg;
    qpOASES::real_t *qpA;
    qpOASES::real_t *qplbA;
    qpOASES::real_t *qpubA;

    qpOASES::QProblem qpserver;
    qpOASES::Options qpoptions;

    Eigen::MatrixXd CholeskyDecomp(Eigen::MatrixXd Q);

public:
    qpsolver(int nV, int nC);
    
    void qpsetup(
        Eigen::MatrixXd _MQM, 
        Eigen::MatrixXd _A, 
        Eigen::MatrixXd _ub, 
        Eigen::MatrixXd _lb);
    
    void solve();

    void solve_trial();

    ~qpsolver();
};

qpsolver::qpsolver(int nV, int nC) : _nV(nV), _nC(nC)
{
    this->qpserver = qpOASES::QProblem(_nV, _nC);//, qpOASES::HST_SEMIDEF);
    
}

qpsolver::~qpsolver()
{
}

void qpsolver::qpsetup(
    Eigen::MatrixXd _MQM, 
    Eigen::MatrixXd _A, 
    Eigen::MatrixXd _ub, 
    Eigen::MatrixXd _lb)
{
    // _MQM.setIdentity();
    // cout<<_MQM<<endl;
    _MQM = CholeskyDecomp(_MQM);
    // cout<<"~~~~~~~~~~~~~~~~~"<<endl;
    // cout<<_MQM<<endl;
    qpH = _MQM.data();
    qpA = _A.data();

    Eigen::VectorXd _g;
    _g.resize(_nV);
    _g.setZero();
    qpg = _g.data();

    qpubA = _ub.data();
    qplbA = _lb.data();

    qpoptions.printLevel = qpOASES::PL_DEBUG_ITER;
    qpoptions.terminationTolerance = 1e-5;
    // qpoptions.setToReliable();


    qpOASES::Options options;
    options.setToReliable();

    qpserver.setOptions(options);


    


}

void qpsolver::solve()
{
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
        ROS_DEBUG_STREAM("Solved...");
    }

}

Eigen::MatrixXd qpsolver::CholeskyDecomp(Eigen::MatrixXd MQM) // return square root F of Q; Q = F' * F
{
    // make the Hessian to a 
    // symmetric positive semidefinite matrix
	Eigen::MatrixXd spd;
    Eigen::MatrixXd B = (MQM + MQM.transpose()) / 2;
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(B, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    Eigen::MatrixXd H = V * svd.singularValues().asDiagonal() * V.transpose();
    spd = (B + H) / 2;

    return spd;
    // spd = 
}

void qpsolver::solve_trial()
{
    qpoptions.printLevel = qpOASES::PL_DEBUG_ITER;
    // qpoptions.terminationTolerance = 1e-5;
    // qpoptions.setToReliable();

    qpOASES::int_t nWSR = 10^8;
    qpOASES::QProblemB qptest(_nV, qpOASES::HST_SEMIDEF);

    
    qpOASES::Options options;

    qptest.setOptions(options);

    qptest.init(qpH, qpg, NULL, NULL, nWSR);

    if(qpserver.isInfeasible())
    {
        ROS_ERROR("cannot solve...");
    }
    else
    {
        cout<<"hi"<<endl;
        ROS_DEBUG_STREAM("Solved...");
    }

    qpOASES::real_t xOpt[_nV];
    // for(int i = 0; i < _nV; i++)
    // {
    //     cout<<xOpt[i]<<endl;

    // }
    cout<<"lala"<<endl;
	qptest.getPrimalSolution( xOpt );
    
    // for(int i = 0; i < _nV; i++)
    // {
    //     cout<<xOpt[i]<<endl;

    // }
    cout<<"final value: "<<qptest.getObjVal()<<endl;

}




#endif