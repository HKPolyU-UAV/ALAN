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

    ~qpsolver();
};

qpsolver::qpsolver(int nV, int nC) : _nV(nV), _nC(nC)
{
    this->qpserver = qpOASES::QProblem(_nV, _nC, qpOASES::HST_SEMIDEF);
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
    // _MQM = CholeskyDecomp(_MQM);
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

    qpoptions.printLevel = qpOASES::PL_HIGH;
    qpoptions.terminationTolerance = 1e-5;
    qpoptions.setToReliable();

    qpserver.setOptions(qpoptions);


    


}

void qpsolver::solve()
{
    qpOASES::int_t nWSR = 4000;

    qpserver.init(qpH, qpg, qpA, NULL, NULL, qplbA, qpubA, nWSR);

    cout<<qpserver.isInfeasible()<<endl;;

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

Eigen::MatrixXd qpsolver::CholeskyDecomp(Eigen::MatrixXd Q) // return square root F of Q; Q = F' * F
{
	Eigen::MatrixXd F, Ft;
	Eigen::LDLT< Eigen::MatrixXd > ldlt(Q);
    F = ldlt.matrixL();
    F = ldlt.transpositionsP().transpose() * F;
    F *= ldlt.vectorD().array().sqrt().matrix().asDiagonal();
	Ft = F.transpose();

	return Ft;
}


#endif