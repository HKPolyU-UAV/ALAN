#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H

#include "essential.h"
#include "bernstein.hpp"

#include "qpsolver.hpp"
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>


//test gan
class traj_gen
{
private:
    vector<Eigen::VectorXd> PolyCoeff;

    //variable set
    int _n_dim;

    //constraints set
    Eigen::MatrixXd _A;
    Eigen::VectorXd _ub, _lb;

    
    endpt_cond _start, _end;
    vector<corridor> _cube_list;
    dynamic_constraints _d_constraints;

    //Cost term
    Eigen::MatrixXd _MQM;
    int _n_order, _m, _d_order;
    vector<double> _s;  

    
public:

    traj_gen(bezier_info b_info, bezier_constraints b_constraints);
    ~traj_gen(){};

    void solveqp();
    vector<Eigen::VectorXd> getPolyCoeff(){return PolyCoeff;}
    
};

traj_gen::traj_gen(bezier_info b_info, bezier_constraints b_constraints)
    //for variable set
    : _n_dim((b_info.n_order + 1) * b_info.m),

    //for constraints set
    _start(b_constraints.start), _end(b_constraints.end), 
    _cube_list(b_constraints.cube_list),
    _d_constraints(b_constraints.d_constraints),

    //for cost term
    _n_order(b_info.n_order), _m(b_info.m), _d_order(b_info.d_order), _s(b_info.s)    
{
    ROS_WARN("entering traj generator!");
    bernstein bezier_base(
        _n_order, _m, _d_order, _s,
        _start, _end,
        _cube_list, _d_constraints            
        );//pass everything in one pass
    
    //1. variable set
    // _n_dim = (b_info.n_order + 1) + b_info.m
    printf("1. variable set:\n");
    // cout<<_n_dim<<endl;

    //2. constraints set
    _A = bezier_base.getA();
    _ub = bezier_base.getUB();
    _lb = bezier_base.getLB();

    printf("2. constraints set:\n");
    cout<<_A<<endl;
    // cout<<_lb<<endl;
    // cout<<_ub<<endl;
    // cout<<_A.rows()<<endl;
    // cout<<_A.cols()<<endl;
    // cout<<_ub.size()<<endl;
    // cout<<_lb.size()<<endl;
    
    //3. cost set
    printf("3. cost term\n");
    _MQM = bezier_base.getMQM();
    // cout<<_MQM.rows()<<endl;
    // cout<<_MQM.cols()<<endl;





    

};

void traj_gen::solveqp()
{
    qpsolver qpsolve(_n_dim, _A.rows());

    qpsolve.qpsetup(_MQM, _A, _ub, _lb);
    qpsolve.solve();
    // qpsolve.solve_trial();

    
}


#endif