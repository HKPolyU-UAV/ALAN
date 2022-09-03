#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H

#include "essential.h"
#include "bernstein.hpp"

#include "qpsolver.h"
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>


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

    ifopt::Problem nlp;
    ifopt::IpoptSolver ipopt;    
    
    
public:

    traj_gen(bezier_info b_info, bezier_constraints b_constraints);
    ~traj_gen(){};

    void solveqp();
    vector<Eigen::VectorXd> getPolyCoeff(){return PolyCoeff;}

    

    
};

traj_gen::traj_gen(bezier_info b_info, bezier_constraints b_constraints)
    //for variable set
    : _n_dim((b_info.n_order + 1) + b_info.m),

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
    //n_dim = (b_info.n_order + 1) + b_info.m

    //2. constraints set
    _A = bezier_base.getA();
    _ub = bezier_base.getUB();
    _lb = bezier_base.getLB();
    
    //3. bezier_base.
    _MQM = bezier_base.getMQM();

    //now set nlp variable, constraints, and cost
    this->nlp.AddVariableSet  (std::make_shared<ifopt::ExVariables>(_n_dim));
    this->nlp.AddConstraintSet(std::make_shared<ifopt::ExConstraint>(_A, _ub, _lb));
    this->nlp.AddCostSet      (std::make_shared<ifopt::ExCost>(_MQM));

    //now set optimization methods
    this->nlp.PrintCurrent();
    this->ipopt.SetOption("linear_solver", "mumps");
    this->ipopt.SetOption("jacobian_approximation", "exact");

};

void traj_gen::solveqp()
{
    double t0 = ros::Time::now().toSec();

    ipopt.Solve(nlp);
    PolyCoeff.push_back(nlp.GetOptVariables()->GetValues()); //temp should be 3D

    double t1 = ros::Time::now().toSec();

    cout <<"Optimization Time: " << 1000 / (t1 - t0) <<" ms" <<endl;
}


#endif