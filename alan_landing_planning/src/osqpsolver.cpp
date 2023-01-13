// #include <OsqpEigen/OsqpEigen.h>
// #include <osqp.h>
// #include "../tools/essential.h"

#include "include/bezier_lib/osqpsolver.h"

osqpsolver::osqpsolver()
{

}

osqpsolver::~osqpsolver()
{
    //gan
}

void osqpsolver::qp_opt(
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
    qpsolver.settings()->setVerbosity(false);
    // qpsolver.settings()->setMaxIteration(10);

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

    if(!qpsolver.updateHessianMatrix(Hessian))
        cout<<"Hessian not set!"<<endl;
    
    if(!qpsolver.updateLinearConstraintsMatrix(ALinear))
        cout<<"linear matrix not set!"<<endl;
    

    
    // if(!qpsolver.initSolver())
    //     cout<<"please initialize solver!!"<<endl;
    
    if(!qpsolver.solve())
        cout<<"not yet solved"<<endl;

    
    
    

    qpsol = qpsolver.getSolution();

    // cout<<"cost: "<<qpsol.transpose() * H * qpsol<<endl;

}

void osqpsolver::qp_opt_samples()
{
    int success_i = 0;

    double cost_min = INFINITY;
    double cost_temp = INFINITY;
    int cost_min_i = 0;

    if(_Hessian_array.size() != _Alinear_array.size())
        return;
    else
    {
         for(int i = 0; i < _Hessian_array.size(); i++)
        {
            // cout<<i<<endl;
            if(!qpsolver.updateHessianMatrix(_Hessian_array[i]))
                cout<<"Hessian not update!"<<endl;
            
            if(!qpsolver.updateLinearConstraintsMatrix(_Alinear_array[i]))
                cout<<"linear matrix not update!"<<endl;
            
            if(!qpsolver.solve())
                ROS_RED_STREAM("FAILED!");
            else
            {                   
                ROS_GREEN_STREAM("Succeed!");
                cout<<_time_samples[i][0]<<" "<<_time_samples[i][1]<<endl;
                cost_temp = qpsolver.getSolution().transpose() * _MQM_array[i] * qpsolver.getSolution();
                _cost_array.emplace_back(cost_temp);

                cout<<"cost: "<<_cost_array[_cost_array.size() - 1]<<endl;

                if(cost_temp < cost_min)
                {
                    cost_min = cost_temp;
                    cost_min_i = i;
                }
                    
                    

                success_i++;
                // cout<<qpsolver.getSolution()<<endl<<endl;;
                // qpsolver.workspace()->;
                
                // cout<<"cost: "<< qpsolver.getSolution().transpose() * Hessian_array[i] * qpsolver.getSolution().transpose()<<endl;

            }
                
        }

        if(success_i < 1)
            ROS_RED_STREAM("SAMPLING FAIL, PLEASE CHECK CONSTRAINTS & TIME ALLOCTION...");
        else
        {
            string msg_display0
                = "MINIMUM COST TRAJECTORY SEARCHED, " 
                    + to_string(cost_min_i) 
                    + " th SAMPLE, COST: " 
                    + to_string(cost_min)
                    + "TIME ALLOCATION: "
                    + to_string(_time_samples[cost_min_i][0])
                    + " "
                    + to_string(_time_samples[cost_min_i][1]);

            ROS_GREEN_STREAM(msg_display0);    
        }
    }
}

