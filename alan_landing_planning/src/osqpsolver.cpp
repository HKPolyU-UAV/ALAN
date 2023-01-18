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

bool osqpsolver::qp_opt_samples(
    vector<Eigen::VectorXd>& qpsol_array,
    vector<vector<double>>& sample_time_array,
    vector<double>& optimal_time_allocation,
    int& optimal_index
)
{
    double cost_min = INFINITY;
    double cost_temp = INFINITY;
    
    int success_i = 0;
    int index_for_all_array = 0;
    int index_for_extracted = 0;

    qpsol_array.clear();

    if(_Hessian_array.size() != _Alinear_array.size())
        return false;
    else
    {
         for(int i = 0; i < _Hessian_array.size(); i++)
        {
            // cout<<i<<endl;
            if(!_qpsolver.updateHessianMatrix(_Hessian_array[i]))
                cout<<"Hessian not update!"<<endl;
            
            if(!_qpsolver.updateLinearConstraintsMatrix(_Alinear_array[i]))
                cout<<"linear matrix not update!"<<endl;
            
            if(!_qpsolver.solve())
                ROS_RED_STREAM("FAILED!");
            else
            {                   
                ROS_GREEN_STREAM("SUCCEED!");
                qpsol_array.emplace_back(_qpsolver.getSolution());
                sample_time_array.emplace_back(_time_samples[i]);

                cout<<_time_samples[i][0]<<" "<<_time_samples[i][1]<<endl;

                cost_temp = 
                    _qpsolver.getSolution().transpose() 
                    * _MQM_array[i] 
                    * _qpsolver.getSolution();
                    
                _cost_array.emplace_back(cost_temp);

                cout<<"cost: "<<_cost_array[_cost_array.size() - 1]<<endl;

                if(cost_temp < cost_min)
                {
                    cost_min = cost_temp;
                    index_for_all_array = i;
                    index_for_extracted = _cost_array.size() - 1;
                }       
                success_i ++;                                                 
            }                            
        }

        if(success_i < 1)
        {
            cout<<endl;
            ROS_RED_STREAM("SAMPLING FAIL, PLEASE CHECK CONSTRAINTS & TIME ALLOCTION...");
            cout<<endl;
            return false;
        }        
        else
        {
            optimal_index = index_for_extracted;

            optimal_time_allocation = _time_samples[index_for_all_array];

            string msg_display0
                = to_string(_Hessian_array.size()) 
                    + " TRAJ EVALUATED, SUCCEEDED "
                    + to_string(qpsol_array.size())
                    + " TRAJ";

            string msg_display1
                = to_string(optimal_index) 
                    + " th SAMPLE POSSESSES LOWEST COST: " 
                    + to_string(cost_min);

            string msg_display2
                = "TIME ALLOCATION: "
                    + to_string(_time_samples[index_for_all_array][0])
                    + " "
                    + to_string(_time_samples[index_for_all_array][1]);

            cout<<endl;
            ROS_GREEN_STREAM(msg_display0);   
            ROS_GREEN_STREAM(msg_display1);   
            ROS_GREEN_STREAM(msg_display2);    
            cout<<endl;

            return true;
        }
    }
}

