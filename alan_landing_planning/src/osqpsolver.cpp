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
        std::cout<<"Hessian not set!"<<std::endl;

    if(!qpsolver.data()->setGradient(g))
        std::cout<<"gradient not set!"<<std::endl;
    
    if(!qpsolver.data()->setLinearConstraintsMatrix(ALinear))
        std::cout<<"linear matrix not set!"<<std::endl;
    
    if(!qpsolver.data()->setLowerBound(lb))
        std::cout<<"lb not set!!"<<std::endl;
    
    if(!qpsolver.data()->setUpperBound(ub))
        std::cout<<"ub not set!"<<std::endl;
    
    if(!qpsolver.initSolver())
        std::cout<<"please initialize solver!!"<<std::endl;

    if(!qpsolver.updateHessianMatrix(Hessian))
        std::cout<<"Hessian not set!"<<std::endl;
    
    if(!qpsolver.updateLinearConstraintsMatrix(ALinear))
        std::cout<<"linear matrix not set!"<<std::endl;
    

    
    // if(!qpsolver.initSolver())
    //     std::cout<<"please initialize solver!!"<<std::endl;
    
    if(!qpsolver.solve())
        std::cout<<"not yet solved"<<std::endl;

    
    
    

    qpsol = qpsolver.getSolution();

    // std::cout<<"cost: "<<qpsol.transpose() * H * qpsol<<std::endl;

}

bool osqpsolver::qp_opt_single(
    Eigen::MatrixXd& _MQM,
    Eigen::MatrixXd& _A,
    Eigen::VectorXd& _ub,
    Eigen::VectorXd& _lb,
    Eigen::VectorXd& final_sol
)
{
    std::cout<<"here in qp_opt_single"<<std::endl;
    // std::cout<<_ub<<std::endl;
    // std::cout<<_lb<<std::endl;

    Eigen::SparseMatrix<double> Hessian = _MQM.sparseView();
    if(!_qpsolver.updateHessianMatrix(Hessian))
        std::cout<<"Hessian not updated!"<<std::endl;
    
    Eigen::SparseMatrix<double> ALinear = _A.sparseView();
    if(!_qpsolver.updateLinearConstraintsMatrix(ALinear));

    if(!_qpsolver.data()->setUpperBound(_ub))
        std::cout<<"ub not set!!"<<std::endl;
    
    if(!_qpsolver.data()->setLowerBound(_lb))
        std::cout<<"lb not set!"<<std::endl;


    if(!_qpsolver.solve())
    {
        ROS_RED_STREAM("FAILED!");
        return false;
    }
    else
    {
        final_sol = _qpsolver.getSolution();
        std::cout<<"end..."<<final_sol.size()<<std::endl; 
        ROS_GREEN_STREAM("SUCCEEDED!");
        return true;
    }                   
}

bool osqpsolver::qp_opt_samples(
    std::vector<Eigen::VectorXd>& qpsol_array,
        std::vector<std::vector<double>>& sample_time_array,
        std::vector<Eigen::MatrixXd>& MQM_opti_array,
        std::vector<Eigen::MatrixXd>& A_opti_array,
        std::vector<double>& optimal_time_allocation,
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
            // std::cout<<i<<std::endl;
            if(!_qpsolver.updateHessianMatrix(_Hessian_array[i]))
                std::cout<<"Hessian not update!"<<std::endl;
            
            if(!_qpsolver.updateLinearConstraintsMatrix(_Alinear_array[i]))
                std::cout<<"linear matrix not update!"<<std::endl;
            
            if(!_qpsolver.solve())
                ROS_RED_STREAM("FAILED!");
            else
            {                   
                ROS_GREEN_STREAM("SUCCEED!");
                qpsol_array.emplace_back(_qpsolver.getSolution());
                sample_time_array.emplace_back(_time_samples[i]);
                MQM_opti_array.emplace_back(_MQM_array[i]);
                A_opti_array.emplace_back(_Alinear_array[i]);

                std::cout<<_time_samples[i][0]<<" "<<_time_samples[i][1]<<std::endl;

                cost_temp = 
                    _qpsolver.getSolution().transpose() 
                    * _MQM_array[i] 
                    * _qpsolver.getSolution();
                    
                _cost_array.emplace_back(cost_temp);

                std::cout<<"cost: "<<_cost_array[_cost_array.size() - 1]<<std::endl;

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
            std::cout<<std::endl;
            ROS_RED_STREAM("SAMPLING FAIL, PLEASE CHECK CONSTRAINTS & TIME ALLOCTION...");
            std::cout<<std::endl;
            return false;
        }        
        else
        {
            optimal_index = index_for_extracted;

            optimal_time_allocation = _time_samples[index_for_all_array];

            std::string msg_display0
                = std::to_string(_Hessian_array.size()) 
                    + " TRAJ EVALUATED, SUCCEEDED "
                    + std::to_string(qpsol_array.size())
                    + " TRAJ";

            std::string msg_display1
                = std::to_string(optimal_index) 
                    + " th SAMPLE POSSESSES LOWEST COST: " 
                    + std::to_string(cost_min);

            std::string msg_display2
                = "TIME ALLOCATION: "
                    + std::to_string(_time_samples[index_for_all_array][0])
                    + " "
                    + std::to_string(_time_samples[index_for_all_array][1]);

            std::cout<<std::endl;
            ROS_GREEN_STREAM(msg_display0);   
            ROS_GREEN_STREAM(msg_display1);   
            ROS_GREEN_STREAM(msg_display2);    
            std::cout<<std::endl;

            return true;
        }
    }
}



