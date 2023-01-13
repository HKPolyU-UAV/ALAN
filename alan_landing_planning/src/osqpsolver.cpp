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

    // qpsolver.p

    for(int i = 0 ; i < 100; i++)
    {
        cout<<i<<endl;
        if(!qpsolver.updateHessianMatrix(Hessian))
        cout<<"Hessian not set!"<<endl;
        
        if(!qpsolver.updateLinearConstraintsMatrix(ALinear))
            cout<<"linear matrix not set!"<<endl;
        

        
        // if(!qpsolver.initSolver())
        //     cout<<"please initialize solver!!"<<endl;
        
        if(!qpsolver.solve())
            cout<<"not yet solved"<<endl;

    }
    
    

    qpsol = qpsolver.getSolution();

    // cout<<"cost: "<<qpsol.transpose() * H * qpsol<<endl;

}

void osqpsolver::qp_opt_samples()
{
    int success_i = 0;
    if(Hessian_array.size() != Alinear_array.size())
        return;
    else
    {
         for(int i = 0; i < Hessian_array.size(); i++)
        {
            // cout<<i<<endl;
            if(!qpsolver.updateHessianMatrix(Hessian_array[i]))
                cout<<"Hessian not update!"<<endl;
            
            if(!qpsolver.updateLinearConstraintsMatrix(Alinear_array[i]))
                cout<<"linear matrix not update!"<<endl;
            

            
            // if(!qpsolver.initSolver())
            //     cout<<"please initialize solver!!"<<endl;
            
            if(!qpsolver.solve())
                cout<<"not yet solved"<<endl;
            else
            {
                success_i++;
                // cout<<qpsolver.getSolution()<<endl<<endl;;
                // qpsolver.workspace()->;
                
                // cout<<"cost: "<< qpsolver.getSolution().transpose() * Hessian_array[i] * qpsolver.getSolution().transpose()<<endl;

            }
                
        }
    }
    
    cout<<success_i<<endl;
   
    
}