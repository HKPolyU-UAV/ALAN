#ifndef QPSOLVER_H
#define QPSOLVER_H
#include <OsqpEigen/OsqpEigen.h>

#include "../tools/essential.h"

class osqpsolver
{
private:
    //solver itself
    OsqpEigen::Solver _qpsolver;

    //solutions
    Eigen::VectorXd qpsol;
    vector<Eigen::VectorXd> _qpsol_array;

    //sampling
    vector<Eigen::MatrixXd> _MQM_array;
    vector<Eigen::SparseMatrix<double>> _Hessian_array;
    vector<Eigen::SparseMatrix<double>> _Alinear_array;
    Eigen::VectorXd _ub;
    Eigen::VectorXd _lb;
    vector<vector<double>> _time_samples;
    vector<double> _cost_array;

    inline void initiate_qpsolve(int nV, int nC)
    {
        Eigen::VectorXd g;
        g.resize(nV);
        // g << -2, -6;
        g.setZero();

        _qpsolver.settings()->setWarmStart(true);
        _qpsolver.settings()->setVerbosity(false);

        _qpsolver.data()->setNumberOfVariables(nV);
        _qpsolver.data()->setNumberOfConstraints(nC);

        if(!_qpsolver.data()->setHessianMatrix(_Hessian_array[0]))
        cout<<"Hessian not set!"<<endl;

        if(!_qpsolver.data()->setGradient(g))
            cout<<"gradient not set!"<<endl;
        
        if(!_qpsolver.data()->setLinearConstraintsMatrix(_Alinear_array[0]))
            cout<<"linear matrix not set!"<<endl;
        
        if(!_qpsolver.data()->setLowerBound(_lb))
            cout<<"lb not set!!"<<endl;
        
        if(!_qpsolver.data()->setUpperBound(_ub))
            cout<<"ub not set!"<<endl;
        
        if(!_qpsolver.initSolver())
            cout<<"please initialize solver!!"<<endl;

    }

    /* data */
public:
    osqpsolver(/* args */);    
    ~osqpsolver();

    // single traj
    void qp_opt(
        Eigen::MatrixXd _MQM, 
        Eigen::MatrixXd _A, 
        Eigen::MatrixXd _ub, 
        Eigen::MatrixXd _lb
    );

    bool qp_opt_single(
        Eigen::MatrixXd& _MQM, 
        Eigen::MatrixXd& _A, 
        Eigen::VectorXd& _ub, 
        Eigen::VectorXd& _lb,
        Eigen::VectorXd& final_sol
    );

    inline Eigen::VectorXd getQpsol(){return qpsol;}


    void update_b_vectors(Eigen::VectorXd& ub, Eigen::VectorXd& lb)
    {
        _ub = ub;
        _lb = lb;
    }

    
    // sampling traj
    bool qp_opt_samples(
        vector<Eigen::VectorXd>& qpsol_array,
        vector<vector<double>>& sample_time_array,
        vector<Eigen::MatrixXd>& MQM_opti_array,
        vector<Eigen::MatrixXd>& A_opti_array,
        vector<double>& optimal_time_allocation,
        int& optimal_index
    );

    inline void set_sampling_matrices(
        vector<Eigen::MatrixXd>& MQM_array,
        vector<Eigen::MatrixXd>& A_array,
        Eigen::VectorXd& ub,
        Eigen::VectorXd& lb
    )
    {
        _MQM_array.clear();
        _Hessian_array.clear();
        _Alinear_array.clear();

        for(auto& what : MQM_array)
        {
            _MQM_array.emplace_back(what);
            _Hessian_array.emplace_back(what.sparseView());
        }
            
        for(auto& what : A_array)
            _Alinear_array.emplace_back(what.sparseView());
        
        _ub = ub;
        _lb = lb;

        int nV = MQM_array[0].rows();
        int nC = A_array[0].rows();

        // cout<<"hi we now in osqpsolver class..."<<endl;
        // cout<<_Hessian_array.size()<<endl;

        initiate_qpsolve(nV, nC);
    };

    inline void set_time_sampling(vector<vector<double>> time_samples){_time_samples = time_samples;}


    
};

#endif