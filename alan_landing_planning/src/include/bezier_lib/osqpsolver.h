#ifndef QPSOLVER_H
#define QPSOLVER_H
#include <OsqpEigen/OsqpEigen.h>

#include "../tools/essential.h"

class osqpsolver
{
private:
    Eigen::VectorXd qpsol;
    vector<Eigen::SparseMatrix<double>> Hessian_array;
    vector<Eigen::SparseMatrix<double>> Alinear_array;
    Eigen::VectorXd _ub;
    Eigen::VectorXd _lb;

    /* data */
public:
    osqpsolver(/* args */);    
    ~osqpsolver();

    void qp_opt(
        Eigen::MatrixXd _MQM, 
        Eigen::MatrixXd _A, 
        Eigen::MatrixXd _ub, 
        Eigen::MatrixXd _lb);

    void update_b_vectors();
    void qp_opt_samples(Eigen::VectorXd& current_state);

    inline void set_sampling_matrices(
        vector<Eigen::MatrixXd>& MQM_array,
        vector<Eigen::MatrixXd>& A_array,
        Eigen::VectorXd& ub,
        Eigen::VectorXd& lb
    )
    {
        for(auto& what : MQM_array)
            Hessian_array.emplace_back(what.sparseView());

        for(auto& what : A_array)
            Alinear_array.emplace_back(what.sparseView());
        
        _ub = ub;
        _lb = lb;

        cout<<"hi we now in osqpsolver class..."<<endl;
        cout<<Hessian_array.size()<<endl;
        
        // cout<<_ub<<endl;
        // cout<<_lb<<endl;

    };

    inline Eigen::VectorXd getQpsol(){return qpsol;}
    
};

#endif