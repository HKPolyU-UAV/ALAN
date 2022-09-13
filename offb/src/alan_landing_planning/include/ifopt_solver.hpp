#ifndef IFOPT_SOLVER_H
#define IFOPT_SOLVER_H

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <iostream>

namespace ifopt 
{
    // using Eigen::Vector2d;
    using namespace std;

    class ExVariables : public VariableSet 
    {
        public:

            //pass in the variable info
            ExVariables(int n_dim) : ExVariables("crtl_pts", n_dim)
            {
                

            };

            ExVariables(const std::string& name, int n_dim)
            : VariableSet( n_dim, name)
            {
                // the initial values where the NLP starts iterating from
                // int n_dim = (b_info.n_order + 1) * b_info.m;
                c_crtlpts.resize(n_dim);
                c_crtlpts.setZero();
            }

            //set target variables here
            //transform from our representation to solver Eigen::Vector
            VectorXd GetValues() const override
            {            
                return c_crtlpts;
            };

            //get our own representation
            void SetVariables(const VectorXd& x) override
            {
                // cout<<"setvariables"<<endl;
                c_crtlpts = x;            
            };

            

            // U&L bound of variables here
            VecBound GetBounds() const override
            {
                VecBound bounds(GetRows());
                for(int i = 0; i < GetRows(); i++)
                {
                    bounds.at(i) = NoBound;
                }
                return bounds;
            }

        private:
            Eigen::VectorXd c_crtlpts;        
    };


    class ExConstraint : public ConstraintSet 
    {
        public:
            ExConstraint(Eigen::MatrixXd A, Eigen::VectorXd ub, Eigen::VectorXd lb) 
            : ExConstraint("constraints", A, ub, lb) {}

            // This constraint set just contains 1 constraint, however generally
            // each set can contain multiple related constraints.
            ExConstraint(
                const std::string& name, 
                Eigen::MatrixXd A, 
                Eigen::VectorXd ub, 
                Eigen::VectorXd lb
                ) 
            : ConstraintSet(1, name) 
            {
                //set all needed constraints here
                //A c b all here 
                _A = A;
                _ub = ub;
                _lb = lb;
            }

            VectorXd GetValues() const override
            {
                VectorXd g(GetRows());
                Eigen::VectorXd _c = GetVariables()->GetComponent("crtl_pts")->GetValues();
                g = _A * _c;                

                return g;
            };

            // The only constraint in this set is an equality constraint to 1.
            // Constant values should always be put into GetBounds(), not GetValues().
            // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
            VecBound GetBounds() const override
            {
                VecBound b(GetRows());

                for(int i = 0; i < GetRows(); i++)
                {
                    b.at(i) = Bounds(_lb(i),_ub(i));
                }
                return b;
            }

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
            //not yet complete...

            // This function provides the first derivative of the constraints.
            // In case this is too difficult to write, you can also tell the solvers to
            // approximate the derivatives by finite differences and not overwrite this
            // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
            // Attention: see the parent class function for important information on sparsity pattern.
            void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
            {
                // must fill only that submatrix of the overall Jacobian that relates
                // to this constraint and "crtl_pts". even if more constraints or variables
                // classes are added, this submatrix will always start at row 0 and column 0,
                // thereby being independent from the overall problem.
                if (var_set == "crtl_pts") 
                {
                    Eigen::VectorXd x = GetVariables()->GetComponent("crtl_pts")->GetValues();

                    // jac_block.co
                    cout<<_A.rows()<<endl;
                    cout<<_A.cols()<<endl;

                    // jac_block.resize(_A.rows(), _A.cols());

                    Eigen::SparseMatrix<double, Eigen::RowMajor> temp;
                    temp = _A.sparseView();

                    // cout<<temp(0)<<endl;;
                    jac_block.resize(_A.rows(), _A.cols());

                    for(int i = 0; i < _A.rows(); i++)
                    {
                        for(int j = 0; j < _A.cols(); j++)
                        {
                            jac_block.coeffRef(i,j) = 0.0;

                        }
                    }

                    

                    // jac_block = temp;
                    
                    // jac_block = _A.sparseView();
                }
            }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


        private:  
            Eigen::MatrixXd _A;
            Eigen::VectorXd _ub, _lb;
            
    };


    class ExCost: public CostTerm 
    {
        public:
            ExCost(Eigen::MatrixXd MQM) : ExCost("jerkorsnap", MQM) {}
            ExCost(const std::string& name, Eigen::MatrixXd MQM) 
            : CostTerm(name) 
            {
                cout<<"cost here"<<endl;
                _MQM = MQM;
            }

            double GetCost() const override
            {
                Eigen::VectorXd _c = GetVariables()->GetComponent("crtl_pts")->GetValues();
                return _c.transpose() * _MQM * _c;
            };



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            //not yet complete...
            void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
            {
                // cout<<"jac"<<endl;
                // cout<<"a: "<<var_set<<endl;
                if (var_set == "crtl_pts") 
                {

                    Eigen::VectorXd x = GetVariables()->GetComponent("crtl_pts")->GetValues();

                    Eigen::MatrixXd temp = (_MQM + _MQM.transpose()) * x;
                    cout<<"alalala"<<endl;
                    // jac = temp.sparseView();
                }
            }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        private:
            Eigen::MatrixXd _MQM;

    };

} // namespace opt

#endif