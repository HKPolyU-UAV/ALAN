#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <iostream>
#include "bernstein.hpp"
namespace ifopt 
{
    // using Eigen::Vector2d;
    using namespace std;

    class ExVariables : public VariableSet 
    {
        public:
            ExVariables(bezier_info b_info) : ExVariables("crtl_pts", b_info)
            {

            };

            ExVariables(const std::string& name, bezier_info b_info)
            : VariableSet( (b_info.n_order + 1) * b_info.m, name)
            {
                // the initial values where the NLP starts iterating from
                int n_dim = (b_info.n_order + 1) * b_info.m;
                // cout<<"dim: "<< n_dim <<endl;            
                c_crtlpts.resize(n_dim);
                c_crtlpts.setZero();
                // cout<<"initial value: "<<endl<<c_crtlpts<<endl;
            }

            //get our own representation
            void SetVariables(const VectorXd& x) override
            {
                cout<<"setvariables"<<endl;
                c_crtlpts = x;            
            };

            //transform from our representation to solver Eigen::Vector
            VectorXd GetValues() const override
            {            
                return c_crtlpts;
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
            ExConstraint(bezier_info b_info, bezier_constraints b_constraints) 
            : ExConstraint("constraints", b_info, b_constraints) {}

            // This constraint set just contains 1 constraint, however generally
            // each set can contain multiple related constraints.
            ExConstraint(
                const std::string& name, 
                bezier_info b_info, 
                bezier_constraints b_constraints) 
            : ConstraintSet(2, name) 
            {

            }

            VectorXd GetValues() const override
            {
                VectorXd g(GetRows());
                Eigen::VectorXd c = GetVariables()->GetComponent("crtl_pts")->GetValues();
                g = A * c;                

                return g;
            };

            // The only constraint in this set is an equality constraint to 1.
            // Constant values should always be put into GetBounds(), not GetValues().
            // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
            VecBound GetBounds() const override
            {
                VecBound b(GetRows());
                b.at(0) = Bounds(1.0, 1.0);
                return b;
            }

            // This function provides the first derivative of the constraints.
            // In case this is too difficult to write, you can also tell the solvers to
            // approximate the derivatives by finite differences and not overwrite this
            // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
            // Attention: see the parent class function for important information on sparsity pattern.
            void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
            {
                // must fill only that submatrix of the overall Jacobian that relates
                // to this constraint and "var_set1". even if more constraints or variables
                // classes are added, this submatrix will always start at row 0 and column 0,
                // thereby being independent from the overall problem.
                if (var_set == "var_set1") 
                {
                    Eigen::Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

                    jac_block.coeffRef(0, 0) = 2.0*x(0); // derivative of first constraint w.r.t x0
                    jac_block.coeffRef(0, 1) = 1.0;      // derivative of first constraint w.r.t x1
                }
            }
        private:  
            Eigen::MatrixXd A;
            //Ac=b function
            //function details here  
            void getA()
            {

            }
            

    };


    class ExCost: public CostTerm 
    {
    public:
        ExCost() : ExCost("cost_term1") {}
        ExCost(const std::string& name) : CostTerm(name) {}

            double GetCost() const override
            {
                Eigen::Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
                return -std::pow(x(1)-2,2);
            };

            void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
            {
                if (var_set == "var_set1") 
                {
                    Eigen::Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

                    jac.coeffRef(0, 0) = 0.0;             // derivative of cost w.r.t x0
                    jac.coeffRef(0, 1) = -2.0*(x(1)-2.0); // derivative of cost w.r.t x1
                    std::cout<<"show jacobian here...: "<<jac<<std::endl;
                }
            }
    };

} // namespace opt
