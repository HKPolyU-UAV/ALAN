#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H

#include "essential.h"
#include "bernstein.hpp"

#include "osqpsolver.hpp"


namespace alan_traj
{

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
        vector< corridor> _cube_list;
        dynamic_constraints _d_constraints;

        //Cost term
        Eigen::MatrixXd _MQM;
        int _n_order, _m, _d_order;
        vector<double> _s;  

        //math tool
        Eigen::MatrixXd get_nearest_SPD(Eigen::MatrixXd Q);

        //other tool
        void msg_printer(char *s);

        
    public:

        traj_gen( bezier_info b_info,  bezier_constraints b_constraints);
        ~traj_gen(){};

        void solve_opt();
        vector<Eigen::VectorXd> getPolyCoeff(){return PolyCoeff;}
        
    };

    traj_gen::traj_gen( bezier_info b_info,  bezier_constraints b_constraints)
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
        

        // msg_printer("-----------------------------------------------------------------");
        // msg_printer("             ALan v0.1 - Autonomous Landing for UAV");
        // msg_printer("                 (c) Li-yu LO,  Ching-wei Chang");
        // msg_printer("              The Hong Kong Polytechnic University");
        // msg_printer("-----------------------------------------------------------------");



        bernstein bezier_base(
            _n_order, _m, _d_order, _s,
            _start, _end,
            _cube_list, _d_constraints            
            );//pass everything in one pass
        
        //1. variable set
        // _n_dim = (b_info.n_order + 1) + b_info.m
        printf("1. variable set: ");
        cout<<_n_dim<<endl;

        //2. constraints set
        _A = bezier_base.getA();
        _ub = bezier_base.getUB();
        _lb = bezier_base.getLB();

        printf("2. constraints set: ");
        // cout<<"_A:"<<endl;
        cout<<_A.rows()<<endl;
        // cout<<"_lb:"<<endl;
        // cout<<_lb<<endl;
        // cout<<"_ub:"<<endl;
        // cout<<_ub<<endl;

        // cout<<"\nsummary:\n";
        // cout<<"A size:\n";
        // cout<<_A.rows()<<endl;
        // cout<<_A.cols()<<endl;
        // cout<<"upper bound size:\n";
        // cout<<_ub.size()<<endl;
        // cout<<"lower bound size:\n";
        // cout<<_lb.size()<<endl;


        
        //3. cost set
        printf("3. cost term: x'M'QMx\n");
        _MQM = bezier_base.getMQM();
        // _MQM = get_nearest_SPD(_MQM);

    };

    void traj_gen::solve_opt()
    {
        // qpsolver qpsolve(_n_dim, _A.rows());

        // qpsolve.solve_qp(_MQM, _A, _ub, _lb);

        osqpsolver temp;

        temp.qp_opt(_MQM, _A, _ub, _lb);
        // qpsolve.solve();
        // qpsolve.ifopt_test(_MQM, _A, _ub, _lb);
        // qpsolve.solve_trial();

        
    }


    Eigen::MatrixXd traj_gen::get_nearest_SPD(Eigen::MatrixXd MQM) 
    {
        // make the Hessian to a 
        // symmetric positive semidefinite matrix
        // depends on the solver, whether require this step or not
        
        // From Higham: "The nearest symmetric positive semidefinite matrix in the
        // Frobenius norm to an arbitrary real matrix A is shown to be (B + H)/2,
        // where H is the symmetric polar factor of B=(A + A')/2."


        Eigen::MatrixXd spd;
        Eigen::MatrixXd B = (MQM + MQM.transpose()) / 2;
        
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(B, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXd U = svd.matrixU();
        Eigen::MatrixXd V = svd.matrixV();

        Eigen::MatrixXd H = V * svd.singularValues().asDiagonal() * V.transpose();
        spd = (B + H) / 2;
        spd = (spd + spd.transpose()) / 2;


        bool psd_or_not = false;
        double k = 0;

        // cout<<"spd:"<<endl<<spd<<endl;

        while(!psd_or_not && k < 10)
        {
            cout<<k<<endl;
            Eigen::LLT<Eigen::MatrixXd> llt_check(spd);
            if(llt_check.info() == Eigen::NumericalIssue)//
            //try to do cholesky decomposition
            //as if A has A=LL^T
            //A is Hermitian & Positive (semi-)Definite
            {
                // cout<<"Possibly non semi-positive definitie matrix!"<<endl;;
            }   
            else
            {
                psd_or_not = true;
                // cout<<"we got it semi-positive definte!"<<endl;
                continue;
            }

            k = k + 1;

            Eigen::VectorXd spd_eigen_vector = spd.eigenvalues().real();


            double mineig = INFINITY;

            for(int i = 0; i < spd_eigen_vector.size(); i++)
            {
                // cout<<"here's one:"<<endl;
                // cout<<mineig<<endl;
                // cout<<spd_eigen_vector(i)<<endl<<endl;;;

                if(spd_eigen_vector(i) < mineig)
                    mineig = spd_eigen_vector(i);
            }

            Eigen::MatrixXd tweak;
            tweak.setIdentity(spd.rows(), spd.cols());

            double epsd = std::numeric_limits<double>::epsilon();

            double eps_mineig = std::nextafter(mineig, epsd) - mineig;

            tweak = tweak * (-mineig * pow(k,2) + eps_mineig);
            

            spd = spd + tweak;

            
        }

        return spd;
    }

    void traj_gen::msg_printer(char *s)
    {
        // printf("%*s%*s\n",10+strlen(s)/2,s,10-strlen(s)/2," ");
    }

}

#endif