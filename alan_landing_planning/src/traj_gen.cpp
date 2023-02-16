/*
    This file is part of ALan - the non-robocentric dynamic landing system for quadrotor

    ALan is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ALan is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ALan.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file traj_gen.cpp
 * \date 01/10/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for trajectory generation
 */

#include "include/bezier_lib/traj_gen.h"

namespace alan_traj
{
    traj_gen::traj_gen( 
        bezier_info b_info,  
        bezier_constraints b_constraints, 
        int discrete_freq,
        std::string log_path)
        //for variable set
        :
        _axis_dim(b_info.axis_dim), 
        _n_dim((b_info.n_order + 1) * b_info.m * b_info.axis_dim),

        _n_dim_per_axis((b_info.n_order + 1) * b_info.m),

        //for constraints set
        _start(b_constraints.start), 
        _end(b_constraints.end), 

        _cube_list(b_constraints.cube_list),
        _sfc_list(b_constraints.sfc_list),

        _d_constraints(b_constraints.d_constraints),

        //for cost term
        _n_order(b_info.n_order), _m(b_info.m), _d_order(b_info.d_order), _s(b_info.s),

        //freq
        _discrete_freq(discrete_freq),

        //log_path
        _log_path(log_path)
    {
        ROS_WARN("entering traj generator!");  

             

        // msg_printer("-----------------------------------------------------------------");
        // msg_printer("             ALan v0.1 - Autonomous Landing for UAV");
        // msg_printer("                 (c) Li-yu LO,  Ching-wei Chang");
        // msg_printer("              The Hong Kong Polytechnic University");
        // msg_printer("-----------------------------------------------------------------");

        if(b_constraints.corridor_type == "POLYH")
        {
            //all axis matrices set here

            printf("POLYH constraints...\n");

            bernstein bezier_base(
                _axis_dim,
                _n_order, _m, _d_order, _s, 
                _start, _end,
                _sfc_list, _d_constraints
            );
            printf("1. variable set: ");
            std::cout<<_n_dim<<std::endl;

            //2. constraints set
            //should get everything (all axis) here
            _A = bezier_base.getA(); 
            _ub = bezier_base.getUB();
            _lb = bezier_base.getLB();

            printf("2. constraints set: ");
            // std::cout<<"_A:"<<std::endl;
            std::cout<<_A.rows()<<std::endl;



            
            //3. cost set
            printf("3. cost term: x'M'QMx\n");
            _MQM = bezier_base.getMQM();
            // _MQM = get_nearest_SPD(_MQM);


            


        }
        else if(b_constraints.corridor_type == "CUBE")
        {
            //all axis matrices set here

            printf("CUBE constraints...");
            bernstein bezier_base(
            _axis_dim,
            _n_order, _m, _d_order, _s,
            _start, _end,
            _cube_list, _d_constraints             
            );//pass everything in one pass
            
            // if
            
            //1. variable set
            // _n_dim = (b_info.n_order + 1) + b_info.m
            // printf("1. variable set: ");
            // std::cout<<_n_dim<<std::endl;

            //2. constraints set
            //should be containing all axises matrices
            _A = bezier_base.getA();
            _ub = bezier_base.getUB();
            _lb = bezier_base.getLB();

            // printf("2. constraints set: ");
            // std::cout<<_A.rows()<<std::endl;

            // std::cout<<"_lb:"<<std::endl;
            // std::cout<<_lb.size()<<std::endl;







            // std::cout<<"_ub:"<<std::endl;
            // std::cout<<_ub<<std::endl;

            // std::cout<<"\nsummary:\n";
            // std::cout<<"A size:\n";
            // std::cout<<_A.rows()<<std::endl;
            // std::cout<<_A.cols()<<std::endl;
            // std::cout<<"upper bound size:\n";
            // std::cout<<_ub.size()<<std::endl;
            // std::cout<<"lower bound size:\n";
            // std::cout<<_lb.size()<<std::endl;


            
            //3. cost set
            printf("3. cost term: x'M'QMx\n");
            _MQM = bezier_base.getMQM();

            
            // std::cout<<_MQM<<std::endl;
            // _MQM = get_nearest_SPD(_MQM);

        }
        else
        {
            ROS_ERROR("Please check kinematic constraint type...");
        }

        log(); 

    }; 

    void traj_gen::solve_opt(int freq)
    {
        // qpsolver qpsolve(_n_dim, _A.rows());

        // qpsolve.solve_qp(_MQM, _A, _ub, _lb);

        osqpsolver trajSolver;

        // _MQM = _MQM * 100;
        // _A = _A * 100;
        // _ub = _ub * 100;
        // _lb = _lb * 100;

        // std::cout<<_A<<std::endl;
        // std::cout<<_ub<<std::endl;
        // std::cout<<"\n-----------------------------------------------lalalalalalalalala\n";
        // std::cout<<_MQM<<std::endl;
        // std::cout<<_MQM.rows()<<"  "<<_MQM.cols()<<std::endl<<std::endl;;
        


        trajSolver.qp_opt(_MQM, _A, _ub, _lb);
        PolyCoeff = trajSolver.getQpsol();

        std::string temp = _log_path + "polycoef.txt";
        remove(temp.c_str()); 
        std::ofstream save(temp ,std::ios::app);
        save<<PolyCoeff<<std::endl;
        save.close();

        std::cout<<"\n----------------results here-------------------------\n";
        std::cout<<PolyCoeff<<std::endl<<std::endl<<std::endl;

        std::cout<<"size of ctrl pts..."<<PolyCoeff.size()<<std::endl;

        std::cout<<"enter set Time Discrete..."<<std::endl;
        setTimeDiscrete();
        std::cout<<"enter set Opti Traj..."<<std::endl;
        setOptiTraj();




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

        // std::cout<<"spd:"<<std::endl<<spd<<std::endl;

        while(!psd_or_not && k < 10)
        {
            std::cout<<k<<std::endl;
            Eigen::LLT<Eigen::MatrixXd> llt_check(spd);
            if(llt_check.info() == Eigen::NumericalIssue)//
            //try to do cholesky decomposition
            //as if A has A=LL^T
            //A is Hermitian & Positive (semi-)Definite
            {
                // std::cout<<"Possibly non semi-positive definitie matrix!"<<std::endl;;
            }   
            else
            {
                psd_or_not = true;
                // std::cout<<"we got it semi-positive definte!"<<std::endl;
                continue;
            }

            k = k + 1;

            Eigen::VectorXd spd_eigen_vector = spd.eigenvalues().real();


            double mineig = INFINITY;

            for(int i = 0; i < spd_eigen_vector.size(); i++)
            {
                // std::cout<<"here's one:"<<std::endl;
                // std::cout<<mineig<<std::endl;
                // std::cout<<spd_eigen_vector(i)<<std::endl<<std::endl;;;

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

    void traj_gen::setOptiTraj()
    {
        alan_landing_planning::AlanPlannerMsg traj_discrete_pt;
        
        double x_pos = 0, y_pos = 0, z_pos = 0;
        double p_base;

        remove("/home/patty/alan_ws/src/alan/alan_landing_planning/src/test/traj.txt");
        // remove("/home/patty/alan_ws/src/alan/alan_landing_planning/src/test/p_base.txt");

        // std::cout<<"seg_i: "<<_m<<std::endl;
        // std::cout<<"_n_order: "<<_n_order<<std::endl;


        // std::cout<<time_vector.size()<<std::endl;


        for(int seg_i = 0; seg_i < _m; seg_i++)
        {

            std::cout<<"seg time size: "<<time_vector[seg_i].size()<<std::endl;

            for(int i = 0; i < time_vector[seg_i].size(); i++)
            {
                x_pos = 0;
                y_pos = 0;
                z_pos = 0;
                
                for(int k = 0; k < _n_order + 1; k++)
                {
                    // std::cout<<"here: "<<seg_i<<" "<<i<<" "<<k<<std::endl;
                    // std::cout<<"here: "<<_n_order<<"  "<<k<<std::endl;

                    // std::cout<<nchoosek(_n_order, k)<<std::endl;

                    

                    p_base = nchoosek(_n_order, k) 
                        * pow(time_vector[seg_i][i], k) 
                        * pow(1 - time_vector[seg_i][i], _n_order - k);
                    // std::ofstream save("/home/patty/alan_ws/src/alan/alan_landing_planning/src/test/p_base.txt",std::ios::app);
                    // save<<p_base<<std::endl;
                
                
                    // save.close();
                    
                    // std::cout<<"p_Base: "<<p_base<<std::endl;

                    // std::cout<< seg_i * (_n_order + 1) + k<<std::endl;
                    // std::cout<< seg_i * (_n_order + 1) + k + _n_dim_per_axis<<std::endl;

                    // std::cout<<PolyCoeff(seg_i * (_n_order + 1) + k)<<std::endl;
                    // std::cout<<PolyCoeff(seg_i * (_n_order + 1) + k + _n_dim_per_axis)<<std::endl;
                    
                    x_pos = x_pos + PolyCoeff(seg_i * (_n_order + 1) + k) * p_base;
                    y_pos = y_pos + PolyCoeff(seg_i * (_n_order + 1) + k + _n_dim_per_axis) * p_base;

                    

                    // x_pos(idx) = x_pos(idx) + poly_coef_x((k-1)*(n_order+1)+i+1) * basis_p;
                    // y_pos(idx) = y_pos(idx) + poly_coef_y((k-1)*(n_order+1)+i+1) * basis_p;

                }
                traj_discrete_pt.position.x = x_pos;
                traj_discrete_pt.position.y = y_pos;


                

                // std::ofstream save("/home/patty/alan_ws/src/alan/alan_landing_planning/src/test/traj.txt",std::ios::app);
                // save<<x_pos<<std::endl;
                // save<<y_pos<<std::endl;
                // save<<std::endl;
                // save.close();

                optiTraj.trajectory.emplace_back(traj_discrete_pt);                
            }
        }


        
        std::cout<<"here are the traj size...: "<<optiTraj.trajectory.size()<<std::endl;

    }

    void traj_gen::setTimeDiscrete()
    {
        std::vector<double> time_vector_per_seg;
        double discrete_time_step;
        double seg_time;

        for(auto what : time_vector)
            what.clear();

        for(int i = 0; i < _m; i++)
        {
            seg_time = _s[i];
            discrete_time_step = seg_time * _discrete_freq;

            time_vector_per_seg.clear();

            for(double j = 0; j < discrete_time_step; j++)
            {
                // std::cout<<1 * j / discrete_time_step + 1 / discrete_time_step<<std::endl;
                time_vector_per_seg.emplace_back(1 * j / discrete_time_step + 1 / discrete_time_step);
            }            

            time_vector.emplace_back(time_vector_per_seg);            
        }
        
    }

    

    double traj_gen::nchoosek(int n, int k)
    {
        return pascal[n][k];
    }

    void traj_gen::log()
    {
        //b_traj info log...
        std::string temp = _log_path + "b_traj.txt";
        remove(temp.c_str()); 
        std::ofstream save(temp ,std::ios::app);

        save<<"log of trajectory optimization..."<<std::endl;
        save<<std::endl<<"log start..."<<std::endl<<std::endl;
        save<<"_axis_dim:\n"<<_axis_dim<<std::endl<<std::endl;
        save<<"_n_dim:\n"<<_n_dim<<std::endl<<std::endl;
        save<<"_n_dim_per_axis:\n"<<_n_dim_per_axis<<std::endl<<std::endl;
        save<<"_start_posi:\n"<<_start.posi<<std::endl<<std::endl;
        save<<"_start_velo:\n"<<_start.velo<<std::endl<<std::endl;
        save<<"_start_accl:\n"<<_start.accl<<std::endl<<std::endl;
        save<<"_sfc_list:\n";
        for(auto what : _sfc_list)
            save<<"here is one corridor...: "<<what.PolyhedronTangentArray.size()<<std::endl;
        save<<std::endl;
        save<<"_d_constraints v:\n"<<_d_constraints.v_max<<std::endl<<std::endl;
        save<<"_d_constraints a:\n"<<_d_constraints.a_max<<std::endl<<std::endl;
        save<<"_n_order\n"<<_n_order<<std::endl<<std::endl;
        save<<"_m\n"<<_m<<std::endl<<std::endl;
        save<<"_d_order\n"<<_d_order<<std::endl<<std::endl;

        for(auto what : _s)
            save<<"_s\n"<<what<<std::endl;
        save<<std::endl<<"log end..."<<std::endl;
        
        save.close();


        //matrix log...
        temp = _log_path + "MQM_matrix.txt";
        remove(temp.c_str()); 
        save = std::ofstream(temp ,std::ios::app);
        save<<_MQM<<std::endl;
        save.close();

        temp = _log_path + "A_matrix.txt";
        remove(temp.c_str()); 
        save = std::ofstream(temp ,std::ios::app);
        save<<_A<<std::endl;
        save.close();

        temp = _log_path + "ub.txt";
        remove(temp.c_str()); 
        save = std::ofstream(temp ,std::ios::app);
        save<<_ub<<std::endl;
        save.close();

        temp = _log_path + "lb.txt";
        remove(temp.c_str()); 
        save = std::ofstream(temp ,std::ios::app);
        save<<_lb<<std::endl;
        save.close();
    }
}