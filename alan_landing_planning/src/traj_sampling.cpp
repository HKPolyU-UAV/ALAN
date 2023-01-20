

// #include "../tools/essential.h"
// #include "bernstein.hpp"

// #include "osqpsolver.hpp"

#include "include/bezier_lib/traj_sampling.h"

namespace alan_traj
{
    traj_sampling::traj_sampling( 
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
        _sfc_list(b_constraints.sfc_list),

        _d_constraints(b_constraints.d_constraints),

        //for cost term
        _n_order(b_info.n_order), _m(b_info.m), _d_order(b_info.d_order),

        //freq
        _discrete_freq(discrete_freq),

        //log_path
        _log_path(log_path)
    {
        ROS_WARN("TRAJ_SAMPLING INSTANTIATED!");          
    }; 

    void traj_sampling::setTimeDiscrete(std::vector<double> _s_sample)
    {
        std::vector<double> time_vector_per_seg;
        double discrete_time_step;
        double seg_time;

        // for(auto what : time_vector)
        //     what.clear();

        time_vector.clear();
        _s.clear();
        _s = _s_sample;

        for(int i = 0; i < _m; i++)
        {
            seg_time = _s_sample[i];
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

    void traj_sampling::setCtrlPts(Eigen::VectorXd& qpsol)
    {
        
        int no_of_ctrl = qpsol.size() / _axis_dim; //16
        int no_of_ctrl_per_seg = no_of_ctrl / _m;//8
        
        for(int i = 0; i < _m; i++)
        {
            double t_per_seg = _s[i];
            // std::cout<<t_per_seg<<std::endl;

            for(int j = 0; j < no_of_ctrl_per_seg; j++)
            {
                qpsol(i * no_of_ctrl_per_seg + j) 
                    = qpsol(i * no_of_ctrl_per_seg + j) * t_per_seg;
                qpsol(i * no_of_ctrl_per_seg + j + no_of_ctrl) 
                    = qpsol(i * no_of_ctrl_per_seg + j + no_of_ctrl) * t_per_seg;
                qpsol(i * no_of_ctrl_per_seg + j + no_of_ctrl * 2) 
                    = qpsol(i * no_of_ctrl_per_seg + j + no_of_ctrl * 2) * t_per_seg;
            }
        }

    }

    void traj_sampling::setOptiTrajSample(Eigen::VectorXd PolyCoeff)
    {        
        alan_landing_planning::AlanPlannerMsg traj_discrete_pt;
        alan_landing_planning::Traj optiTraj;
        optiTraj.trajectory.clear();
        
        double x_pos = 0, y_pos = 0, z_pos = 0;
        double p_base;

        // remove("/home/patty/alan_ws/src/alan/alan_landing_planning/src/test/traj.txt");
        // remove("/home/patty/alan_ws/src/alan/alan_landing_planning/src/test/p_base.txt");

        // std::cout<<"seg_i: "<<_m<<std::endl;
        // std::cout<<"_n_order: "<<_n_order<<std::endl;


        // std::cout<<time_vector.size()<<std::endl;


        for(int seg_i = 0; seg_i < _m; seg_i++)
        {
            // std::cout<<"seg time size: "<<time_vector[seg_i].size()<<std::endl;

            for(int i = 0; i < time_vector[seg_i].size(); i++)
            {
                x_pos = 0;
                y_pos = 0;
                z_pos = 0;
                
                for(int k = 0; k < _n_order + 1; k++)
                {
                    p_base =  nchoosek(_n_order, k) //_s[seg_i] *
                        * pow(time_vector[seg_i][i], k) 
                        * pow(1 - time_vector[seg_i][i], _n_order - k);
                      
                    x_pos = x_pos + PolyCoeff(seg_i * (_n_order + 1) + k) * p_base;
                    y_pos = y_pos + PolyCoeff(seg_i * (_n_order + 1) + k + _n_dim_per_axis) * p_base;
                    z_pos = z_pos + PolyCoeff(seg_i * (_n_order + 1) + k + _n_dim_per_axis * 2) * p_base;            
                }

                traj_discrete_pt.position.x = x_pos;
                traj_discrete_pt.position.y = y_pos;
                traj_discrete_pt.position.z = z_pos;

                optiTraj.trajectory.emplace_back(traj_discrete_pt);                
            }
        }
        
        // std::cout<<"here are the traj size...: "<<optiTraj.trajectory.size()<<std::endl;
        _optiTrajArray.trajectory_array.emplace_back(optiTraj);

    }

    void traj_sampling::setOptiFinalTraj(Eigen::VectorXd PolyCoeff)
    {        
        std::cout<<"hi setOptiFinalTraj"<<std::endl;
        std::cout<<PolyCoeff.size()<<std::endl;
        alan_landing_planning::AlanPlannerMsg traj_discrete_pt;
        alan_landing_planning::Traj optiTraj;
        optiTraj.trajectory.clear();
        
        double x_pos = 0, y_pos = 0, z_pos = 0;
        double p_base;

        for(int seg_i = 0; seg_i < _m; seg_i++)
        {
            // std::cout<<"seg time size: "<<time_vector[seg_i].size()<<std::endl;

            for(int i = 0; i < time_vector[seg_i].size(); i++)
            {
                x_pos = 0;
                y_pos = 0;
                z_pos = 0;
                
                for(int k = 0; k < _n_order + 1; k++)
                {
                    p_base =  nchoosek(_n_order, k) //_s[seg_i] *
                        * pow(time_vector[seg_i][i], k) 
                        * pow(1 - time_vector[seg_i][i], _n_order - k);
                      
                    x_pos = x_pos + PolyCoeff(seg_i * (_n_order + 1) + k) * p_base;
                    y_pos = y_pos + PolyCoeff(seg_i * (_n_order + 1) + k + _n_dim_per_axis) * p_base;
                    z_pos = z_pos + PolyCoeff(seg_i * (_n_order + 1) + k + _n_dim_per_axis * 2) * p_base;            
                }

                traj_discrete_pt.position.x = x_pos;
                traj_discrete_pt.position.y = y_pos;
                traj_discrete_pt.position.z = z_pos;

                optiTraj.trajectory.emplace_back(traj_discrete_pt);                
            }
        }    
        _optiTraj = optiTraj;

    }

        
    void traj_sampling::set_prerequisite(
        std::vector<double> time_minmax, 
        int total_time_sample_no,
        int seg_time_sample_no
    )
    {
        std::vector<std::vector<double>> sampling_time;

        bernstein bezier_base;

        setSampling_time(
            sampling_time, 
            time_minmax, 
            total_time_sample_no, 
            seg_time_sample_no
        );

        setMatrices(bezier_base, sampling_time);
        setBoundary(bezier_base);
        trajSolver.set_sampling_matrices(MQM_samples, A_samples, _ub, _lb);
        

        // for(auto what : sampling_time)
        // {
        //     std::cout<<what[0]<<" "<<what[1]<<std::endl;
        // }

    }

    void traj_sampling::setSampling_time(
        std::vector<std::vector<double>>& sampling_time,
        std::vector<double> time_minmax, 
        int total_time_sample_no, 
        int seg_time_sample_no
    )
    {
        if(_m != 2)
        {
            ROS_ERROR("PLEASE CHECK SEGMENT NO., NOT CORRECT!");
            return;
        }

        sampling_time.clear();
            
        int total_sample_no = total_time_sample_no * seg_time_sample_no;

        // for(int i = 0)
        double delta_time = time_minmax[1] - time_minmax[0];

        double time_min = time_minmax[0];
        double time_max = time_minmax[1];

        // std::cout<<time_min<<std::endl;
        // std::cout<<time_max<<std::endl;

        for(int i = 0; i < total_time_sample_no ; i++)
        {
            double total_time_per_sample = 
                (1.0 / (total_time_sample_no - 1)) * i * time_min + 
                (1.0 / (total_time_sample_no - 1)) * ((total_time_sample_no - 1) - i) * time_max;
            // std::cout<<total_time_per_sample<<std::endl;

            for(int j = 0; j < seg_time_sample_no; j++)
            {
                // std::cout<<"hi"<<std::endl;
                // std::cout<<total_time_per_sample<<std::endl;
                double time_seg_0 = 
                    (1.0 / (seg_time_sample_no + 1)) * (j + 1) * total_time_per_sample;
                double time_seg_1 = total_time_per_sample - time_seg_0;

            
                std::vector<double> temp_sample = {time_seg_0, time_seg_1};
                // std::cout<<time_seg_0<<" "<<time_seg_1<<std::endl<<std::endl;;
                sampling_time.emplace_back(temp_sample);
            }
        }
        trajSolver.set_time_sampling(sampling_time);
        // std::cout<<"final sampling size..."<<sampling_time.size()<<std::endl;
    }

    void traj_sampling::setMatrices(bernstein& bezier_base, std::vector<std::vector<double>>& sampling_time)
    {
        for(auto& what : sampling_time)
        {
            MQM_samples.emplace_back(
                bezier_base.set_1_MQMSample(_axis_dim, _n_order, _m, _d_order, what)
            );   

            A_samples.emplace_back(
                bezier_base.set_1_ASample(
                    _axis_dim, 
                    _n_order, 
                    _m, 
                    _d_order,
                    _sfc_list,
                    what
                )
            );


        }
  
    }

    void traj_sampling::setBoundary(bernstein& bezier_base)
    {
        std::tuple<Eigen::VectorXd, Eigen::VectorXd> temp_tuple
            = bezier_base.set_ub_lb(_axis_dim, _n_order, _m, _d_order, _d_constraints);

        _ub = std::get<0>(temp_tuple);
        _lb = std::get<1>(temp_tuple);
    }

    void traj_sampling::updateBoundary(
        Eigen::Vector3d& posi_start, 
        Eigen::Vector3d& posi_end,
        Eigen::Vector3d& velo_constraints
    )
    {
        int starto = 0;
        //set starting state in {b}, in particular for the ALAN project 
        for(int axis_i = 0; axis_i < _axis_dim; axis_i++)
        {
            starto = _axis_dim * 2 + _d_order;
            // std::cout<<axis_i * starto<<std::endl;
            _ub(axis_i * starto) = posi_start(axis_i);
            _lb(axis_i * starto) = posi_start(axis_i);

            _ub(axis_i * starto + _axis_dim) = posi_end(axis_i);
            _lb(axis_i * starto + _axis_dim) = posi_end(axis_i);

        }

        // std::cout<<_ub<<std::endl;
        // std::cout<<_lb<<std::endl;
        trajSolver.update_b_vectors(_ub, _lb);
    }

    void traj_sampling::optSamples()
    {
        _optiTrajArray.trajectory_array.clear();

        std::vector<Eigen::VectorXd> qpsol_array;
        std::vector<std::vector<double>> sample_time_array;
        std::vector<Eigen::MatrixXd> MQM_opti_array;
        std::vector<Eigen::MatrixXd> A_opti_array;
        std::vector<double> optimal_time_allocation;
        int optimal_index;

        bool sample_success = trajSolver.qp_opt_samples(
            qpsol_array,
            sample_time_array,
            MQM_opti_array,
            A_opti_array,
            optimal_time_allocation, 
            optimal_index
        );

        if(sample_success)
        {
            if(sample_time_array.size() == qpsol_array.size() && !qpsol_array.empty())
            {
                for(int i = 0; i < qpsol_array.size(); i++)
                {
                    setTimeDiscrete(sample_time_array[i]);
                    setCtrlPts(qpsol_array[i]);
                    setOptiTrajSample(qpsol_array[i]);
                }
            }
            else
            {
                ROS_ERROR("OPTIMIZATION WENT WRONG...");
            }

            std::cout<<"Total Sample Size: "<<_optiTrajArray.trajectory_array.size()<<std::endl;
            _optiTraj = _optiTrajArray.trajectory_array[optimal_index];
            _ctrl_pts_optimal = qpsol_array[optimal_index];

            std::string temp = _log_path + "coeeff.txt";
            int no_of_ctrl = _ctrl_pts_optimal.size() / 3;
            remove(temp.c_str());
            std::ofstream save(temp ,std::ios::app);
            for(int i = 0; i < no_of_ctrl; i++)
            {
                save<<_ctrl_pts_optimal(i)<<" "
                    <<_ctrl_pts_optimal(i + no_of_ctrl)<<" "
                    <<_ctrl_pts_optimal(i + no_of_ctrl * 2)<<std::endl<<std::endl;
            }
            save.close();

            optimal_traj_info.optimal_index = optimal_index;
            optimal_traj_info.optimal_time_allocation = optimal_time_allocation;
            optimal_traj_info.optiTraj = _optiTraj;
            optimal_traj_info.optiTrajArray = _optiTrajArray;
            optimal_traj_info.ctrl_pts_optimal = setCtrlVis(_ctrl_pts_optimal);
            optimal_traj_info.MQM = MQM_opti_array[optimal_index];
            optimal_traj_info.A = A_opti_array[optimal_index];
            optimal_traj_info.got_heuristic_optimal = true;
        }
        else
            optimal_traj_info.got_heuristic_optimal = false;

    }

    alan_landing_planning::Traj traj_sampling::opt_traj_online(Eigen::Vector3d& posi_current, Eigen::Vector3d& posi_goal)
    {
        Eigen::VectorXd final_sol;
        Eigen::Vector3d velo_temp;
        velo_temp.setZero();
        updateBoundary(posi_current, posi_goal, velo_temp);

        bool traj_gen_success = trajSolver.qp_opt_single(
            optimal_traj_info.MQM,
            optimal_traj_info.A,
            _ub,
            _lb,
            final_sol
        );

        if(traj_gen_success)
        {
            setTimeDiscrete(optimal_traj_info.optimal_time_allocation);
            setCtrlPts(final_sol);
            setOptiFinalTraj(final_sol);
        }
        else
        {
            _optiTraj = optimal_traj_info.optiTraj;
        }

        
        std::cout<<"execute this..."<<_optiTraj.trajectory.size()<<std::endl;
        return _optiTraj;
    }

    alan_landing_planning::Traj traj_sampling::setCtrlVis(Eigen::VectorXd optiCtrl)
    {
        alan_landing_planning::Traj vis_ctrl;
        alan_landing_planning::AlanPlannerMsg posi_temp;

        int no_of_ctrl = optiCtrl.size() / 3;

        for(int i = 0; i < no_of_ctrl; i++)
        {
            posi_temp.position.x = optiCtrl(i);
            posi_temp.position.y = optiCtrl(i + no_of_ctrl);
            posi_temp.position.z = optiCtrl(i + no_of_ctrl * 2);

            vis_ctrl.trajectory.emplace_back(posi_temp);
        }

        return vis_ctrl;
    }

    void traj_sampling::log()
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

    double traj_sampling::nchoosek(int n, int k)
    {
        return pascal[n][k];
    }
}
