

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
        string log_path)
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

    void traj_sampling::setTimeDiscrete(vector<double> _s_sample)
    {
        vector<double> time_vector_per_seg;
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
                // cout<<1 * j / discrete_time_step + 1 / discrete_time_step<<endl;
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
            cout<<t_per_seg<<endl;

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

        // cout<<"seg_i: "<<_m<<endl;
        // cout<<"_n_order: "<<_n_order<<endl;


        // cout<<time_vector.size()<<endl;


        for(int seg_i = 0; seg_i < _m; seg_i++)
        {
            // cout<<"seg time size: "<<time_vector[seg_i].size()<<endl;

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
        
        // cout<<"here are the traj size...: "<<optiTraj.trajectory.size()<<endl;
        optiTrajArray.trajectory_array.emplace_back(optiTraj);

    }

    

    

    void traj_sampling::set_prerequisite(
        vector<double> time_minmax, 
        int total_time_sample_no,
        int seg_time_sample_no
    )
    {
        vector<vector<double>> sampling_time;

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
        //     cout<<what[0]<<" "<<what[1]<<endl;
        // }

    }

    void traj_sampling::setSampling_time(
        vector<vector<double>>& sampling_time,
        vector<double> time_minmax, 
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

        // cout<<time_min<<endl;
        // cout<<time_max<<endl;

        for(int i = 0; i < total_time_sample_no ; i++)
        {
            double total_time_per_sample = 
                (1.0 / (total_time_sample_no - 1)) * i * time_min + 
                (1.0 / (total_time_sample_no - 1)) * ((total_time_sample_no - 1) - i) * time_max;
            // cout<<total_time_per_sample<<endl;

            for(int j = 0; j < seg_time_sample_no; j++)
            {
                // cout<<"hi"<<endl;
                // cout<<total_time_per_sample<<endl;
                double time_seg_0 = 
                    (1.0 / (seg_time_sample_no + 1)) * (j + 1) * total_time_per_sample;
                double time_seg_1 = total_time_per_sample - time_seg_0;

            
                vector<double> temp_sample = {time_seg_0, time_seg_1};
                cout<<time_seg_0<<" "<<time_seg_1<<endl<<endl;;
                sampling_time.emplace_back(temp_sample);
            }
        }
        trajSolver.set_time_sampling(sampling_time);
        cout<<"final sampling size..."<<sampling_time.size()<<endl;
    }

    void traj_sampling::setMatrices(bernstein& bezier_base, vector<vector<double>>& sampling_time)
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
       

        // cout<<A_samples[0]<<endl;

        // string temp = _log_path + "A_prerequisite.txt";
        // remove(temp.c_str());
        // ofstream save(temp ,ios::app);
        // save<<A_samples[0]<<endl;
        // save.close();

        // cout<<1 / (tock-tick)<<endl;
        // cout<<MQM_samples.size()<<endl;
        // cout<<A_samples.size()<<endl;
    }

    void traj_sampling::setBoundary(bernstein& bezier_base)
    {
        tuple<Eigen::VectorXd, Eigen::VectorXd> temp_tuple
            = bezier_base.set_ub_lb(_axis_dim, _n_order, _m, _d_order, _d_constraints);

        _ub = get<0>(temp_tuple);
        _lb = get<1>(temp_tuple);
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
            cout<<axis_i * starto<<endl;
            _ub(axis_i * starto) = posi_start(axis_i);
            _lb(axis_i * starto) = posi_start(axis_i);

            _ub(axis_i * starto + _axis_dim) = posi_end(axis_i);
            _lb(axis_i * starto + _axis_dim) = posi_end(axis_i);

        }

        // cout<<_ub<<endl;
        trajSolver.update_b_vectors(_ub, _lb);
    }

    void traj_sampling::optSamples()
    {
        optiTrajArray.trajectory_array.clear();

        vector<Eigen::VectorXd> qpsol_array;
        vector<vector<double>> sample_time_array;
        vector<double> optimal_time_allocation;
        int optimal_index;

        trajSolver.qp_opt_samples(
            qpsol_array,
            sample_time_array,
            optimal_time_allocation, 
            optimal_index
        );
        
        


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

        cout<<"Total Sample Size: "<<optiTrajArray.trajectory_array.size()<<endl;
        optiTraj = optiTrajArray.trajectory_array[optimal_index];
        ctrl_pts_optimal = qpsol_array[optimal_index];

        
        

        string temp = _log_path + "coeeff.txt";
        int no_of_ctrl = ctrl_pts_optimal.size() / 3;
        remove(temp.c_str());
        ofstream save(temp ,ios::app);
        

        for(int i = 0; i < no_of_ctrl; i++)
        {
            // ctrl_pts_optimal(i) = ctrl_pts_optimal(i) * 
            save<<ctrl_pts_optimal(i)<<" "
                <<ctrl_pts_optimal(i + no_of_ctrl)<<" "
                <<ctrl_pts_optimal(i + no_of_ctrl * 2)<<endl<<endl;

        }

        save.close();
        
        // cout<<endl<<qpsol_array.size()<<endl;
        // cout<<sample_time_array.size()<<endl;
        // cout<<optimal_time_allocation[0]<<" "<<optimal_time_allocation[1]<<endl;
        // cout<<optimal_index<<endl<<endl;;

    }

    void traj_sampling::log()
    {
        //b_traj info log...
        string temp = _log_path + "b_traj.txt";
        remove(temp.c_str()); 
        ofstream save(temp ,ios::app);

        save<<"log of trajectory optimization..."<<endl;
        save<<endl<<"log start..."<<endl<<endl;
        save<<"_axis_dim:\n"<<_axis_dim<<endl<<endl;
        save<<"_n_dim:\n"<<_n_dim<<endl<<endl;
        save<<"_n_dim_per_axis:\n"<<_n_dim_per_axis<<endl<<endl;
        save<<"_start_posi:\n"<<_start.posi<<endl<<endl;
        save<<"_start_velo:\n"<<_start.velo<<endl<<endl;
        save<<"_start_accl:\n"<<_start.accl<<endl<<endl;
        save<<"_sfc_list:\n";
        for(auto what : _sfc_list)
            save<<"here is one corridor...: "<<what.PolyhedronTangentArray.size()<<endl;
        save<<endl;
        save<<"_d_constraints v:\n"<<_d_constraints.v_max<<endl<<endl;
        save<<"_d_constraints a:\n"<<_d_constraints.a_max<<endl<<endl;
        save<<"_n_order\n"<<_n_order<<endl<<endl;
        save<<"_m\n"<<_m<<endl<<endl;
        save<<"_d_order\n"<<_d_order<<endl<<endl;

        for(auto what : _s)
            save<<"_s\n"<<what<<endl;
        save<<endl<<"log end..."<<endl;
        
        save.close();


        //matrix log...
        temp = _log_path + "MQM_matrix.txt";
        remove(temp.c_str()); 
        save = ofstream(temp ,ios::app);
        save<<_MQM<<endl;
        save.close();

        temp = _log_path + "A_matrix.txt";
        remove(temp.c_str()); 
        save = ofstream(temp ,ios::app);
        save<<_A<<endl;
        save.close();

        temp = _log_path + "ub.txt";
        remove(temp.c_str()); 
        save = ofstream(temp ,ios::app);
        save<<_ub<<endl;
        save.close();

        temp = _log_path + "lb.txt";
        remove(temp.c_str()); 
        save = ofstream(temp ,ios::app);
        save<<_lb<<endl;
        save.close();
    }

    double traj_sampling::nchoosek(int n, int k)
    {
        return pascal[n][k];
    }
}
