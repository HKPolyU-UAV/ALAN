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
 * \file traj_sampling.h
 * \date 01/01/2023
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for trajectory generation sampling
 */

#ifndef TRAJ_SAMPLING_H
#define TRAJ_SAMPLING_H

#include "../tools/essential.h"
#include "bernstein.h"
#include "osqpsolver.h"

#include "alan_landing_planning/AlanPlannerMsg.h"
#include "alan_landing_planning/TrajArray.h"

namespace alan_traj
{
    typedef struct optimal_traj //
    {
        int optimal_index;
        std::vector<double> optimal_time_allocation;
        alan_landing_planning::Traj optiTraj;
        alan_landing_planning::TrajArray optiTrajArray;
        alan_landing_planning::Traj ctrl_pts_optimal;
        // Eigen::VectorXd ctrl_pts_optimal;
        Eigen::MatrixXd MQM;
        Eigen::MatrixXd A;      
        bool got_heuristic_optimal = false;  
    }optimal_traj;

    //test gan
    class traj_sampling
    {

    private:

        //final result
        osqpsolver trajSolver;
        alan_landing_planning::TrajArray _optiTrajArray;
        alan_landing_planning::Traj _optiTraj;
        Eigen::VectorXd _ctrl_pts_optimal;
        
        int _axis_dim;

        //variable set
        int _n_dim;

        int _n_dim_per_axis;

        int _discrete_freq;

        //constraints set
        Eigen::MatrixXd _A;
        Eigen::VectorXd _ub, _lb;

        
        endpt _start, _end;
        std::vector<corridor> _cube_list;
        dynamic_constraints _d_constraints;
        std::vector<alan_visualization::Polyhedron> _sfc_list;

        std::string _log_path;

        //Cost term
        Eigen::MatrixXd _MQM;
        int _n_order, _m, _d_order;
        std::vector<double> _s;  

        //math tool
        double nchoosek(int n, int i);

        //set OptiTrajSample
        void setOptiTrajSample(Eigen::VectorXd PolyCoeff);
        void setOptiFinalTraj(Eigen::VectorXd PolyCoeff);
        void setCtrlPts(Eigen::VectorXd& qpsol);
        void setTimeDiscrete(std::vector<double> _s_sample);
        std::vector<std::vector<double>> time_vector;

        //other tool
        void log();
        // void log_file(std::string log_txt_location, );

        const double pascal[100][100] 
            = {
                {1},
                {1,  1},
                {1,  2,  1},
                {1,  3,  3,   1},
                {1,  4,  6,   4,   1},
                {1,  5, 10,  10,   5,   1},
                {1,  6, 15,  20,  15,   6,   1},
                {1,  7, 21,  35,  35,  21,   7,   1},
                {1,  8, 28,  56,  70,  56,  28,   8,   1},
                {1,  9, 36,  84, 126, 126,  84,  36,   9,  1},
                {1, 10, 45, 120, 210, 252, 210, 120,  45, 10,  1},
                {1, 11, 55, 165, 330, 462, 462, 330, 165, 55, 11, 1}

            };

    //sampling-related
        // for matrix pre-definition
        std::vector<Eigen::MatrixXd> MQM_samples;
        std::vector<Eigen::MatrixXd> A_samples;

        std::vector<Eigen::VectorXd> ub_samples;
        std::vector<Eigen::VectorXd> lb_samples;

        void setSampling_time(
            std::vector<std::vector<double>>& sampling_time,
            std::vector<double> time_minmax,
            int total_time_sample_no,
            int seg_time_sample_no
        );

        void setMatrices(
            bernstein& bezier_base, 
            std::vector<std::vector<double>>& sampling_time
        );

        void setBoundary(bernstein& bezier_base);

        optimal_traj optimal_traj_info;

        alan_landing_planning::Traj setCtrlVis(Eigen::VectorXd optiCtrl);

    public:

        traj_sampling( 
            bezier_info b_info,  
            bezier_constraints b_constraints,
            int discrete_freq,
            std::string log_path
        );
        
        ~traj_sampling(){};

    
        // for m = 2 (landing trajectory)
        // void set_

        void set_prerequisite(
            std::vector<double> time_minmax, 
            int total_time_sample_no,
            int seg_time_sample_no
        );

        void updateBoundary(
            Eigen::Vector3d& posi_start, 
            Eigen::Vector3d& posi_end, 
            Eigen::Vector3d& velo_constraints
        );

        void optSamples();
        
        alan_landing_planning::TrajArray getOptiTrajSamples(){return _optiTrajArray;}
        alan_landing_planning::Traj getOptiTraj(){return _optiTraj;}      
        Eigen::VectorXd getOptiCtrl(){return _ctrl_pts_optimal;}  

        optimal_traj getOptimalTrajInfo(){return optimal_traj_info;}

        alan_landing_planning::Traj opt_traj_online(Eigen::Vector3d& posi_current, Eigen::Vector3d& posi_goal);
    };
}

#endif