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
 * \file traj_gen.h
 * \date 01/10/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for trajectory generation
 */

#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H

#include "../tools/essential.h"
#include "bernstein.h"
#include "osqpsolver.h"

#include "alan_landing_planning/AlanPlannerMsg.h"
#include "alan_landing_planning/Traj.h"

namespace alan_traj
{

    //test gan
    class traj_gen
    {

    private:

        //final result
        Eigen::VectorXd PolyCoeff;
        alan_landing_planning::Traj optiTraj;

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
        Eigen::MatrixXd get_nearest_SPD(Eigen::MatrixXd Q);
        double nchoosek(int n, int i);

        //set OptiTraj
        void setOptiTraj();
        void setTimeDiscrete();
        std::vector<std::vector<double>> time_vector;

        //other tool
        void msg_printer(char *s);
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

        
    public:

        traj_gen( 
            bezier_info b_info,  
            bezier_constraints b_constraints,
            int discrete_freq,
            std::string log_path
            );
        ~traj_gen(){};

        void solve_opt(int freq);

        alan_landing_planning::Traj getOptiTraj(){return optiTraj;}


        
    };
}

#endif