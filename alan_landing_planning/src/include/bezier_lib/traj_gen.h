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
        vector<corridor> _cube_list;
        dynamic_constraints _d_constraints;
        vector<alan_visualization::Polyhedron> _sfc_list;

        //Cost term
        Eigen::MatrixXd _MQM;
        int _n_order, _m, _d_order;
        vector<double> _s;  

        //math tool
        Eigen::MatrixXd get_nearest_SPD(Eigen::MatrixXd Q);
        double nchoosek(int n, int i);

        //set OptiTraj
        void setOptiTraj();
        void setTimeDiscrete();
        vector<vector<double>> time_vector;

        //other tool
        void msg_printer(char *s);
        void log();

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
            int discrete_freq
            );
        ~traj_gen(){};

        void solve_opt(int freq);

        alan_landing_planning::Traj getOptiTraj(){return optiTraj;}


        
    };
}

#endif