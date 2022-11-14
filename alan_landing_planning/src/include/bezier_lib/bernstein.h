#ifndef BERNSTEIN_H
#define BERNSTEIN_H

#include "../tools/essential.h"
#include "alan_visualization/Polyhedron.h"
// #include "alan_landing_planning/"
#include "alan_landing_planning/AlanPlannerMsg.h"
#include "alan_landing_planning/test.h"


namespace alan_traj
{

    // alan_lan

    /*!
    * @struct      bezier_info
    * @abstract    bezier_info input.
    *
    * @field       axis_dim    dimension of the trajectory
    * @field       n_order     order of the polynomial
    * @field       m           no. of trajectories
    * @field       d_order     depends on the minimization objectives. 1 -- velo, 2 -- accl, 3 -- jerk, 4 -- snap
    * @field       s           time of each segment
    * @field       kinematic   whether there is kinematic constraints
    * @field       dymamic     whether there is dynamic constraints
    */

    typedef struct bezier_info 
    {
        int axis_dim;
        int n_order;
        int m;
        int d_order;
        vector<double> s; 
        bool kinematic = true;
        bool dynamic   = true;

    }bezier_info;

    

    //for all dimension endpt_condition
    typedef struct endpt
    {

        Eigen::Vector3d posi;
        Eigen::Vector3d velo;
        Eigen::Vector3d accl;
        Eigen::Vector3d jerk;

    }endpt;
    
    //for one dimension endpt_condition
    typedef struct endpt_cond 
    {

        double p_;
        double v_;
        double a_;
        double j_;

    }endpt_cond;

    //for defining a cube in up-to-three axis
    typedef struct corridor 
    {
        Eigen::Vector3d p_max;
        Eigen::Vector3d p_min;

    }corridor;

    //for irregular convex polyhedron, just use the alan_visualization::PolyhedronArray
    //which should be more than sufficient

    //for all dynamic constraints
    typedef struct dynamic_constraints 
    {
        Eigen::Vector3d v_max;
        Eigen::Vector3d v_min;

        Eigen::Vector3d a_max;
        Eigen::Vector3d a_min;

        Eigen::Vector3d j_max;
        Eigen::Vector3d j_min;

    }dynamic_constraints;

    
    //stack everything in to one datatype for passing argument convenience
    typedef struct bezier_constraints 
    {
        //equality constraints
        endpt start;
        endpt end;
            
        //double
        vector<corridor> cube_list;
        vector<alan_visualization::Polyhedron> sfc_list;

        string corridor_type;
        
        dynamic_constraints d_constraints;
        
    }bezier_constraints;

    

    // typedef



    class bernstein
    {
    private:

        //final results
        Eigen::MatrixXd MQM_final, A_final;
        Eigen::VectorXd ub_final, lb_final;

        Eigen::MatrixXd Q, Q_temp, M, M_temp, MQM, MQM_spd; 
        
        Eigen::MatrixXd A_eq, A_ieq, A; 

        Eigen::MatrixXd A_ieqsfc;
        Eigen::VectorXd ub_ieqsfc, lb_ieqsfc;

        Eigen::VectorXd ub_eq, ub_ieq, ub;
        Eigen::VectorXd lb_eq, lb_ieq, lb;

        vector<Eigen::MatrixXd> A_eq_array, A_ieq_array, A_array, MQM_array;
        vector<Eigen::VectorXd> ub_eq_array, ub_ieq_array, ub_array;
        vector<Eigen::VectorXd> lb_eq_array, lb_ieq_array, lb_array;

        vector<Eigen::MatrixXd> A_sfc_eq_array, A_sfc_ieq_dyn_array;

        Eigen::MatrixXd getMQM_spd(){return MQM_spd;}

        void setAeq1D(int axis_dim, int n_order, int m, int d_order, vector<double> s);
        void setUBeq1D(int axis_dim, endpt_cond start, endpt_cond end, int n_order, int m, int d_order);
        void setLBeq1D(int axis_dim, endpt_cond start, endpt_cond end, int n_order, int m, int d_order);
        
        void setAieq1D(int axis_dim, int n_order, int m, int d_order, vector<double> s, string corridor_type);
        
        void setUBieq1D(int axis_dim, vector<corridor> cube_list, dynamic_constraints d_constraints, int n_order, int m, int d_order);
        void setLBieq1D(int axis_dim, vector<corridor> cube_list, dynamic_constraints d_constraints, int n_order, int m, int d_order);
        
        void setUBieq1D_polyh(int axis_dim, dynamic_constraints d_constraints, int n_order, int m, int d_order);
        void setlBieq1D_polyh(int axis_dim, dynamic_constraints d_constraints, int n_order, int m, int d_order);

        void setA1D();
        void setUB1D();
        void setLB1D();
    

        void setMQM1D(int axis_dim, int n_order, int m, int d_order, vector<double> s);

        void setM1D(int axis_dim, int n_order);
        void setQM1D(int axis_dim, int n_order, int m, int d_order, vector<double> s);


        void setAieqBieqsfc(int axis_dim, vector<alan_visualization::Polyhedron> corridor, dynamic_constraints d_constraints, int n_order, int m, int d_order, vector<double> s);
        //coupled      

        void setFinalMatrices();

        void setMQMFinal();
        void setAFinal();
        void setUbFinal();
        void setLbFinal();

        void setAFinal_polyh();
        void setUbFinal_polyh();
        void setLbFinal_polyh();



        // Eigen::MatrixXd getSPD(Eigen::MatrixXd Q);

        
        //tools
        double permutation(int p, int q);
        double factorial(int r);
        vector<double> pascal_triangle(int level);

        
        
        
    public:
        bernstein(
            int axis_dim,
            int n_order, int m, int d_order, vector<double> s,
            endpt start, endpt end,
            vector<alan_visualization::Polyhedron> sfc_list,
             dynamic_constraints d_constraints
            );//for polyh

        bernstein(
            int axis_dim,
            int n_order, int m, int d_order, vector<double> s,
            endpt start, endpt end,
            vector<corridor> cube_list,
             dynamic_constraints d_constraints
            );//for cube

        ~bernstein(){};


        inline Eigen::MatrixXd getMQM(){return MQM_final;}
        inline Eigen::MatrixXd getA(){return A_final;}
        inline Eigen::MatrixXd getUB(){return ub_final;}
        inline Eigen::MatrixXd getLB(){return lb_final;}

    };

}

#endif