#include "../tools/essential.h"
#include "alan_visualization/Polyhedron.h"
// #include "alan_landing_planning/"
#include "alan_landing_planning/AlanPlannerMsg.h"



namespace alan_traj
{

    /*!
    * @struct      bezier_info
    * @abstract    bezier_info input.

    * @field       n_order     order of the polynomial
    * @field       m           no. of trajectories
    * @field       d_order     depends on the minimization objectives. 1 -- velo, 2 -- accl, 3 -- jerk, 4 -- snap
    * @field       s           time of each segment
    * @field       kinematic   whether there is kinematic constraints
    * @field       dymamic     whether there is dynamic constraints
    */

    typedef struct bezier_info 
    {
        int dimension;
        int n_order;
        int m;
        int d_order;
        vector<double> s; 
        bool kinematic = true;
        bool dynamic   = true;

    }bezier_info;

    typedef struct  xyz
    {
        double x;
        double y;
        double z;

    } xyz;



    typedef struct info_3d
    {
        double x;
        double y;
        double z;

    }info_3d;

    /*!
    * @struct      corridor
    * @abstract    corridor input -> as a cube.

    * @field       x_max            
    * @field       x_min           
    * @field       y_max
    * @field       y_min
    * @field       z_max
    * @field       z_min
    */
    typedef struct corridor 
    {
        double x_max;
        double x_min;

        double y_max;
        double y_min;

        double z_max;
        double z_min;

    }corridor;

    typedef struct endpt_cond 
    {

        double p_;
        double v_;
        double a_;
        double j_;

    }endpt_cond;

    typedef struct dynamic_constraints 
    {
        xyz v_max;
        xyz v_min;

        xyz a_max;
        xyz a_min;

        xyz j_max;
        xyz j_min;

        // double v_max;
        // double v_min;

        // double a_max;
        // double a_min;

        // double j_max;
        // double j_min;

    }dynamic_constraints;

    typedef struct endpt
    {

        Eigen::Vector3d posi;
        Eigen::Vector3d velo;
        Eigen::Vector3d accl;
        Eigen::Vector3d jerk;

    }endpt;

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
        Eigen::MatrixXd Q, Q_temp, M, M_temp, MQM, MQM_spd; 
        
        Eigen::MatrixXd A_eq, A_ieq, A; 

        Eigen::VectorXd ub_eq, ub_ieq, ub;
        Eigen::VectorXd lb_eq, lb_ieq, lb;

        vector<Eigen::MatrixXd> A_eq_array, A_ieq_array, A_array;
        vector<Eigen::VectorXd> ub_eq_array, ub_ieq_array, ub_array;
        vector<Eigen::VectorXd> lb_eq_array, lb_ieq_array, lb_array;

        Eigen::MatrixXd getMQM_spd(){return MQM_spd;}


        void setAeq1D(int n_order, int m, int d_order, vector<double> s);
        void setUBeq1D(endpt_cond start, endpt_cond end, int n_order, int m, int d_order);
        void setLBeq1D(endpt_cond start, endpt_cond end, int n_order, int m, int d_order);
        
        void setAieq1D(int n_order, int m, int d_order, vector<double> s);
        void setUBieq1D(vector<corridor> cube_list, dynamic_constraints d_constraints, int n_order, int m, int d_order);
        void setLBieq1D(vector<corridor> cube_list, dynamic_constraints d_constraints, int n_order, int m, int d_order);
        
        void setA1D();
        void setUB1D();
        void setLB1D();

        void setMQM1D(int n_order, int m, int d_order, vector<double> s);

        void setM1D(int n_order);
        void setQM1D(int n_order, int m, int d_order, vector<double> s);


        void setAieqsfc(vector<alan_visualization::Polyhedron> corridor, dynamic_constraints d_constraints, int n_order, int m, int d_order);
        //coupled      

        void setFinalMatrices();

        Eigen::MatrixXd getSPD(Eigen::MatrixXd Q);

        
        //tools
        double permutation(int p, int q);
        double factorial(int r);
        vector<double> pascal_triangle(int level);

        
        
        
    public:
        bernstein(
            int axis_order,
            int n_order, int m, int d_order, vector<double> s,
            endpt start, endpt end,
            vector<alan_visualization::Polyhedron> sfc_list,
             dynamic_constraints d_constraints
            );//for polyh

        bernstein(
            int axis_order,
            int n_order, int m, int d_order, vector<double> s,
            endpt start, endpt end,
            vector<corridor> cube_list,
             dynamic_constraints d_constraints
            );//for cube

        ~bernstein(){};

        void set1D_();

        inline Eigen::MatrixXd getMQM(){return MQM;}
        inline Eigen::MatrixXd getA(){return A;}
        inline Eigen::MatrixXd getUB(){return ub;}
        inline Eigen::MatrixXd getLB(){return lb;}

    };

}