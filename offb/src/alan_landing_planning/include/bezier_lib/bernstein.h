#include "../tools/essential.h"

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
        int n_order;
        int m;
        int d_order;
        vector<double> s; 
        bool kinematic = true;
        bool dynamic   = true;
    }bezier_info;

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
        double v_max;
        double v_min;

        double a_max;
        double a_min;

        double j_max;
        double j_min;

    }dynamic_constraints;


    typedef struct bezier_constraints 
    {
        //equality constraints
        endpt_cond start;
        endpt_cond end;
            
        //double
        vector<corridor> cube_list;
        
        dynamic_constraints d_constraints;
        
    }bezier_constraints;

    typedef struct  xyz
    {
        double x;
        double y;
        double z;

    } xyz;

    // typedef struct endpt
    // {
    //     endpt_cond x;
    //     endpt_cond y;
    //     endpt_cond z;

    // }endpt;

    typedef struct endpt
    {
        xyz posi;
        xyz velo;
        xyz accl;
        xyz jerk;
    }endpt;

    typedef



    class bernstein
    {
    private:
        Eigen::MatrixXd Q, Q_temp, M, M_temp, MQM, MQM_spd; 
        
        Eigen::MatrixXd A_eq, A_ieq, A; 

        Eigen::VectorXd ub_eq, ub_ieq, ub;
        Eigen::VectorXd lb_eq, lb_ieq, lb;

        Eigen::MatrixXd getMQM_spd(){return MQM_spd;}


        void setAeq(int n_order, int m, int d_order, vector<double> s);
        void setUBeq(endpt_cond start, endpt_cond end, int n_order, int m, int d_order);
        void setLBeq(endpt_cond start, endpt_cond end, int n_order, int m, int d_order);
        
        void setAieq(int n_order, int m, int d_order, vector<double> s);
        void setUBieq(vector<corridor> cube_list, dynamic_constraints d_constraints, int n_order, int m, int d_order);
        void setLBieq(vector<corridor> cube_list, dynamic_constraints d_constraints, int n_order, int m, int d_order);
        
        void setA();
        void setUB();
        void setLB();

        void setM(int n_order);
        void setQM(int n_order, int m, int d_order, vector<double> s);

        Eigen::MatrixXd getSPD(Eigen::MatrixXd Q);

        void setMQM(int n_order, int m, int d_order, vector<double> s);
        
        //tools
        double permutation(int p, int q);
        double factorial(int r);
        vector<double> pascal_triangle(int level);
        
        
    public:
        bernstein(
            int n_order, int m, int d_order, vector<double> s,
            endpt_cond start, endpt_cond end,
            vector<corridor> cube_list, dynamic_constraints d_constraints
            );

        ~bernstein(){};

        inline Eigen::MatrixXd getMQM(){return MQM;}
        inline Eigen::MatrixXd getA(){return A;}
        inline Eigen::MatrixXd getUB(){return ub;}
        inline Eigen::MatrixXd getLB(){return lb;}

    };

}