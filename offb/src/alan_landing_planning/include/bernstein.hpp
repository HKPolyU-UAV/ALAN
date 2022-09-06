#ifndef BERNSTEIN_H
#define BERNSTEIN_H

#include "essential.h"

/*!
 * @struct      bezier_info
 * @abstract    bezier_info input.

 * @field       n_order     order of the polynomial
 * @field       m           no. of trajectories
 * @field       d_order     depends on the minimization objectives. 1 -- velo, 2 -- accl, 3 -- jerk, 4 -- snap
*/
typedef struct bezier_info 
{
    int n_order;
    int m;
    int d_order;
    vector<double> s; 
}bezier_info;

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

/*!
 * @struct      bezier_constraints
 * @abstract    bezier_constraints input.

 * @field       
 * @field       
 * @field       
*/
typedef struct bezier_constraints 
{
    //equality constraints
    endpt_cond start;
    endpt_cond end;
        
    //double
    vector<corridor> cube_list;
    
    dynamic_constraints d_constraints;
     
}bezier_constraints;



class bernstein
{
private:
    Eigen::MatrixXd Q, Q_temp, M, M_temp, MQM, MQM_spd; 
    
    Eigen::MatrixXd A_eq, A_ieq, A; 

    Eigen::VectorXd ub_eq, ub_ieq, ub;
    Eigen::VectorXd lb_eq, lb_ieq, lb;

    Eigen::MatrixXd getMQM_spd(){return MQM_spd;}


    void setAeq(int n_order, int m, int d_order, vector<double> s);
    void setAieq(int n_order, int m, int d_order, vector<double> s);
    void setA()
    {
        //combine A_eq && A_ieq
        A.resize(A_eq.rows() + A_ieq.rows(), A_eq.cols());

        A << A_eq,
             A_ieq;

    }

    void setUBeq(endpt_cond start, endpt_cond end, int n_order, int m, int d_order);
    void setUBieq(vector<corridor> cube_list, dynamic_constraints d_constraints, int n_order, int m, int d_order);
    void setUB()
    {
        //combine ub_eq && ub_ieq
        ub.resize(ub_eq.size() + ub_ieq.size());
        ub << ub_eq,
              ub_ieq;

    }

    void setLBeq(endpt_cond start, endpt_cond end, int n_order, int m, int d_order);
    void setLBieq(vector<corridor> cube_list, dynamic_constraints d_constraints, int n_order, int m, int d_order);
    void setLB()
    {
        //combine lb_eq && lb_ieq
        lb.resize(lb_eq.size() + lb_ieq.size());
        lb << lb_eq,
              lb_ieq;

    };

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
        )
    {        
        setAeq(n_order, m, d_order, s);
        setAieq(n_order, m, d_order, s);
        setA();

        setUBeq(start, end, n_order, m, d_order);//remember continuotiy
        setUBieq(cube_list, d_constraints, n_order, m, d_order);
        setUB();
        
        setLBeq(start, end, n_order, m, d_order);//remember continuotiy
        setLBieq(cube_list, d_constraints, n_order, m, d_order);
        setLB();

        setMQM(n_order, m, d_order, s);

    };
    ~bernstein();

    Eigen::MatrixXd getMQM(){return MQM;}
    Eigen::MatrixXd getA(){return A;}
    Eigen::MatrixXd getUB(){return ub;}
    Eigen::MatrixXd getLB(){return lb;}

};


bernstein::~bernstein()
{

}

void bernstein::setAeq(int n_order, int m, int d_order, vector<double> s)
{    
    int n_cond = 0;
    int _dim = (n_order + 1) * m; //how many control points
    vector<double> pascal;

    //Aeq_start
    Eigen::MatrixXd Aeq_start;
    n_cond =  d_order;//p v a s -> 0 1 2 3
    Aeq_start.resize(n_cond, _dim);
    Aeq_start.setZero();

    for(int i = 0 ; i < n_cond; i++)
    {
        pascal.clear();
        pascal = pascal_triangle(i + 1);
        
        double p = permutation(n_order, n_order - i);

        for(int j = 0; j <  i + 1; j++)
        {
            Aeq_start(i, j) = p * pascal[j] * pow(s[0], 1-j);
        }
    }

    //Aeq_end
    Eigen::MatrixXd Aeq_end;
    n_cond = d_order;
    Aeq_end.resize(n_cond, _dim);
    Aeq_end.setZero();

    for(int i = 0 ; i < n_cond; i++)
    {
        pascal.clear();
        pascal = pascal_triangle(i + 1);

        double p = permutation(n_order, n_order - i);

        for(int j = 0; j <  i + 1; j++)
        {
            Aeq_end(i, _dim-1 - j) = p * pascal[j] * pow(s[s.size()-1], 1-j);
        }
    }

    //Aeq_continuity
    Eigen::MatrixXd Aeq_cont;
    n_cond = (m - 1) * d_order;
    Aeq_cont.resize(n_cond, _dim);
    Aeq_cont.setZero();

    int i_m = (m - 1) - 1; //how many continuity should there be? A: m - 1 points, indicator m-1 -1 
    int starto_hori = 0, starto_vert = 0;

    for(int i = 0; i < i_m; i++) //each intersection point condition
    {
        starto_hori = (n_order + 1) * (i + 1) - 1;
        for(int j = 0; j < d_order; j++) // p v a continuity
        {
            starto_vert = i * d_order + j;
            pascal = pascal_triangle(j + 1);
            double p = permutation(n_order, n_order - j);

            switch (j)
            {
            case 0://p 
                Aeq_cont(starto_hori + 0, starto_vert) = p * pascal[0] * pow(s[i+0], 1-j);

                Aeq_cont(starto_hori + 1, starto_vert) = p * pascal[0] * pow(s[i+1], 1-j) * (-1);
                break;
            
            case 1://v
                Aeq_cont(starto_hori - 1, starto_vert) = p * pascal[0] * pow(s[i+0], 1-j);
                Aeq_cont(starto_hori - 0, starto_vert) = p * pascal[1] * pow(s[i+0], 1-j);

                Aeq_cont(starto_hori + 1, starto_vert) = p * pascal[1] * pow(s[i+1], 1-j);
                Aeq_cont(starto_hori + 2, starto_vert) = p * pascal[0] * pow(s[i+1], 1-j);
                break;
            
            case 2://a
                Aeq_cont(starto_hori - 2, starto_vert) = p * pascal[0] * pow(s[i+0], 1-j);
                Aeq_cont(starto_hori - 1, starto_vert) = p * pascal[1] * pow(s[i+0], 1-j);
                Aeq_cont(starto_hori - 0, starto_vert) = p * pascal[2] * pow(s[i+0], 1-j);

                Aeq_cont(starto_hori + 1, starto_vert) = p * pascal[2] * pow(s[i+1], 1-j);
                Aeq_cont(starto_hori + 2, starto_vert) = p * pascal[1] * pow(s[i+1], 1-j);
                Aeq_cont(starto_hori + 3, starto_vert) = p * pascal[0] * pow(s[i+1], 1-j);

            case 3://j
                Aeq_cont(starto_hori - 3, starto_vert) = p * pascal[0] * pow(s[i+0], 1-j);
                Aeq_cont(starto_hori - 2, starto_vert) = p * pascal[1] * pow(s[i+0], 1-j);
                Aeq_cont(starto_hori - 1, starto_vert) = p * pascal[2] * pow(s[i+0], 1-j);
                Aeq_cont(starto_hori - 0, starto_vert) = p * pascal[3] * pow(s[i+0], 1-j);

                Aeq_cont(starto_hori + 1, starto_vert) = p * pascal[3] * pow(s[i+1], 1-j);
                Aeq_cont(starto_hori + 2, starto_vert) = p * pascal[2] * pow(s[i+1], 1-j);
                Aeq_cont(starto_hori + 3, starto_vert) = p * pascal[1] * pow(s[i+1], 1-j);
                Aeq_cont(starto_hori - 4, starto_vert) = p * pascal[0] * pow(s[i+1], 1-j);
            
            default:
                ROS_ERROR("Please Re-select Minimization Order: with Max. Order Snap(4)!");
                break;
            }

        }
    }

    A_eq.resize(Aeq_start.rows() + Aeq_end.rows() + Aeq_cont.rows(), _dim);
    A_eq << Aeq_start,
            Aeq_end,
            Aeq_cont;
}

void bernstein::setAieq(int n_order, int m, int d_order, vector<double> s)
{

}

void bernstein::setUBeq(endpt_cond start, endpt_cond end, int n_order, int m, int d_order)
{
    //set BUeq_start
    Eigen::VectorXd BUeq_start;
    BUeq_start.resize(3);
    
    BUeq_start(0) = start.p_;
    BUeq_start(1) = start.v_;
    BUeq_start(2) = start.a_;

    //set BUeq_end
    Eigen::VectorXd BUeq_end;
    BUeq_end.resize(3);

    BUeq_end(0) = end.p_;
    BUeq_end(1) = end.v_;
    BUeq_end(2) = end.a_;

    //set BUeq_cont
    Eigen::VectorXd BUeq_cont;
    BUeq_cont.resize((m - 1) * d_order);

    BUeq_cont.setZero();

    //combine BUeq_start, BU_end, BU_cont
    ub_eq.resize(BUeq_start.size() + BUeq_end.size() + BUeq_cont.size());
    ub_eq << BUeq_start,
             BUeq_end,
             BUeq_cont;

}

void bernstein::setUBieq(vector<corridor> cube_list, dynamic_constraints d_constraints, int n_order, int m, int d_order)
{
    Eigen::VectorXd UBieq_p;
    int _dim_p = (m * n_order) + 1;
    UBieq_p.resize(_dim_p);

    int starto = 0;
    for(int i = 0; i < cube_list.size(); i++)//cube_list.size() = size of corridor = m
    {
        for(int j = 0; j < n_order + 1; j++) //each control point, n_order + 1 = size of ctrl_pts per segment
        {
            int _i = starto + j;
            UBieq_p(_i) = cube_list[i].x_max;
        }
        
        starto = starto + (n_order + 1);
    }
        
    
    Eigen::VectorXd UBieq_v;
    int _dim_v = (m * n_order) + 1 - 1;
    UBieq_v.resize(_dim_v);

    for(int i = 0; i < _dim_v; i++)
        UBieq_v(i) = d_constraints.v_max;


    Eigen::VectorXd UBieq_a;
    int _dim_a = (m * n_order) + 1 - 2;
    UBieq_a.resize(_dim_a);

    for(int i = 0; i < _dim_a; i++)
        UBieq_a(i) = d_constraints.a_max;

    
    Eigen::VectorXd UBieq_j;
    int _dim_j = (m * n_order) + 1 - 3;
    UBieq_j.resize(_dim_j);
    
    for(int i = 0; i < _dim_j; i++)
        UBieq_j(i) = d_constraints.j_max;

    
    switch (d_order)
    {
    case 2:
        ub_ieq.resize(UBieq_p.size() + UBieq_v.size());
        ub_ieq << UBieq_p,
                  UBieq_v;
        break;
    
    case 3:
        ub_ieq.resize(UBieq_p.size() + UBieq_v.size() + UBieq_a.size());
        ub_ieq << UBieq_p,
                  UBieq_v,
                  UBieq_a;
        break;
    
    case 4:
        ub_ieq.resize(UBieq_p.size() + UBieq_v.size() + UBieq_a.size() + UBieq_j.size());
        ub_ieq << UBieq_p,
                  UBieq_v,
                  UBieq_a,
                  UBieq_j;
        break;
    
    default:
        ROS_ERROR("Re-Select d_order:\n\t2 for min. accl,\n\t 3 for min. jerk,\n\t 4 for min. snap\n");
        break;
    }
}

void bernstein::setLBeq(endpt_cond start, endpt_cond end, int n_order, int m, int d_order)
{
    //set BLeq_start
    Eigen::VectorXd BLeq_start;
    BLeq_start.resize(3);
    
    BLeq_start(0) = start.p_;
    BLeq_start(1) = start.v_;
    BLeq_start(2) = start.a_;

    //set BLeq_end
    Eigen::VectorXd BLeq_end;
    BLeq_end.resize(3);

    BLeq_end(0) = end.p_;
    BLeq_end(1) = end.v_;
    BLeq_end(2) = end.a_;

    //set BLeq_cont
    Eigen::VectorXd BLeq_cont;

    BLeq_cont.resize((m - 1) * d_order);
    BLeq_cont.setZero();

    //combine BLeq_start, BLeq_end, BLeq_cont
    lb_eq.resize(BLeq_start.size() + BLeq_end.size() + BLeq_cont.size());
    lb_eq << BLeq_start,
             BLeq_end,
             BLeq_cont;
             
}

void bernstein::setLBieq(vector<corridor> cube_list, dynamic_constraints d_constraints, int n_order, int m, int d_order)
{
    Eigen::VectorXd LBieq_p;
    int _dim_p = (m * n_order) + 1;
    LBieq_p.resize(_dim_p);

    int starto = 0;
    for(int i = 0; i < cube_list.size(); i++)//cube_list.size() = size of corridor = m
    {
        for(int j = 0; j < n_order + 1; j++) //each control point, n_order + 1 = size of ctrl_pts per segment
        {
            int _i = starto + j;
            LBieq_p(_i) = cube_list[i].x_min;
        }
        
        starto = starto + (n_order + 1);
    }
        
    
    Eigen::VectorXd LBieq_v;
    int _dim_v = (m * n_order) + 1 - 1;
    LBieq_v.resize(_dim_v);

    for(int i = 0; i < _dim_v; i++)
        LBieq_v(i) = d_constraints.v_min;


    Eigen::VectorXd LBieq_a;
    int _dim_a = (m * n_order) + 1 - 2;
    LBieq_a.resize(_dim_a);

    for(int i = 0; i < _dim_a; i++)
        LBieq_a(i) = d_constraints.a_min;

    
    Eigen::VectorXd LBieq_j;
    int _dim_j = (m * n_order) + 1 - 3;
    LBieq_j.resize(_dim_j);
    
    for(int i = 0; i < _dim_j; i++)
        LBieq_j(i) = d_constraints.j_min;

    
    switch (d_order)
    {
    case 2:
        ub_ieq.resize(LBieq_p.size() + LBieq_v.size());
        ub_ieq << LBieq_p,
                  LBieq_v;
        break;
    
    case 3:
        ub_ieq.resize(LBieq_p.size() + LBieq_v.size() + LBieq_a.size());
        ub_ieq << LBieq_p,
                  LBieq_v,
                  LBieq_a;
        break;
    
    case 4:
        ub_ieq.resize(LBieq_p.size() + LBieq_v.size() + LBieq_a.size() + LBieq_j.size());
        ub_ieq << LBieq_p,
                  LBieq_v,
                  LBieq_a,
                  LBieq_j;
        break;
    
    default:
        ROS_ERROR("Re-Select d_order:\n\t2 for min. accl,\n\t 3 for min. jerk,\n\t 4 for min. snap\n");
        break;
    }


}

void bernstein::setMQM(int n_order, int m, int d_order, vector<double> s)
{
    setQM(n_order, m, d_order, s);
    if(M.size() != Q.size())
    {
        cout<<"Q & M dimension not corresponding"<<endl;
        return;
    }
    
    MQM.resize(Q.rows(), Q.cols());
    MQM = M.transpose() * Q * M;

    cout<<MQM<<endl;
    // MQM_spd = getSPD(MQM);

}


void bernstein::setQM(int n_order, int m, int d_order, vector<double> s)
{
    if(s.size() != m)
    {
        cout<<"size does not correspond!"<<endl;
        return;
    }
    int n_dim = (n_order + 1) * m;

    Q.resize(n_dim, n_dim);
    Q.setZero();
    M.resize(n_dim, n_dim);
    M.setZero();
    // cout<<Q.rows()<<endl;
    // cout<<Q.cols()<<endl;

    Q_temp.resize(n_order + 1, n_order + 1);
    Q_temp.setZero();
    M_temp.resize(n_order + 1, n_order + 1);
    M_temp.setZero();
    int starto = 0;
    
    for(int t = 0; t < m; t++)
    {        
        for(int i = 0; i < Q_temp.rows(); i++)
        {
            for(int j = 0; j < Q_temp.cols(); j++)
            {
                if(i < 4 || j < 4)
                    Q_temp(i, j) = 0;
                else
                    Q_temp(i, j) = i * (i-1) * (i-2) * (i-3)
                                * j * (j-1) * (j-2) * (j-3)
                                * pow(s[t], (-2 * d_order + 3))  
                                / (i+j-7);
            }
        }

        setM(n_order);
        M.block(starto, starto, n_order + 1, n_order + 1) = M_temp;

        Q.block(starto, starto, n_order + 1, n_order + 1) = Q_temp;
        starto = t * (n_order + 1);
    }

}

void bernstein::setM(int order)
{
    M_temp.resize(order + 1, order + 1);
    switch (order)
    {
    case 0: 
    {
        M_temp << 1;
        break;

    }
    case 1: 
    {
        M_temp << -1,  0,
                -1,  1;
        break;

    }
    case 2:
    {
        M_temp << -1,  0,  0,
                -2,  2,  0,
                1, -2,  1;
        break;

    }
    case 3: 
    {
        M_temp << -1,  0,  0,  0,
                -3,  3,  0,  0,
                3, -6,  3,  0,
                -1,  3, -3,  1;	
        break;

    }
    case 4:
    {
        M_temp <<  1,   0,   0,   0,  0,
                -4,   4,   0,   0,  0,
                6, -12,   6,   0,  0,
                -4,  12, -12,   4,  0,
                1,  -4,   6,  -4,  1;
        break;
    }
    case 5:
    {
        M_temp << 1,   0,   0,   0,  0,  0,
            -5,   5,   0,   0,  0,  0,
            10, -20,  10,   0,  0,  0,
            -10,  30, -30,  10,  0,  0,
                5, -20,  30, -20,  5,  0,
            -1,   5, -10,  10, -5,  1;
        break;
    }
    case 6:
    {	

        M_temp << 1,   0,   0,   0,   0,  0,  0,
            -6,   6,   0,   0,   0,  0,  0,
            15, -30,  15,   0,   0,  0,  0,
            -20,  60, -60,  20,   0,  0,  0,
            15, -60,  90, -60,  15,  0,  0,
            -6,  30, -60,  60, -30,  6,  0,
                1,  -6,  15, -20,  15, -6,  1;
        break;
    }
    case 7:
    {
        M_temp << 1,    0,    0,    0,    0,   0,   0,   0,
            -7,    7,    0,    0,    0,   0,   0,   0,
            21,   42,   21,    0,    0,   0,   0,   0,
            -35,  105, -105,   35,    0,   0,   0,   0, 
            35, -140,  210, -140,   35,   0,   0,   0,
            -21,  105, -210,  210, -105,  21,   0,   0,
                7,  -42,  105, -140,  105, -42,   7,   0,
            -1,    7,  -21,   35,  -35,  21,  -7,   1;
        break;
    }
    case 8:
    {
        M_temp << 1,    0,    0,    0,    0,    0,   0,   0,   0,
            -8,    8,    0,    0,    0,    0,   0,   0,   0,
            28,  -56,   28,    0,    0,    0,   0,   0,   0,
            -56,  168, -168,   56,    0,    0,   0,   0,   0, 
            70, -280,  420, -280,   70,    0,   0,   0,   0,
            -56,  280, -560,  560, -280,   56,   0,   0,   0,
            28, -168,  420, -560,  420, -168,  28,   0,   0,
            -8,   56, -168,  280, -280,  168, -56,   8,   0,
                1,   -8,   28,  -56,   70,  -56,  28,  -8,   1;
        break;
    }
    case 9:
    {
        M_temp << 1,    0,     0,     0,     0,    0,    0,     0,     0,    0,
            -9,    9,     0,     0,     0,    0,    0,     0,     0,    0, 
            36,  -72,    36,     0,     0,    0,    0,     0,     0,    0, 
            -84,  252,  -252,    84,     0,    0,    0,     0,     0,    0, 
            126, -504,   756,  -504,   126,    0,    0,     0,     0,    0,
            -126,  630, -1260,  1260,  -630,  126,    0,     0,     0,    0,
            84, -504,  1260, -1680,  1260, -504,   84,     0,     0,    0,
            -36,  252,  -756,  1260, -1260,  756, -252,    36,     0,    0,
                9,  -72,   252,  -504,   630, -504,  252,   -72,     9,    0,
            -1,    9,   -36,    84,  -126,  126,  -84,    36,    -9,    1;
        break;
    }
    case 10:
    {
        M_temp <<  1,     0,     0,     0,      0,     0,    0,     0,     0,    0,   0,
            -10,    10,     0,     0,      0,     0,    0,     0,     0,    0,   0,
                45,   -90,    45,     0,      0,     0,    0,     0,     0,    0,   0,
            -120,   360,  -360,   120,      0,     0,    0,     0,     0,    0,   0,
            210,  -840,  1260,  -840,    210,     0,    0,     0,     0,    0,   0,
            -252,  1260, -2520,  2520,  -1260,   252,    0,     0,     0,    0,   0,
            210, -1260,  3150, -4200,   3150, -1260,  210,     0,     0,    0,   0,
            -120,  840,  -2520,  4200,  -4200,  2520, -840,   120,     0,    0,   0,
                45, -360,   1260, -2520,   3150, -2520, 1260,  -360,    45,    0,   0,
            -10,   90,   -360,   840,  -1260,  1260, -840,   360,   -90,   10,   0,
                1,  -10,     45,  -120,    210,  -252,  210,  -120,    45,  -10,   1;
        break;
    }
    case 11:
    {
        M_temp <<  1,     0,    0,      0,      0,      0,     0,     0,     0,    0,   0,  0,
            -11,    11,    0,      0,      0,      0,     0,     0,     0,    0,   0,  0,
                55,  -110,   55,      0,      0,      0,     0,     0,     0,    0,   0,  0,
            -165,   495, -495,    165,      0,      0,     0,     0,     0,    0,   0,  0,
            330, -1320, 1980,  -1320,    330,      0,     0,     0,     0,    0,   0,  0,
            -462,  2310, -4620,  4620,  -2310,    462,     0,     0,     0,    0,   0,  0,
            462, -2772,  6930, -9240,   6930,  -2772,   462,     0,     0,    0,   0,  0,
            -330,  2310, -6930, 11550, -11550,   6930, -2310,   330,     0,    0,   0,  0,
            165, -1320,  4620, -9240,  11550,  -9240,  4620, -1320,   165,    0,   0,  0,
            -55,   495, -1980,  4620,  -6930,   6930, -4620,  1980,  -495,   55,   0,  0,
                11,  -110,   495, -1320,   2310,  -2772,  2310, -1320,   495, -110,  11,  0,
                -1,    11,   -55,   165,   -330,    462,  -462,   330,  -165,   55, -11,  1;
        break;
    }
    case 12:
    {
        M_temp <<  1,     0,     0,      0,      0,      0,     0,     0,     0,    0,    0,   0,   0,
            -12,    12,     0,      0,      0,      0,     0,     0,     0,    0,    0,   0,   0,
                66,  -132,    66,      0,      0,      0,     0,     0,     0,    0,    0,   0,   0,
            -220,   660,  -660,    220,      0,      0,     0,     0,     0,    0,    0,   0,   0,
            495, -1980,  2970,  -1980,    495,      0,     0,     0,     0,    0,    0,   0,   0, 
            -792,  3960, -7920,   7920,  -3960,    792,     0,     0,     0,    0,    0,   0,   0,
            924, -5544, 13860, -18480,  13860,  -5544,   924,     0,     0,    0,    0,   0,   0,
            -792,  5544,-16632,  27720, -27720,  16632, -5544,   792,     0,    0,    0,   0,   0,
            495, -3960, 13860, -27720,  34650, -27720, 13860, -3960,   495,    0,    0,   0,   0,
            -220,  1980, -7920,  18480, -27720,  27720,-18480,  7920, -1980,  220,    0,   0,   0,
                66,  -660,  2970,  -7920,  13860, -16632, 13860, -7920,  2970, -660,   66,   0,   0,
            -12,   132,  -660,   1980,  -3960,   5544, -5544,  3960, -1980,  660, -132,  12,   0,
                1,   -12,    66,   -220,    495,   -792,   924,  -792,   495, -220,   66, -12,   1;
        break;
    }
    }
}

inline double bernstein::permutation(int p, int q)
{
    return factorial(p) / factorial(q);
}

inline double bernstein::factorial(int r)
{
    double _r = 1;
    for(int i = 0 ; i < r; i++)
        _r = _r * i;
    
    return _r;
}

vector<double> bernstein::pascal_triangle(int level)
{
    vector<double> _array;
    switch(level)
    {
    case 1: 
    {
        _array.push_back(1);
        break;

    }
    case 2: 
    {
        _array.push_back(1 * (-1));
        _array.push_back(1);
        break;

    }
    case 3:
    {
        _array.push_back(1);
        _array.push_back(2 * (-1));
        _array.push_back(1);
        break;

    }
    case 4:
    {
        _array.push_back(1 * (-1));
        _array.push_back(3);
        _array.push_back(3 * (-1));
        _array.push_back(1);
        break;

    }
    case 5:
    {
        _array.push_back(1);
        _array.push_back(4 * (-1));
        _array.push_back(6);
        _array.push_back(4 * (-1));
        _array.push_back(1);
        break;

    }
    case 7:
    {
        _array.push_back(1 * (-1));
        _array.push_back(5);
        _array.push_back(10* (-1));
        _array.push_back(10);
        _array.push_back(5 * (-1));
        _array.push_back(1);
        break;

    }
    }

}


#endif