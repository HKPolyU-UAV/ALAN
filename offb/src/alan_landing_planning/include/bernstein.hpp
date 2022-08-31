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

/*!
 * @struct      bezier_constraints
 * @abstract    bezier_constraints input.

 * @field       
 * @field       
 * @field       
*/
typedef struct bezier_constraints 
{
    double p_start;
    double v_start;
    double a_start;

    double p_end;
    double v_end;
    double a_end;
        
    //double
    vector<corridor> cube_list;
    
    double v_max;
    double v_min;

    double a_max;
    double a_min;
     
}bezier_constraints;



class bernstein
{
private:
    Eigen::MatrixXd Q, Q_temp, M, M_temp, MQM, MQM_spd;
    Eigen::MatrixXd getQ()
    {
        return Q;
    }
    
    Eigen::MatrixXd getM()
    {
        return M;
    }

    Eigen::MatrixXd getMQM_spd()
    {
        return MQM_spd;
    }

    void setM(int n_order);
    void setQM(int n_order, int m, int d_order, vector<double> s);

    Eigen::MatrixXd getSPD(Eigen::MatrixXd Q);

    
public:
    bernstein(/* args */);
    ~bernstein();



    void setMQM(int n_order, int m, int d_order, vector<double> s)
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

    
    Eigen::MatrixXd getMQM()
    {
        return MQM;
    }


    

};

bernstein::bernstein(/* args */)
{
}

bernstein::~bernstein()
{
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

#endif