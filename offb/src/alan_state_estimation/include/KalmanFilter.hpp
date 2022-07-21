#include "essential.h"

enum kf_type
{
    linear,
    extended,
    unscented,
};

class KalmanFilter
{
private:
    Eigen::MatrixXd Fk, Pk, Pk_bar, Qk, Hk, Rk, xk;
    int state_size, mea_size;
    kf_type type;
    void lkf_predict(Eigen::VectorXd& x, float dt);
    void lkf_update(Eigen::VectorXd& x, Eigen::VectorXd z);
    void ekf_predict(Eigen::VectorXd& x, float dt);
    void ekf_update(Eigen::VectorXd& x, Eigen::VectorXd z);
    void ukf_predict(Eigen::VectorXd& x, float dt);
    void ukf_update(Eigen::VectorXd& x, Eigen::VectorXd z);
public:
    KalmanFilter(
        Eigen::MatrixXd& F, 
        Eigen::MatrixXd& P, 
        Eigen::MatrixXd& Q,  
        Eigen::MatrixXd& H,
        Eigen::MatrixXd& R,
        kf_type which,
        int n,
        int m //[0] state [1] measurement
    );
    ~KalmanFilter();

    void predict(
        Eigen::VectorXd& x,
        float dt
    );
    void update(
        Eigen::VectorXd& x,
        Eigen::VectorXd zk
    );

};

KalmanFilter::KalmanFilter(
    Eigen::MatrixXd& F, 
    Eigen::MatrixXd& P, 
    Eigen::MatrixXd& Q,  
    Eigen::MatrixXd& H,
    Eigen::MatrixXd& R,
    kf_type which,
    int n,
    int m
) : state_size(n), mea_size(m), type(which)
{
    //for convenience, all matrix are manually input first here

    F = Eigen::MatrixXd::Identity(n,n);
    
    P = Eigen::MatrixXd::Zero(n,n);
    P << 
        1.0, 0, 0, 0, 0, 0,
        0, 1.0, 0, 0, 0, 0,
        0, 0, 1.0, 0, 0, 0,
        0, 0, 0, 5.0, 0, 0,
        0, 0, 0, 0, 5.0, 0,
        0, 0, 0, 0, 0, 5.0;

    Q = Eigen::MatrixXd::Zero(n,n);
    Q << 
        0.001, 0, 0, 0, 0, 0,
        0, 0.001, 0, 0, 0, 0,
        0, 0, 0.001, 0, 0, 0,
        0, 0, 0, 0.100, 0, 0,
        0, 0, 0, 0, 0.100, 0,
        0, 0, 0, 0, 0, 0.100;

    H = Eigen::MatrixXd::Zero(m,n);
    H(0,0) = H(1,1) = H(2,2) = 1;

    R = Eigen::MatrixXd::Zero(m,m);
    R << 
        0.05, 0, 0,
        0, 0.05, 0,
        0, 0, 1.00;

    cerr << "initiation succeed" << endl;

    
}

KalmanFilter::~KalmanFilter(){}

void KalmanFilter::predict(Eigen::VectorXd& x, float dt)
{
    if(type == linear)
        lkf_predict(x, dt);
    else if (type == extended)
        ekf_predict(x, dt);
    else if (type == unscented)
        ukf_predict(x, dt);
    else
        cout<< "Please identify Kalman filter type!" <<endl;
}

void KalmanFilter::update(Eigen::VectorXd& x, Eigen::VectorXd zk)
{
    if(type == linear)
        lkf_update(x, zk);
    else if (type == extended)
        ekf_update(x, zk);
    else if (type == unscented)
        ukf_update(x, zk);
    else
        cout<< "Please identify Kalman filter type!" <<endl;
}

void KalmanFilter::lkf_predict(Eigen::VectorXd& x, float dt)
{
    // Eigen::MatrixXd F(state_size, state_size);
    // F << 1, 0, 0, dt,  0,  0,
    //      0, 1, 0,  0, dt,  0,
    //      0, 0, 1,  0,  0, dt,
    //      0, 0, 0,  1,  0,  0,
    //      0, 0, 0,  0,  0,  1;

    Fk(0,3) = Fk(1,4) = Fk(2,5) = dt;

    xk = Fk * x;
    Pk_bar = Fk * Pk * Fk.transpose();
    
    x = xk;
}

void KalmanFilter::lkf_update(Eigen::VectorXd& x, Eigen::VectorXd zk)
{
    Eigen::MatrixXd Sk = Hk * Pk_bar * Hk.transpose() + Rk; // system uncertainty
    Eigen::MatrixXd Kk = Pk_bar * Hk * Sk.inverse(); //Kalman Gain
    Eigen::VectorXd yk = zk - Hk * x; //residual

    x = x + Kk * yk;
    
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(state_size, state_size) - Kk * Hk;
    Pk = I_KH * Pk_bar * I_KH.transpose() + Kk * Rk * Kk.transpose();

}

void KalmanFilter::ekf_predict(Eigen::VectorXd& x, float dt)
{
    // Eigen::MatrixXd F(state_size, state_size);
    // F << 1, 0, 0, dt,  0,  0,
    //      0, 1, 0,  0, dt,  0,
    //      0, 0, 1,  0,  0, dt,
    //      0, 0, 0,  1,  0,  0,
    //      0, 0, 0,  0,  0,  1;

    xk = Fk * x;
    Pk_bar = Fk * Pk * Fk.transpose();
    
    x = xk;
}

void KalmanFilter::ekf_update(Eigen::VectorXd& x, Eigen::VectorXd zk)
{
    Eigen::MatrixXd Sk = Hk * Pk_bar * Hk.transpose() + Rk; // system uncertainty
    Eigen::MatrixXd Kk = Pk_bar * Hk * Sk.inverse(); //Kalman Gain
    Eigen::VectorXd yk = zk - Hk * x; //residual

    x = x + Kk * yk;
    
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(state_size, state_size) - Kk * Hk;
    Pk = I_KH * Pk_bar * I_KH.transpose() + Kk * Rk * Kk.transpose();

}

void KalmanFilter::ukf_predict(Eigen::VectorXd& x, float dt)
{
    // Eigen::MatrixXd F(state_size, state_size);
    // F << 1, 0, 0, dt,  0,  0,
    //      0, 1, 0,  0, dt,  0,
    //      0, 0, 1,  0,  0, dt,
    //      0, 0, 0,  1,  0,  0,
    //      0, 0, 0,  0,  0,  1;

    xk = Fk * x;
    Pk_bar = Fk * Pk * Fk.transpose();
    
    x = xk;
}

void KalmanFilter::ukf_update(Eigen::VectorXd& x, Eigen::VectorXd zk)
{
    Eigen::MatrixXd Sk = Hk * Pk_bar * Hk.transpose() + Rk; // system uncertainty
    Eigen::MatrixXd Kk = Pk_bar * Hk * Sk.inverse(); //Kalman Gain
    Eigen::VectorXd yk = zk - Hk * x; //residual

    x = x + Kk * yk;
    
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(state_size, state_size) - Kk * Hk;
    Pk = I_KH * Pk_bar * I_KH.transpose() + Kk * Rk * Kk.transpose();

}




// Vec7 GenerateCameraLP(Vec7 UGV_lp){ /* Special thanks to Jazzy Dorian Feng */
//     Eigen::Quaterniond CARq;
//     CARq.w() = UGV_lp[3];
//     CARq.x() = UGV_lp[4];
//     CARq.y() = UGV_lp[5];
//     CARq.z() = UGV_lp[6];
//     Eigen::Matrix3d CAR_Rotation_world = Eigen::Matrix3d::Identity();
//     CAR_Rotation_world = CARq.matrix();
//     Vec3 CAR_Translation_world(UGV_lp[0], UGV_lp[1], UGV_lp[2]);
//     SE3 T_ugv(CAR_Rotation_world, CAR_Translation_world);

//     Mat3x3 r;
//     Vec3 t;
//     r<< 0.0342161,   -0.334618,  -0.941732,
//         0.999403,     0.0159477,  0.0306448,
//         0.00476414,  -0.942219,   0.334964;
//     t << 0.567003, -0.018069, 0.0174849;
//     SE3 T_ugv_cam(r,t);
//     SE3 T_cam = T_ugv * T_ugv_cam;

//     Vec7 CAM_lp;
//     CAM_lp << T_cam.translation().x(), T_cam.translation().y(), T_cam.translation().z(),
//               T_cam.unit_quaternion().w(), T_cam.unit_quaternion().x(), T_cam.unit_quaternion().y(), T_cam.unit_quaternion().z();
//     return(CAM_lp);
// }