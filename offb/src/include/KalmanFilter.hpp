#include <Eigen/Dense>
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
        Eigen::MatrixXd F, 
        Eigen::MatrixXd P, 
        Eigen::MatrixXd Q,  
        Eigen::MatrixXd H,
        Eigen::MatrixXd R,
        kf_type which,
        Eigen::Vector2i dim //[0] state [1] measurement
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
    Eigen::MatrixXd F, 
    Eigen::MatrixXd P, 
    Eigen::MatrixXd Q,  
    Eigen::MatrixXd H,
    Eigen::MatrixXd R,
    kf_type which,
    Eigen::Vector2i dim
)
{
    this->Fk = F;
    this->Pk = P;
    this->Qk = Q;
    this->Hk = H;
    this->Rk = R;
    this->state_size = dim[0];
    this->mea_size   = dim[1];
    this->type       = which;
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
    Eigen::MatrixXd F(state_size, state_size);
    F << 1, 0, 0, dt,  0,  0,
         0, 1, 0,  0, dt,  0,
         0, 0, 1,  0,  0, dt,
         0, 0, 0,  1,  0,  0,
         0, 0, 0,  0,  0,  1;

    xk = F * x;
    Pk_bar = F * Pk * F.transpose();
    
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
    Eigen::MatrixXd F(state_size, state_size);
    F << 1, 0, 0, dt,  0,  0,
         0, 1, 0,  0, dt,  0,
         0, 0, 1,  0,  0, dt,
         0, 0, 0,  1,  0,  0,
         0, 0, 0,  0,  0,  1;

    xk = F * x;
    Pk_bar = F * Pk * F.transpose();
    
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
    Eigen::MatrixXd F(state_size, state_size);
    F << 1, 0, 0, dt,  0,  0,
         0, 1, 0,  0, dt,  0,
         0, 0, 1,  0,  0, dt,
         0, 0, 0,  1,  0,  0,
         0, 0, 0,  0,  0,  1;

    xk = F * x;
    Pk_bar = F * Pk * F.transpose();
    
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