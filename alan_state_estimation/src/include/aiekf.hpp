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
 * \file aiekf.hpp
 * \date 24/08/2023
 * \author pattylo
 * \copyright (c) AIRO-Lab, RCUAS of Hong Kong Polytechnic University
 * \brief adaptive, iterated, extended Kalman filter for passive pose tracking
 */

#ifndef AIEKF_HPP
#define AIEKF_HPP

#include "tools/essential.h"
#include "cameraModel.hpp"
#include <sophus/se3.hpp>

namespace kf
{
    typedef struct STATE
    {
        Sophus::SE3d X_SE3;
        Sophus::SE3d V_SE3;
        Eigen::MatrixXd PMatrix;
    } STATE;

    typedef struct MEASUREMENT
    {
        Sophus::SE3d pose_initial_SE3;
        std::vector<Eigen::Vector2d> pts_2d_detected;
        std::vector<Eigen::Vector3d> pts_3d_exists;
    } MEASUREMENT;

    typedef struct FINAL_RETURN
    {
        STATE X;
        Eigen::MatrixXd COV_MAT;

        double residual_error;
        double delta_now_then;
    }FINAL_RETURN;


    class aiekf : public vision::cameraModel
    {
    private:
        /* ================ main flow ================ */
        
        STATE X_current_posterori;       // X @ k
        STATE X_previous_posterori;      // X @ k - 1
        STATE X_current_dynamic_priori;  // X @ k, priori
        STATE X_current_camera_priori;   // X @ k, priori 
        int trackedSize = 0;

        Eigen::MatrixXd F_k;
        Eigen::MatrixXd H_k;

        Eigen::MatrixXd K_k;
        
        Eigen::MatrixXd Q_init;
        Eigen::MatrixXd R_init;
        Eigen::MatrixXd Q_k;
        Eigen::MatrixXd R_k;

        MEASUREMENT Z_current_meas;      // Z @ k        

        void setPredict(); //set_X_current_dynamic_priori
        void doOptimize(MEASUREMENT meas_at_k);
        void setPostOptimize(MEASUREMENT meas_at_k);

        /* ================ utilities ================ */
        double deltaT = 0;
        Eigen::Vector3d g = {0,0,-9.81};
        double QAdaptiveAlpha = 0;
        double RAdaptiveBeta = 0;

        Eigen::VectorXd dfx();
        void setKalmanGain();
        void setAdaptiveQ(MEASUREMENT meas_at_k);
        void setAdaptiveR(MEASUREMENT meas_at_k);
        void setJacobianDynamic(Eigen::MatrixXd& Jacob);
        void setJacobianCamera(Eigen::MatrixXd& Jacob, MEASUREMENT meas_at_k);
        Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d w);

    public:
        aiekf(){};
        ~aiekf(){
            std::cout<<"EXIT AIEKF"<<std::endl;
        };

        void run_AIEKF(MEASUREMENT meas_at_k, double deltaT_);
        void initKF(
            MEASUREMENT meas_at_k,
            Eigen::MatrixXd Q_init_,
            Eigen::MatrixXd R_init_,
            double QAdaptiveAlpha_,
            double RAdaptiveBeta_
        );
        void reinitKF(MEASUREMENT meas_at_k);

    };

}

void kf::aiekf::initKF(
    MEASUREMENT meas_at_k,
    Eigen::MatrixXd Q_init_,
    Eigen::MatrixXd R_init_,
    double QAdaptiveAlpha_,
    double RAdaptiveBeta_
)
{
    X_previous_posterori.X_SE3 = meas_at_k.pose_initial_SE3;
    X_previous_posterori.V_SE3 = Sophus::SE3d(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d::Zero()
    );
    trackedSize = 
          X_previous_posterori.X_SE3.log().size()
        + X_previous_posterori.V_SE3.log().head<3>().size();

    X_previous_posterori.PMatrix.resize(trackedSize, trackedSize);
    X_previous_posterori.PMatrix.setZero();
    X_previous_posterori.PMatrix.block<6,6>(0,0).setConstant(0.1);
    X_previous_posterori.PMatrix.block<6,6>(0,0).setConstant(1.0);


    if(trackedSize != Q_init_.rows())
    {
        ROS_RED_STREAM("KF DIMENSION DOES NOT MATCH!!!");
        return;
    }
            
    Q_k = this->Q_init = Q_init_;
    R_k = this->R_init = R_init_;

    QAdaptiveAlpha = QAdaptiveAlpha_;
    RAdaptiveBeta  = RAdaptiveBeta_;

    setAdaptiveQ(meas_at_k);
    setAdaptiveR(meas_at_k);
}

void kf::aiekf::reinitKF(MEASUREMENT meas_at_k)
{
    X_previous_posterori.X_SE3 = meas_at_k.pose_initial_SE3;
    X_previous_posterori.V_SE3 = Sophus::SE3d(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d::Zero()
    );

    Q_k = Q_init;
    R_k = R_init;

    setAdaptiveQ(meas_at_k);
    setAdaptiveR(meas_at_k);
    // X_previous_posterori.size = X_previous_posterori.size;
}

void kf::aiekf::run_AIEKF(MEASUREMENT meas_at_k, double deltaT)
{
    setPredict();    
    doOptimize(meas_at_k);
    setPostOptimize(meas_at_k);
}

void kf::aiekf::setPredict()
{
    Eigen::VectorXd dx = dfx() * deltaT; // first 6 element: se(3) of pose
                                   // last 3 element: velocity

    X_current_dynamic_priori.X_SE3 = 
        X_previous_posterori.X_SE3 * Sophus::SE3d::exp(dx.head<6>());
    
    X_current_dynamic_priori.V_SE3.translation() = 
        X_previous_posterori.V_SE3.translation() + dx.tail<3>();

    setJacobianDynamic(F_k);

    X_current_dynamic_priori.PMatrix = F_k * X_previous_posterori.PMatrix * F_k.transpose();
}



void kf::aiekf::setPostOptimize(MEASUREMENT meas_at_k)
{
    // update velocity
    setJacobianCamera(H_k, meas_at_k);

    setKalmanGain();
    setAdaptiveQ(meas_at_k);
    setAdaptiveR(meas_at_k);
}

void kf::aiekf::doOptimize(MEASUREMENT meas_at_k)
{
    const int MAX_ITERATION = 40;

    const double converge_threshold = 1e-6;

    const int points_no = meas_at_k.pts_2d_detected.size();

    Eigen::Matrix<double, 2, 6> J;
    Eigen::Matrix<double, 6, 6> A; // R6*6
    Eigen::Matrix<double, 6, 1> b; // R6
    Eigen::Vector2d e; // R2
    Eigen::Matrix<double, 6, 1> dx;

    Sophus::SE3d pose;//= Sophus::SE3d::exp(meas_at_k.pose_initial_se3);

    int i;
    double cost = 0, lastcost = INFINITY;
    
    for(i = 0; i < MAX_ITERATION; i++)
    {
        A.setZero();
        b.setZero();

        cost = 0;
        

        for(int i=0; i < points_no; i++)
        {
            //get the Jacobian for this point
            solveJacobianCamera(J, pose, meas_at_k.pts_3d_exists[i]);

            e = meas_at_k.pts_2d_detected[i] - reproject_3D_2D(meas_at_k.pts_3d_exists[i], pose) ; 
            
            cost += e.norm();

            //form Ax = b
            A += J.transpose() * J;
            b += -J.transpose() * e;
        }
    
        //solve Adx = b
        dx = A.ldlt().solve(b);
        //

        for(int i = 0; i < 6; i++)
            if(isnan(dx(i,0)))
                break;


        pose = Sophus::SE3d::exp(dx) * pose;

        lastcost = cost;

        if(dx.norm() < converge_threshold)        
            break;
    }

    // BA_error = lastcost;

    // cout<<"gone thru: "<<i<<" th, end optimize"<<endl<<endl;;;

}


/* ================ utilities ================ */

Eigen::VectorXd kf::aiekf::dfx()
{
    // x_dot = Ax here  
    // i.e., give x_dot here, basically (in se(3))
    Eigen::VectorXd returnDfx; // first 6 element: se(3) of pose
                               // last 3 element: velocity 
    returnDfx.resize(trackedSize);
    
    X_previous_posterori.V_SE3.log();
    
    returnDfx.head<Sophus::SE3d::DoF>() = 
        Sophus::SE3d(
            Eigen::Matrix3d::Identity(),
            X_previous_posterori.V_SE3.translation()
        ).log();
    returnDfx.tail<Sophus::SE3d::DoF/2>() = X_previous_posterori.X_SE3.rotationMatrix() * g; 

    return returnDfx;
    
}

void kf::aiekf::setKalmanGain()
{
    K_k = X_current_dynamic_priori.PMatrix // R 9*9
        * H_k.transpose()                  // R 9*2
        * (H_k * X_current_camera_priori.PMatrix * H_k.transpose() + R_k).inverse(); // R 2*2
}

void kf::aiekf::setAdaptiveQ(MEASUREMENT meas_at_k)
{
    // Eigen::Vector
    // Q_k = QAdaptiveAlpha * Q_k + (1 - QAdaptiveAlpha) *



}

void kf::aiekf::setAdaptiveR(MEASUREMENT meas_at_k)
{

}

void kf::aiekf::setJacobianDynamic(Eigen::MatrixXd& Jacob)
{
    Jacob.resize(trackedSize, trackedSize);
    Jacob.setZero();

    Jacob.block<3,3>(0,0).setIdentity();
    Jacob.block<3,3>(3,3).setIdentity();
    Jacob.block<3,3>(6,6).setIdentity();

    Jacob.block<3,3>(0,6).setConstant(deltaT);
    Jacob.block<3,3>(6,3) = 
        -1.0 * 
        skewSymmetricMatrix(X_previous_posterori.X_SE3.rotationMatrix() * g);
}

void kf::aiekf::setJacobianCamera(Eigen::MatrixXd& Jacob, MEASUREMENT meas_at_k)
{
    Eigen::Matrix<double, 2, 6> JCamWRTPose;
    Eigen::Matrix<double, 2, 3> JCamWRTVelo;

    JCamWRTPose.setZero();
    JCamWRTVelo.setZero();

    for(int i = 0; meas_at_k.pts_3d_exists.size(); i++)
    {
        solveJacobianCamera(
            JCamWRTPose, 
            meas_at_k.pose_initial_SE3, 
            meas_at_k.pts_3d_exists[i]
        );

        JCamWRTPose = JCamWRTPose + JCamWRTPose;
    }

    Jacob.resize(JCamWRTPose.rows(), JCamWRTPose.cols() + JCamWRTVelo.cols());
    Jacob.block<2,6>(0,0) = JCamWRTPose;
    Jacob.block<2,3>(6,0) = JCamWRTVelo;    
}

Eigen::Matrix3d kf::aiekf::skewSymmetricMatrix(const Eigen::Vector3d w)
{
  Eigen::Matrix3d Omega;
  Omega << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return Omega;
}

#endif