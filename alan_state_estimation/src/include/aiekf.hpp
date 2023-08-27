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
        Sophus::SE3d X_se3;
        Sophus::SE3d V_se3;
    } STATE;

    typedef struct MEASUREMENT
    {
        Sophus::Vector6d pose_initial_se3;
        std::vector<Eigen::Vector3d> pts_3d_exists;
        std::vector<Eigen::Vector2d> pts_2d_detected;
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
        
        Eigen::MatrixXd Q_init;
        Eigen::MatrixXd R_init;
        Eigen::MatrixXd Q_k;
        Eigen::MatrixXd R_k;

        MEASUREMENT Z_current_meas;      // Z @ k        

        void setPredict(); //set_X_current_dynamic_priori
        void setMeasurement();
        void doOptimize(MEASUREMENT meas_at_k);
        void setAdaptiveQ();
        void setAdaptiveR();
        void setPostOptimize();

        Eigen::VectorXd dfx();

        /* ================ utilities ================ */
        double deltaT = 0;

    public:
        aiekf(){};
        ~aiekf(){
            std::cout<<"EXIT AIEKF"<<std::endl;
        };

        void run_AIEKF(MEASUREMENT meas_at_k, double deltaT_);
        void initKF(
            Sophus::SE3d initial_pose,
            Eigen::MatrixXd Q_init_,
            Eigen::MatrixXd R_init_
        );
        void reinitKF(Sophus::SE3d reinitial_pose);

    };

}

void kf::aiekf::initKF(
    Sophus::SE3d initial_pose, 
    Eigen::MatrixXd Q_init_, 
    Eigen::MatrixXd R_init_
)
{
    X_previous_posterori.X_se3 = initial_pose;
    X_previous_posterori.V_se3 = Sophus::SE3d(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d::Zero()
    );
    trackedSize = 
          X_previous_posterori.X_se3.log().size()
        + X_previous_posterori.V_se3.log().head<3>().size();

    if(trackedSize != Q_init_.rows())
    {
        ROS_RED_STREAM("KF DIMENSION DOES NOT MATCH!!!");
        return;
    }
        
    Q_k = this->Q_init = Q_init_;
    R_k = this->R_init = R_init_;

}

void kf::aiekf::reinitKF(Sophus::SE3d reinitial_pose)
{
    X_previous_posterori.X_se3 = reinitial_pose;
    X_previous_posterori.V_se3 = Sophus::SE3d(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d::Zero()
    );

    Q_k = Q_init;
    R_k = R_init;
    // X_previous_posterori.size = X_previous_posterori.size;
}

void kf::aiekf::run_AIEKF(MEASUREMENT meas_at_k, double deltaT)
{
    setPredict();
    setMeasurement();
    setAdaptiveQ();
    doOptimize(meas_at_k);
    setAdaptiveR();
    setPostOptimize();
}

void kf::aiekf::setPredict()
{
    


}

void kf::aiekf::setMeasurement()
{

}

void kf::aiekf::setAdaptiveQ()
{

}

void kf::aiekf::setAdaptiveR()
{

}

void kf::aiekf::setPostOptimize()
{
    // update velocity

}

Eigen::VectorXd kf::aiekf::dfx()
{
    // x_dot = Ax here  
    // i.e., give delta x here, basically (in se(3))
    Eigen::VectorXd returnDfx; // first 6 element: se(3) of pose
                               // last 3 element: velocity 
    returnDfx.resize(trackedSize);

    

    return returnDfx;
    
}



/* ================ utilities function below ================ */

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

#endif