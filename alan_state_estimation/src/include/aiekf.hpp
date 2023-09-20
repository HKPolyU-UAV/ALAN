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

using namespace std;

namespace kf
{
    #define kfINITIATE 100
    #define kfREINITIATE 101
    #define kfNORMALKF 102

    typedef struct STATE
    {
        Sophus::SE3d X_SE3;
        Sophus::SE3d V_SE3;
        Eigen::MatrixXd PCov;
    } STATE;

    typedef struct MEASUREMENT
    {
        Sophus::SE3d pose_initial_SE3;
        Sophus::SE3d velo_initial_SE3;
        std::vector<Eigen::Vector2d> pts_2d_detected;
        std::vector<Eigen::Vector3d> pts_3d_exists;
        Eigen::Vector3d mean3d;
        Eigen::Vector2d mean2d;
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
        /* ================ states here ================ */
        STATE XpreviousPosterori;     // X @ k - 1
        STATE XprePreviousPosterori;  // X @ k - 2
        STATE XcurrentDynamicPriori;  // X @ k, priori
        STATE XoptimizePriori;        // X @ k, priori 
        MEASUREMENT ZcurrentMeas;   // Z @ k  

        /* ================ covariances here ================ */
        Eigen::MatrixXd F_k;
        Eigen::MatrixXd H_k;

        Eigen::MatrixXd K_k;

        Eigen::MatrixXd Q_k;
        Eigen::MatrixXd R_k;

        std::vector<Eigen::MatrixXd> info_matrix;
        
        /* ================ set Predict ================ */
        void setPredict(); // set_XcurrentDynamicPriori
        Eigen::VectorXd dfx();

        /* ================ set Measurement ================ */
        void setMeasurement(Sophus::SE3d initial_pose);
        void setMeasurement(
            std::vector<Eigen::Vector3d>& pts_on_body_frame_in_corres_order,
            std::vector<Eigen::Vector2d>& pts_detected_in_corres_order
        );
        void calculatePtsAverage();

        /* ================ set PreOptimize ================ */
        void setPreOptimize();

        /* ================ NLS Optimization ================ */
        void doOptimize();
        virtual void setGNBlocks(
            STATE X_var,
            Eigen::MatrixXd& JPJt, 
            Eigen::VectorXd& nJtPf,
            std::vector<Eigen::MatrixXd> Ps
        );
        void setDFJacobianDynamic(
            Eigen::MatrixXd& Jacob, 
            STATE X_var,
            bool optimize
        );
        void setDHJacobianCamera(
            Eigen::MatrixXd& Jacob, 
            STATE X_var, 
            Eigen::Vector3d pts_3d
        ); // for pose based on cameramodel
        void setDHJacobianCamera(
            Eigen::MatrixXd& Jacob
        ); // for velo based on inferred value

        Eigen::VectorXd getCameraPoseResidual(
            STATE X_var,
            Eigen::Vector2d pt_2d_detected,
            Eigen::Vector3d pt_3d_exist
        );
        Eigen::VectorXd getCameraVeloResidual(
            STATE X_var
        );
        Eigen::VectorXd getDynamicResidual(STATE pose_priori, STATE pose);
        double getCost(STATE X);

        /* ================ set PostOptimize ================ */
        void setPostOptimize();
        void setDHJacobianMeasurement(
            Eigen::MatrixXd& Jacob, 
            STATE X_var, 
            Eigen::Vector3d pts_3d
        ); // final jacobian for propagation
        void setKalmanGain();
        void setPosterioriCovariance();
        void setMisc();
        void setAdaptiveQ();
        void setAdaptiveR();        

        /* ================ utilities here ================ */
        int veloMeasureIndi = 0;
        double deltaT = 0;
        Eigen::Vector3d g = {0,0,-9.81};
        Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d w);

    public:
        aiekf(){};
        ~aiekf(){
            std::cout<<"EXIT AIEKF"<<std::endl;
        };
        STATE XcurrentPosterori;      // X @ k

        /* ================ main flow ================ */
        void initKF(Sophus::SE3d pose_initial_sophus);
        void reinitKF(Sophus::SE3d pose_reinitial_sophus);
        void run_AIEKF(
            double deltaT_,
            std::vector<Eigen::Vector3d>& pts_on_body_frame_in_corres_order,
            std::vector<Eigen::Vector2d>& pts_detected_in_corres_order
        );

        bool kf_initiated = false; 

        // for derived classes:        
        int kf_size;
        int kfZ_size;
        int kfZPose_size = 2;
        int kfZVelo_size = 3;
        Eigen::MatrixXd Q_init;
        Eigen::MatrixXd R_init;
        double QAdaptiveAlpha = 0;
        double RAdaptiveBeta = 0;   

        int MAX_ITERATION = 0;
        double CONVERGE_THRESHOLD = 0;
    };
}

/*=======main flow=======*/
void kf::aiekf::initKF(Sophus::SE3d pose_initial_sophus)
{
    // ROS_GREEN_STREAM("initKF");
    setMeasurement(pose_initial_sophus);

    XpreviousPosterori.X_SE3 = ZcurrentMeas.pose_initial_SE3;
    XpreviousPosterori.V_SE3 = Sophus::SE3d(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d::Zero()
    );

    XpreviousPosterori.PCov.resize(kf_size, kf_size);
    XpreviousPosterori.PCov.setIdentity();
    XpreviousPosterori.PCov = XpreviousPosterori.PCov * R_init(0,0);

    if(kf_size != Q_init.rows())
    {
        ROS_RED_STREAM("KF DIMENSION DOES NOT MATCH!!!");
        return;
    }
            
    Q_k = this->Q_init;
    R_k = this->R_init;
    std::cout<<"R_k"<<std::endl;
    std::cout<<R_k<<std::endl;

    // ros::shutdown();
}

void kf::aiekf::reinitKF(Sophus::SE3d pose_reinitial_sophus)
{
    setMeasurement(pose_reinitial_sophus);

    XpreviousPosterori.X_SE3 = ZcurrentMeas.pose_initial_SE3;
    XpreviousPosterori.V_SE3 = Sophus::SE3d(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d::Zero()
    );
    XpreviousPosterori.PCov = XpreviousPosterori.PCov * R_init(0,0);

    Q_k = Q_init;
    R_k = R_init;
}

void kf::aiekf::run_AIEKF(
    double deltaT_,
    std::vector<Eigen::Vector3d>& pts_on_body_frame_in_corres_order,
    std::vector<Eigen::Vector2d>& pts_detected_in_corres_order
)
{
    // ROS_GREEN_STREAM("runAIEKF");
    this->deltaT = deltaT_;

    setPredict(); 

    setMeasurement(pts_on_body_frame_in_corres_order,pts_detected_in_corres_order);
    
    setPreOptimize();
    
    doOptimize();
    
    setPostOptimize();
    
}

/*=======set Predict=======*/
void kf::aiekf::setPredict()
{
    // ROS_GREEN_STREAM("setPredict");

    Eigen::VectorXd dx = dfx() * deltaT; // first 6 element: se(3) of pose
                                   // last 3 element: velocity

    XcurrentDynamicPriori.X_SE3 = 
        Sophus::SE3d::exp(dx.head(6)) * XpreviousPosterori.X_SE3 ;

    // std::cout<<XcurrentDynamicPriori.X_SE3.matrix()<<std::endl;



    XcurrentDynamicPriori.V_SE3 = 
        XpreviousPosterori.V_SE3 * Sophus::SE3d(Eigen::Matrix3d::Identity(), dx.tail(3));

    setDFJacobianDynamic(
        F_k,
        XpreviousPosterori,
        false
    );

    XcurrentDynamicPriori.PCov = 
        F_k * XpreviousPosterori.PCov * F_k.transpose() + Q_k;
}

Eigen::VectorXd kf::aiekf::dfx()
{
    // x_dot = Ax here  
    // i.e., give x_dot here, basically (in se(3))
    Eigen::VectorXd returnDfx; // first 6 element: se(3) of pose
                               // last 3 element: velocity 
    returnDfx.resize(kf_size);
    
    XpreviousPosterori.V_SE3.log();
    
    returnDfx.head(Sophus::SE3d::DoF) = 
        Sophus::SE3d(
            Eigen::Matrix3d::Identity(),
            XpreviousPosterori.V_SE3.translation()
        ).log();
    
    returnDfx.tail<Sophus::SE3d::DoF/2>() = Sophus::SE3d(
        Eigen::Matrix3d::Identity(), 
        XpreviousPosterori.X_SE3.rotationMatrix() * g
    ).log().head(3);

    return returnDfx;
}

/*=======set Measurement=======*/
void kf::aiekf::setMeasurement(Sophus::SE3d initial_pose)
{
    // ROS_GREEN_STREAM("setMeasurementInitial");
    ZcurrentMeas.pose_initial_SE3 = initial_pose;
};

void kf::aiekf::setMeasurement(
    std::vector<Eigen::Vector3d>& pts_on_body_frame_in_corres_order,
    std::vector<Eigen::Vector2d>& pts_detected_in_corres_order
)
{
    // ROS_GREEN_STREAM("setMeasurementNormal");
    ZcurrentMeas.pts_3d_exists = pts_on_body_frame_in_corres_order;
    ZcurrentMeas.pts_2d_detected = pts_detected_in_corres_order;
    calculatePtsAverage();

    if(veloMeasureIndi == 0)
    {
        // std::cout<<"here veloMeasureIndi:"<<std::endl;
        // std::cout<<XcurrentDynamicPriori.X_SE3.translation()<<std::endl<<std::endl;
        // std::cout<<XpreviousPosterori.X_SE3.translation()<<std::endl<<std::endl;
        ZcurrentMeas.velo_initial_SE3 = Sophus::SE3d(
            Eigen::Matrix3d::Identity(),
            (XcurrentDynamicPriori.X_SE3 * XpreviousPosterori.X_SE3.inverse()).translation() 
            / deltaT
        ); 
        veloMeasureIndi++;
    }
    else
    {
        ZcurrentMeas.velo_initial_SE3 = Sophus::SE3d(
            Eigen::Matrix3d::Identity(),
            (XprePreviousPosterori.X_SE3 * XpreviousPosterori.X_SE3.inverse()).translation() 
            / deltaT
        );
        // use k-2 and k-1 pose difference
    }
}

void kf::aiekf::calculatePtsAverage()
{
    int n = ZcurrentMeas.pts_3d_exists.size();
    if(n != ZcurrentMeas.pts_2d_detected.size())
    {
        ROS_RED_STREAM("MEASUREMENT GOT ERROR; SIZE DIFFERENT!");

    }

    Eigen::Vector3d sum3d;
    sum3d.setZero();
    Eigen::Vector2d sum2d;
    sum2d.setZero();

    for(int i = 0; i < n; i++)
    {
        sum3d += ZcurrentMeas.pts_3d_exists[i];
        sum2d += ZcurrentMeas.pts_2d_detected[i];
    }

    ZcurrentMeas.mean3d = sum3d / n;
    ZcurrentMeas.mean2d = sum2d / n;
}

/*=======set PreOptimize=======*/
void kf::aiekf::setPreOptimize()
{
    // ROS_GREEN_STREAM("setPreOptimize");
    XoptimizePriori = XcurrentDynamicPriori;

    info_matrix.clear();
    info_matrix.emplace_back(R_k.inverse());
    info_matrix.emplace_back(XcurrentDynamicPriori.PCov.inverse());
}

/*=======NLS Optimization=======*/
void kf::aiekf::doOptimize()
{
    Eigen::MatrixXd JPJt; //R9*9
    JPJt.resize(kf_size,kf_size);
    Eigen::VectorXd nJtPf; // R9
    nJtPf.resize(kf_size);

    Eigen::Matrix<double, 9, 1> dx;
    Sophus::Vector6d dx_pose_se3;
    Sophus::Vector6d dx_velo_se3;

    STATE X_var = XoptimizePriori;

    int i = 0;
    double cost = 0, lastcost = INFINITY;

    // std::cout<<std::endl<<std::endl<<std::endl<<std::endl;

    ROS_BLUE_STREAM("OPTIMIZATION METRIC:...");

    std::cout<<"before cam residual:  "<<get_reprojection_error(
        ZcurrentMeas.pts_3d_exists,
        ZcurrentMeas.pts_2d_detected,
        X_var.X_SE3,
        false
    )<<std::endl;;
    std::cout<<"before dyn residual: "
        <<getDynamicResidual(
            XcurrentDynamicPriori,
            X_var
        ).norm()
        <<std::endl<<std::endl;

    /* ================================================================= */
    double t0 = ros::Time::now().toSec();
    for(i = 0; i < MAX_ITERATION; i++)
    {    
        setGNBlocks(
            X_var,
            JPJt, 
            nJtPf,
            info_matrix
        );

        // std::cout<<"linear system here:..."<<std::endl<<std::endl;;
        // std::cout<<JPJt<<std::endl<<std::endl;;
        // std::cout<<nJtPf<<std::endl<<std::endl;;

        //solve Adx = b
        dx = JPJt.ldlt().solve(nJtPf);

        if(isnan(dx(0,0)))
            break;

        dx_pose_se3 = dx.head(6);
        dx_velo_se3 << dx.tail(3), Eigen::Vector3d::Zero();

        X_var.X_SE3 =  Sophus::SE3d::exp(dx_pose_se3) * X_var.X_SE3 ;
        X_var.V_SE3 = Sophus::SE3d::exp(dx_velo_se3) * X_var.V_SE3;

        cost = getCost(X_var);
        
        // std::cout<<"cost: "<<std::endl;
        // std::cout<<cost<<std::endl;
        // std::cout<<"lastcost:"<<std::endl;
        // std::cout<<lastcost<<std::endl;
        // std::cout<<std::endl;

        if(cost > lastcost)
        {
            // ROS_RED_STREAM("OPTIMIZATION DIVERGE!!!");
            // break;
        }
        lastcost = cost;

        if(dx.norm() < CONVERGE_THRESHOLD)        
            break;
    }

    XcurrentPosterori = X_var;

    // double t1 = ros::Time::now().toSec();
    // std::cout<<"end opti!"<<std::endl;
    // /* ================================================================= */

    double e1 = get_reprojection_error(
        ZcurrentMeas.pts_3d_exists,
        ZcurrentMeas.pts_2d_detected,
        X_var.X_SE3,
        false
    );

    double e2 = getDynamicResidual(
        XcurrentDynamicPriori,
        X_var
    ).norm();

    std::cout<<"\nafter cam residual: "<<e1<<std::endl;
    std::cout<<"after dyn residual: "<<e2<<std::endl<<std::endl;

    std::cout<<"=================================="<<std::endl<<std::endl;

    std::cout<<"gone thru: "<<i<<" th, end optimize"<<std::endl<<std::endl;;;
    std::cout<<"dx.norm():\n "<<dx.norm()<<std::endl<<dx<<std::endl;
    std::cout<<"=================================="<<std::endl<<std::endl;

}

void kf::aiekf::setGNBlocks(
    STATE X_var,
    Eigen::MatrixXd& JPJt,
    Eigen::VectorXd& nJtPf,
    vector<Eigen::MatrixXd> Ps
)
{
    Eigen::MatrixXd JCamWRTPose;
    Eigen::MatrixXd JCamWRTVelo;
    Eigen::MatrixXd JDynWRTXNow;

    Eigen::VectorXd eCamPose; // R2
    Eigen::VectorXd eCamVelo; // R3
    Eigen::VectorXd eDyn; // R9

    JPJt.setZero();
    nJtPf.setZero();

    for(int i = 0; i < ZcurrentMeas.pts_3d_exists.size(); i++)
    // i < ZcurrentMeas.pts_3d_exists.size()
    {
        // camera pose linear system
        setDHJacobianCamera(
            JCamWRTPose,
            X_var,
            ZcurrentMeas.pts_3d_exists[i]
        );

        JCamWRTPose *= (-1);

        

        eCamPose = getCameraPoseResidual(
            X_var,
            ZcurrentMeas.pts_2d_detected[i],
            ZcurrentMeas.pts_3d_exists[i]
        );

        JPJt += JCamWRTPose.transpose() * Ps[0].block<2,2>(0,0) * JCamWRTPose;
        nJtPf += -JCamWRTPose.transpose() * Ps[0].block<2,2>(0,0) * eCamPose;
    }
    std::cout<<Ps[0].block<2,2>(0,0)<<std::endl;
    // std::cout<<"nJtPf"<<std::endl;
    // std::cout<<nJtPf<<std::endl;

    for(int i = 0; i < 1; i ++)
    {
        // camera velo linear system
        setDHJacobianCamera(
            JCamWRTVelo          
        );

        JCamWRTVelo *= (-1);

        eCamVelo = getCameraVeloResidual(
            X_var
        );
        
        JPJt += JCamWRTVelo.transpose() * Ps[0].block<3,3>(2,2) * JCamWRTVelo;
        nJtPf += -JCamWRTVelo.transpose() * Ps[0].block<3,3>(2,2) * eCamVelo;       
    }

    // std::cout<<"nJtPf"<<std::endl;
    // std::cout<<nJtPf<<std::endl;

    for(int i = 0; i < 1; i ++)
    {
        // dynamic pose + velo linear system
        setDFJacobianDynamic(
            JDynWRTXNow,
            X_var,
            true
        );

        JDynWRTXNow *= (-1);

        eDyn = getDynamicResidual(
            XcurrentDynamicPriori,
            X_var
        );
        
        JPJt += JDynWRTXNow.transpose() * Ps[1] * JDynWRTXNow;
        nJtPf += -JDynWRTXNow.transpose() * Ps[1] * eDyn;       
    }
    std::cout<<Ps[0].block<2,2>(0,0)<<std::endl;
    // std::cout<<"nJtPf"<<std::endl;
    // std::cout<<nJtPf<<std::endl;
}

void kf::aiekf::setDFJacobianDynamic(
    Eigen::MatrixXd& Jacob,
    STATE X_var,
    bool at_optimize
)
{
    Jacob.resize(kf_size, kf_size);
    if(at_optimize)
    {
        Jacob.setIdentity();
        return;
    }
        
    Jacob.setZero();

    Jacob.setIdentity();
    // Jacob.block<3,3>(0,0).setIdentity();
    // Jacob.block<3,3>(3,3).setIdentity();
    // Jacob.block<3,3>(6,6).setIdentity();

    Jacob.block<3,3>(0,6).setIdentity();
    Jacob.block<3,3>(0,6) *= deltaT;
    //  .setConstant(deltaT);
    Jacob.block<3,3>(6,3) = 
        -1.0 * 
        skewSymmetricMatrix(X_var.X_SE3.rotationMatrix() * g);
}

void kf::aiekf::setDHJacobianCamera(
    Eigen::MatrixXd& Jacob,
    STATE X_var,
    Eigen::Vector3d pts_3d_exists
)
{
    Jacob.resize(2,9);
    Jacob.setZero();

    Eigen::MatrixXd JCam_temp;

    solveJacobianCamera(
        JCam_temp, 
        X_var.X_SE3, 
        pts_3d_exists
    );

    Jacob.block<2,6>(0,0) = JCam_temp;
}

void kf::aiekf::setDHJacobianCamera(
    Eigen::MatrixXd& Jacob
)
{
    // set DH wrt image measurement
    Jacob.resize(kfZVelo_size, kf_size);
    Jacob.setZero();
    Jacob.block<3,3>(0,6).setIdentity(); 
}

Eigen::VectorXd kf::aiekf::getCameraPoseResidual(
    STATE X_var,
    Eigen::Vector2d pt_2d_detected,
    Eigen::Vector3d pt_3d_exist
)
{
    Eigen::VectorXd eCam;
    eCam.resize(kfZPose_size);
    
    eCam = pt_2d_detected - reproject_3D_2D(
        pt_3d_exist,
        X_var.X_SE3
    );

    return eCam;
}

Eigen::VectorXd kf::aiekf::getCameraVeloResidual(
    STATE X_var
)
{
    Eigen::VectorXd eCam;
    eCam.resize(kfZVelo_size);

    eCam = (ZcurrentMeas.velo_initial_SE3 * X_var.V_SE3.inverse()).log().head(3);
    
    return eCam;
}

Eigen::VectorXd kf::aiekf::getDynamicResidual(
    STATE priori_X,
    STATE X
)
{
    Eigen::VectorXd returnResidual;

    const int posSize = X.X_SE3.log().size() / 2;
    const int angSize = X.X_SE3.log().size() / 2;
    const int velSize  = X.V_SE3.log().head(3).size();
    
    returnResidual.resize(
        X.X_SE3.log().size() + 
        X.V_SE3.log().head<3>().size()
    );

    returnResidual.head(posSize + angSize) = (priori_X.X_SE3 * X.X_SE3.inverse()).log();
    returnResidual.tail(velSize) = (priori_X.V_SE3 * X.V_SE3.inverse()).log().head(velSize);

    return returnResidual;
}

double kf::aiekf::getCost(STATE X)
{
    double eTotal = 0;
    
    for(int i = 0; i < ZcurrentMeas.pts_3d_exists.size(); i++)
    {
        eTotal = eTotal + getCameraPoseResidual(
            X,
            ZcurrentMeas.pts_2d_detected[i],
            ZcurrentMeas.pts_3d_exists[i]
        ).norm();
    }

    eTotal += (getDynamicResidual(XcurrentDynamicPriori, X).norm() + getCameraVeloResidual(X).norm());

    return eTotal;
}

/*=======set PostOptimize=======*/
void kf::aiekf::setPostOptimize()
{
    setDHJacobianMeasurement(H_k, XcurrentPosterori, ZcurrentMeas.mean3d);
    setKalmanGain(); // R 9*5
    setPosterioriCovariance();
    setMisc();
    
    // setAdaptiveQ();
    // setAdaptiveR();
}

void kf::aiekf::setDHJacobianMeasurement(
    Eigen::MatrixXd& Jacob, 
    STATE X_var, 
    Eigen::Vector3d pts_3d
)
{
    Jacob.resize(kfZ_size, kf_size);
    Jacob.setZero();

    Eigen::MatrixXd Jacob_camera_pose; // R2*9
    setDHJacobianCamera(Jacob_camera_pose, X_var, pts_3d);

    Eigen::MatrixXd Jacob_camera_velo; // R3*9
    setDHJacobianCamera(Jacob_camera_velo);

    Jacob.block<2,9>(0,0) = Jacob_camera_pose;
    Jacob.block<3,9>(2,0) = Jacob_camera_velo;
}

void kf::aiekf::setKalmanGain()
{
    K_k = XcurrentDynamicPriori.PCov // R 9*9
        * H_k.transpose()            // R 9*5
        * (H_k * XcurrentDynamicPriori.PCov * H_k.transpose() + R_k).inverse(); // R 5*5
    
    // K_k -> R 9*5        
}

void kf::aiekf::setPosterioriCovariance()
{
    Eigen::MatrixXd E;
    E.resize(kf_size, kf_size);
    E.setIdentity();

    XcurrentPosterori.PCov = 
            (E - K_k * H_k) * XcurrentDynamicPriori.PCov * (E - K_k * H_k).transpose()
        +   K_k * R_k * K_k.transpose();
}

void kf::aiekf::setMisc()
{
    XprePreviousPosterori = XpreviousPosterori;
    XpreviousPosterori = XcurrentPosterori;

}

void kf::aiekf::setAdaptiveQ()
{
    Eigen::Vector2d innovation;
    // innovation = get_reprojection_error(
    //     ZcurrentMeas.pts_3d_exists,
    //     ZcurrentMeas.pts_2d_detected,
    //     ZcurrentMeas.pose_initial_SE3,
    //     false
    // );
    
    Q_k = QAdaptiveAlpha 
            * Q_k 
            + (1 - QAdaptiveAlpha) 
            * (K_k * innovation * innovation.transpose() * K_k.transpose());

}

void kf::aiekf::setAdaptiveR()
{
    Eigen::Vector2d residual;
    // residual = get_reprojection_error(
    //     ZcurrentMeas.pts_3d_exists,
    //     ZcurrentMeas.pts_2d_detected,
    //     XcurrentPosterori.X_SE3,
    //     false
    // );
}


/*=======utilities=======*/
Eigen::Matrix3d kf::aiekf::skewSymmetricMatrix(const Eigen::Vector3d w)
{
  Eigen::Matrix3d Omega;
  Omega << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return Omega;
}

#endif