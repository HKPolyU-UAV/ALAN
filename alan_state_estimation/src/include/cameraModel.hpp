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
 * \file cameraModel.hpp
 * \date 24/08/2023
 * \author pattylo
 * \copyright (c) AIRO-Lab, RCUAS of Hong Kong Polytechnic University
 * \brief utilities functions involving cameraModel
 */

#ifndef CAMERAMODEL_HPP
#define CAMERAMODEL_HPP

#include "tools/essential.h"
#include <sophus/se3.hpp>

namespace vision{
    class cameraModel
    {
    private:
        
    public:
        cameraModel();
        ~cameraModel();

        virtual Eigen::Vector2d reproject_3D_2D(Eigen::Vector3d P, Sophus::SE3d pose);
        virtual double get_reprojection_error(
            std::vector<Eigen::Vector3d> pts_3d, 
            std::vector<Eigen::Vector2d> pts_2d, 
            Sophus::SE3d pose,
            bool draw_reproject
        );
        virtual void camOptimize(
            Sophus::SE3d& pose, 
            std::vector<Eigen::Vector3d> pts_3d_exists, 
            std::vector<Eigen::Vector2d> pts_2d_detected,
            double& BA_error
        );
        virtual void solveJacobianCamera(
            Eigen::MatrixXd& Jacob, 
            Sophus::SE3d pose, 
            Eigen::Vector3d point_3d
        );

        inline virtual Eigen::Vector3d q2rpy(Eigen::Quaterniond q) final;
        inline virtual Eigen::Quaterniond rpy2q(Eigen::Vector3d rpy) final;
        inline virtual Eigen::Vector3d q_rotate_vector(
            Eigen::Quaterniond q, 
            Eigen::Vector3d v
        ) final;

        static Eigen::MatrixXd cameraMat;
        
    };
    
    cameraModel::cameraModel()
    {
    }
    
    cameraModel::~cameraModel()
    {
    }
    
}

Eigen::MatrixXd vision::cameraModel::cameraMat = Eigen::MatrixXd::Zero(3,3);

Eigen::Vector2d vision::cameraModel::reproject_3D_2D(Eigen::Vector3d P, Sophus::SE3d pose)
{
    Eigen::Vector3d result;

    Eigen::Matrix3d R = pose.rotationMatrix();
    Eigen::Vector3d t = pose.translation();

    result = cameraMat * (R * P + t); 

    Eigen::Vector2d result2d;

    result2d <<
        result(0)/result(2), 
        result(1)/result(2);
    
    return result2d;
}

double vision::cameraModel::get_reprojection_error(
            std::vector<Eigen::Vector3d> pts_3d, 
            std::vector<Eigen::Vector2d> pts_2d, 
            Sophus::SE3d pose,
            bool draw_reproject
)
{
    double e = 0;

    Eigen::Vector2d reproject, error;

    for(int i = 0; i < pts_3d.size(); i++)
    {
        reproject = reproject_3D_2D(pts_3d[i], pose);
        error = pts_2d[i] - reproject;
        e = e + error.norm();
    }

    return e;

}

void vision::cameraModel::camOptimize(
    Sophus::SE3d& pose, 
    std::vector<Eigen::Vector3d> pts_3d_exists, 
    std::vector<Eigen::Vector2d> pts_2d_detected,
    double& BA_error
)
{
    //execute Gaussian-Newton Method
    // cout<<"Bundle Adjustment Optimization"<<endl;

    const int MAX_ITERATION = 400;

    const double converge_threshold = 1e-6;

    const int points_no = pts_2d_detected.size();

    Eigen::MatrixXd J;
    J.resize(2,6);
    
    Eigen::Matrix<double, 6, 6> A; // R6*6
    Eigen::Matrix<double, 6, 1> b; // R6
    Eigen::Vector2d e; // R2
    Eigen::Matrix<double, 6, 1> dx;

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
            solveJacobianCamera(J, pose, pts_3d_exists[i]);
            J *= (-1);

            e = pts_2d_detected[i] - reproject_3D_2D(pts_3d_exists[i], pose) ; 
            
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
            /*
                pertubation defined in world frame (left Jacobian),
                left jacobians, 
                hence, add from the left
            */

        lastcost = cost;

        if(dx.norm() < converge_threshold)        
            break;
    }

    BA_error = lastcost;

    // cout<<"gone thru: "<<i<<" th, end optimize"<<endl<<endl;;;

}

void vision::cameraModel::solveJacobianCamera(Eigen::MatrixXd& Jacob, Sophus::SE3d pose, Eigen::Vector3d point_3d)
{
    
    Eigen::Matrix3d R = pose.rotationMatrix();
    Eigen::Vector3d t = pose.translation();
                // cameraMat
    double fx = cameraMat(0,0);
    double fy = cameraMat(1,1);

    Eigen::Vector3d point_in_camera = R * point_3d + t;
    
    double x_c = point_in_camera(0),
        y_c = point_in_camera(1),
        z_c = point_in_camera(2);

    Jacob.resize(2,6);

    //save entries to Jacob and return
    Jacob << 
        //first row
        fx / z_c, 
        0, 
        -fx * x_c / z_c / z_c, 
        -fx * x_c * y_c / z_c / z_c,
        fx + fx * x_c * x_c / z_c / z_c,
        -fx * y_c / z_c,

        //second row
        0,
        fy / z_c,
        -fy * y_c / z_c / z_c,
        -fy - fy * y_c * y_c / z_c / z_c,
        fy * x_c * y_c / z_c / z_c,
        fy * x_c / z_c;

}

Eigen::Vector3d vision::cameraModel::q2rpy(Eigen::Quaterniond q) 
{
    tfScalar yaw, pitch, roll;
    tf::Quaternion q_tf;
    q_tf.setW(q.w());
    q_tf.setX(q.x());
    q_tf.setY(q.y());
    q_tf.setZ(q.z());

    tf::Matrix3x3 mat(q_tf);
    mat.getEulerYPR(yaw, pitch, roll);

    return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Quaterniond vision::cameraModel::rpy2q(Eigen::Vector3d rpy)
{
    Eigen::AngleAxisd rollAngle(rpy(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rpy(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rpy(2), Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    return q;
}

Eigen::Vector3d vision::cameraModel::q_rotate_vector(Eigen::Quaterniond q, Eigen::Vector3d v)
{
    return q * v;
}



#endif