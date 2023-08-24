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
#include <sophus/se3.hpp>

namespace kf
{
    typedef struct STATE
    {
        Sophus::Vector6d X_se3;
        Eigen::Vector3d vel;
        int size;
    } STATE;

    typedef struct MEASUREMENT
    {
        Sophus::Vector6d pose_initial_se3;
        std::vector<Eigen::Vector3d> pts_3d_exists;
        std::vector<Eigen::Vector2d> pts_2d_exists;
    } MEASUREMENT;

    typedef struct FINAL_RETURN
    {
        STATE X;
        Eigen::MatrixXd COV_MAT;

        double residual_error;
        double delta_now_then;
    }FINAL_RETURN;


    class aiekf
    {
        /*
            In this class, sensor fusion will be conducted,
            in which, an adaptive, iterative
            extended Kalman filter is exploited.
            In particular,
            the definied functions will not touch opencv
        */
    private:
        /* data */
        STATE X_current_posterori;       // X @ k
        STATE X_previous_posterori;      // X @ k - 1
        STATE X_current_dynamic_priori;  // X @ k, priori
        STATE X_current_camera_priori;   // X @ k, priori 

        MEASUREMENT Z_current_meas;      // Z @ k

        void predict(); //set_X_current_dynamic_priori

    public:
        aiekf(Sophus::Vector6d initial_pose);
        ~aiekf();

        void run_AIEKF(MEASUREMENT meas_at_k);

    };




}

kf::aiekf::aiekf(Sophus::Vector6d initial_pose)
{
    // this AIEKF is very speficially defined,
    // no prior setup is needed.
}

kf::aiekf::~aiekf()
{
    // EXIT AIEKF
}

void kf::aiekf::run_AIEKF(MEASUREMENT meas_at_k)
{
    predict();


}

void kf::aiekf::predict()
{

}



#endif