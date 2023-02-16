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
 * \file data.cpp
 * \date 16/02/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief executable to record data
 */

#include "./include/essential.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include "alan_state_estimation/alan_log.h"

static std::string log_file;

void msg_callback(
    const alan_state_estimation::alan_log::ConstPtr& ledmsg,
    const alan_state_estimation::alan_log::ConstPtr& uavmsg
)
{

    // std::cout<<"hi"<<std::endl;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_log");
    ros::NodeHandle nh;

    std::string path;
    std::string filename;
    nh.getParam("/data/log_path", path);
    nh.getParam("/data/filename", filename);
    log_file = path + filename;

    std::cout<<log_file<<std::endl;

    message_filters::Subscriber<alan_state_estimation::alan_log> subled;
    message_filters::Subscriber<alan_state_estimation::alan_log> subuav;
    typedef message_filters::sync_policies::ExactTime<alan_state_estimation::alan_log, alan_state_estimation::alan_log> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> sync;//(MySyncPolicy(10), subimage, subdepth);
    boost::shared_ptr<sync> sync_;   

    subled.subscribe(nh, "/alan_state_estimation/led/led_log", 1);                
    subuav.subscribe(nh, "/alan_state_estimation/led/uav_log", 1);                
    sync_.reset(new sync( MySyncPolicy(10), subled, subuav));
    sync_->registerCallback(boost::bind(&msg_callback, _1, _2));                                

    ros::spin();
    return 0;

}