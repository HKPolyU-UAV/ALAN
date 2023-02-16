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
 * \file RosTopicConfigs.cpp
 * \date 28/11/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classess to configure subscribe and publish topics
 */

#include "./include/RosTopicConfigs.h"

RosTopicConfigs::RosTopicConfigs(ros::NodeHandle& _nh, std::string param_namespace)
: nh(_nh)
{
    nh.getParam(param_namespace + "/TOPICLIST", TOPICLIST);

    setupTOPICLIST();
}

RosTopicConfigs::~RosTopicConfigs()
{
}

std::string RosTopicConfigs::getTopicName(std::string designated_types)
{
    std::string returnvalue;

    if(object_ConfigLIST.empty())
    {
        ROS_ERROR("PLEASE CHECK ROSLAUNCH .launch FILE...");
        ros::shutdown();
    }
    // cout<<"topic types: "<<designated_types<<endl;

    for(auto what : object_ConfigLIST)    
    {
        if(designated_types == what.topictypes)
            returnvalue = what.topicnames;
    }
        

    return returnvalue;
}

void RosTopicConfigs::setupTOPICLIST()
{
    CONFIG_SETUP config_setup_temp;
    std::string topic_type;

    for(int i = 0; i < TOPICLIST.size(); i++)
    {        
        // static_cast<std::std::string>(TOPICLIST[i][TOPICTYPE]).c_str();
        topic_type = static_cast<std::string>(TOPICLIST[i][TOPICTYPE]).c_str();
        static_cast<std::string>(TOPICLIST[i][TOPICNAME]).c_str();

        if(
            topic_type != CAMERA_SUB_TOPIC_A ||
            topic_type != CAMERA_SUB_TOPIC_B ||
            topic_type != CAMERA_SUB_TOPIC_C ||
            topic_type != CAMERA_SUB_TOPIC_D ||
            topic_type != CAMERA_SUB_TOPIC_E ||
            topic_type != CAMERA_SUB_TOPIC_F ||
            topic_type != CAMERA_SUB_TOPIC_G ||
            topic_type != CAMERA_SUB_TOPIC_H ||
            topic_type != CAMERA_SUB_TOPIC_I ||
            topic_type != CAMERA_SUB_TOPIC_J ||

            topic_type != IMAGE_PUB_TOPIC_A ||
            topic_type != IMAGE_PUB_TOPIC_B ||
            topic_type != IMAGE_PUB_TOPIC_C ||
            topic_type != IMAGE_PUB_TOPIC_D ||
            topic_type != IMAGE_PUB_TOPIC_E ||
            topic_type != IMAGE_PUB_TOPIC_F ||
            topic_type != IMAGE_PUB_TOPIC_G ||
            topic_type != IMAGE_PUB_TOPIC_H ||
            topic_type != IMAGE_PUB_TOPIC_I ||
            topic_type != IMAGE_PUB_TOPIC_J ||

            topic_type != POSE_SUB_TOPIC_A ||
            topic_type != POSE_SUB_TOPIC_B ||
            topic_type != POSE_SUB_TOPIC_C ||
            topic_type != POSE_SUB_TOPIC_D ||
            topic_type != POSE_SUB_TOPIC_E ||
            topic_type != POSE_SUB_TOPIC_F ||
            topic_type != POSE_SUB_TOPIC_G ||
            topic_type != POSE_SUB_TOPIC_H ||
            topic_type != POSE_SUB_TOPIC_I ||
            topic_type != POSE_SUB_TOPIC_J ||

            topic_type != POSE_PUB_TOPIC_A ||
            topic_type != POSE_PUB_TOPIC_B ||
            topic_type != POSE_PUB_TOPIC_C ||
            topic_type != POSE_PUB_TOPIC_D ||
            topic_type != POSE_PUB_TOPIC_E ||
            topic_type != POSE_PUB_TOPIC_F ||
            topic_type != POSE_PUB_TOPIC_G ||
            topic_type != POSE_PUB_TOPIC_H ||
            topic_type != POSE_PUB_TOPIC_I ||
            topic_type != POSE_PUB_TOPIC_J ||

            topic_type != TWIST_SUB_TOPIC_A ||
            topic_type != TWIST_SUB_TOPIC_B ||
            topic_type != TWIST_SUB_TOPIC_C ||
            topic_type != TWIST_SUB_TOPIC_D ||
            topic_type != TWIST_SUB_TOPIC_E ||
            topic_type != TWIST_SUB_TOPIC_F ||
            topic_type != TWIST_SUB_TOPIC_G ||
            topic_type != TWIST_SUB_TOPIC_H ||
            topic_type != TWIST_SUB_TOPIC_I ||
            topic_type != TWIST_SUB_TOPIC_J ||

            topic_type != TWIST_PUB_TOPIC_A ||
            topic_type != TWIST_PUB_TOPIC_B ||
            topic_type != TWIST_PUB_TOPIC_C ||
            topic_type != TWIST_PUB_TOPIC_D ||
            topic_type != TWIST_PUB_TOPIC_E ||
            topic_type != TWIST_PUB_TOPIC_F ||
            topic_type != TWIST_PUB_TOPIC_G ||
            topic_type != TWIST_PUB_TOPIC_H ||
            topic_type != TWIST_PUB_TOPIC_I ||
            topic_type != TWIST_PUB_TOPIC_J ||

            topic_type != IMU_SUB_TOPIC_A ||
            topic_type != IMU_SUB_TOPIC_B ||
            topic_type != IMU_SUB_TOPIC_C ||
            topic_type != IMU_SUB_TOPIC_D ||
            topic_type != IMU_SUB_TOPIC_E ||
            topic_type != IMU_SUB_TOPIC_F ||
            topic_type != IMU_SUB_TOPIC_G ||
            topic_type != IMU_SUB_TOPIC_H ||
            topic_type != IMU_SUB_TOPIC_I ||
            topic_type != IMU_SUB_TOPIC_J ||

            topic_type != IMU_PUB_TOPIC_A ||
            topic_type != IMU_PUB_TOPIC_B ||
            topic_type != IMU_PUB_TOPIC_C ||
            topic_type != IMU_PUB_TOPIC_D ||
            topic_type != IMU_PUB_TOPIC_E ||
            topic_type != IMU_PUB_TOPIC_F ||
            topic_type != IMU_PUB_TOPIC_G ||
            topic_type != IMU_PUB_TOPIC_H ||
            topic_type != IMU_PUB_TOPIC_I ||
            topic_type != IMU_PUB_TOPIC_J ||

            topic_type != ODOM_SUB_TOPIC_A ||
            topic_type != ODOM_SUB_TOPIC_B ||
            topic_type != ODOM_SUB_TOPIC_C ||
            topic_type != ODOM_SUB_TOPIC_D ||
            topic_type != ODOM_SUB_TOPIC_E ||
            topic_type != ODOM_SUB_TOPIC_F ||
            topic_type != ODOM_SUB_TOPIC_G ||
            topic_type != ODOM_SUB_TOPIC_H ||
            topic_type != ODOM_SUB_TOPIC_I ||
            topic_type != ODOM_SUB_TOPIC_J ||

            topic_type != ODOM_PUB_TOPIC_A ||
            topic_type != ODOM_PUB_TOPIC_B ||
            topic_type != ODOM_PUB_TOPIC_C ||
            topic_type != ODOM_PUB_TOPIC_D ||
            topic_type != ODOM_PUB_TOPIC_E ||
            topic_type != ODOM_PUB_TOPIC_F ||
            topic_type != ODOM_PUB_TOPIC_G ||
            topic_type != ODOM_PUB_TOPIC_H ||
            topic_type != ODOM_PUB_TOPIC_I ||
            topic_type != ODOM_PUB_TOPIC_J 
            
        )
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<std::string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];

        }
        else
        {
            ROS_ERROR("PLEASE CHECK ROSPARAM .yaml FILE...");
            ros::shutdown();
        }    

        object_ConfigLIST.emplace_back(config_setup_temp);
    }

}
