/*
    A HPP file for
    ROS subscribe and publish configuration.
    Created on 05/12/2022
    (c) pattylo
    from the RCUAS of Hong Kong Polytechnic University
*/

/**
 * \file rosconfigs.hpp
 * \brief classes subscribers and publishers configuration
 */
#ifndef ROSCONFIGS_HPP
#define ROSCONFIGS_HPP

#include <ros/ros.h>
#include <iostream>
using namespace std;

#define TOPICTYPE "TOPICTYPE"
#define TOPICNAME "TOPICNAME"
#define SUBORPUB "SUBORPUB"
#define SUB 0
#define PUB 1

#define CAMERA_SUB_TOPIC_A "CAMERA_SUB_TOPIC_A"
#define CAMERA_SUB_TOPIC_B "CAMERA_SUB_TOPIC_B"
#define CAMERA_SUB_TOPIC_C "CAMERA_SUB_TOPIC_C"
#define CAMERA_SUB_TOPIC_D "CAMERA_SUB_TOPIC_D"

#define DEPTH_SUB_TOPIC_A "DEPTH_SUB_TOPIC_A"
#define DEPTH_SUB_TOPIC_B "DEPTH_SUB_TOPIC_B"
#define DEPTH_SUB_TOPIC_C "DEPTH_SUB_TOPIC_C"
#define DEPTH_SUB_TOPIC_D "DEPTH_SUB_TOPIC_D"

#define POSE_SUB_TOPIC_A "POSE_SUB_TOPIC_A"
#define POSE_SUB_TOPIC_B "POSE_SUB_TOPIC_B"
#define POSE_SUB_TOPIC_C "POSE_SUB_TOPIC_C"
#define POSE_SUB_TOPIC_D "POSE_SUB_TOPIC_D"

#define POSE_PUB_TOPIC_A "POSE_PUB_TOPIC_A"
#define POSE_PUB_TOPIC_B "POSE_PUB_TOPIC_B"
#define POSE_PUB_TOPIC_C "POSE_PUB_TOPIC_C"
#define POSE_PUB_TOPIC_D "POSE_PUB_TOPIC_D"

#define IMAGE_PUB_TOPIC_A "IMAGE_PUB_TOPIC_A"
#define IMAGE_PUB_TOPIC_B "IMAGE_PUB_TOPIC_B"
#define IMAGE_PUB_TOPIC_C "IMAGE_PUB_TOPIC_C"
#define IMAGE_PUB_TOPIC_D "IMAGE_PUB_TOPIC_D"

struct CONFIG_SETUP{
    string topictypes;
    string topicnames;
    int SUB_PUB;
};

class rosconfigs
{
private:
    ros::NodeHandle nh;
    XmlRpc::XmlRpcValue TOPICLIST;

    void setupTOPICLIST();
    vector<CONFIG_SETUP> object_ConfigLIST;
    
    
public:
    rosconfigs(ros::NodeHandle& _nh);
    ~rosconfigs();

    string getTopicName(string designated_name);
};

rosconfigs::rosconfigs(ros::NodeHandle& _nh)
: nh(_nh)
{
    nh.getParam("/alan_master/TOPICLIST", TOPICLIST);

    setupTOPICLIST();
}

rosconfigs::~rosconfigs()
{
}

string rosconfigs::getTopicName(string designated_types)
{
    string returnvalue;

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

void rosconfigs::setupTOPICLIST()
{
    CONFIG_SETUP config_setup_temp;
    string topic_type;

    for(int i = 0; i < TOPICLIST.size(); i++)
    {        
        // static_cast<std::string>(TOPICLIST[i][TOPICTYPE]).c_str();
        topic_type = static_cast<string>(TOPICLIST[i][TOPICTYPE]).c_str();
        static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();

        if(topic_type == CAMERA_SUB_TOPIC_A)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == CAMERA_SUB_TOPIC_B)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == CAMERA_SUB_TOPIC_C)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == CAMERA_SUB_TOPIC_D)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == DEPTH_SUB_TOPIC_A)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == DEPTH_SUB_TOPIC_B)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == DEPTH_SUB_TOPIC_C)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == DEPTH_SUB_TOPIC_D)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_SUB_TOPIC_A)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_SUB_TOPIC_B)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_SUB_TOPIC_C)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_SUB_TOPIC_D)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_PUB_TOPIC_A)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_PUB_TOPIC_B)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_PUB_TOPIC_C)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_PUB_TOPIC_D)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == IMAGE_PUB_TOPIC_A)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == IMAGE_PUB_TOPIC_B)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == IMAGE_PUB_TOPIC_C)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == IMAGE_PUB_TOPIC_D)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
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

#endif