#include "../include/tools/RosTopicConfigs.h"

RosTopicConfigs::RosTopicConfigs(ros::NodeHandle& _nh, string param_namespace)
: nh(_nh)
{
    nh.getParam(param_namespace + "/TOPICLIST", TOPICLIST);

    setupTOPICLIST();
}

RosTopicConfigs::~RosTopicConfigs()
{
}

string RosTopicConfigs::getTopicName(string designated_types)
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

void RosTopicConfigs::setupTOPICLIST()
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
        else if(topic_type == CAMERA_SUB_TOPIC_E)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == CAMERA_SUB_TOPIC_F)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == CAMERA_SUB_TOPIC_G)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == CAMERA_SUB_TOPIC_H)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == CAMERA_SUB_TOPIC_I)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == CAMERA_SUB_TOPIC_J)
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
        else if(topic_type == DEPTH_SUB_TOPIC_E)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == DEPTH_SUB_TOPIC_F)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == DEPTH_SUB_TOPIC_G)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == DEPTH_SUB_TOPIC_H)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == DEPTH_SUB_TOPIC_I)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == DEPTH_SUB_TOPIC_J)
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
        else if(topic_type == POSE_SUB_TOPIC_E)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_SUB_TOPIC_F)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_SUB_TOPIC_G)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_SUB_TOPIC_H)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_SUB_TOPIC_I)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_SUB_TOPIC_J)
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
        else if(topic_type == POSE_PUB_TOPIC_E)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_PUB_TOPIC_F)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_PUB_TOPIC_G)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_PUB_TOPIC_H)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_PUB_TOPIC_I)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == POSE_PUB_TOPIC_J)
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
        else if(topic_type == IMAGE_PUB_TOPIC_E)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == IMAGE_PUB_TOPIC_F)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == IMAGE_PUB_TOPIC_G)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == IMAGE_PUB_TOPIC_H)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == IMAGE_PUB_TOPIC_I)
        {
            config_setup_temp.topictypes = topic_type;
            config_setup_temp.topicnames = static_cast<string>(TOPICLIST[i][TOPICNAME]).c_str();
            config_setup_temp.SUB_PUB = TOPICLIST[i][SUBORPUB];
        }
        else if(topic_type == IMAGE_PUB_TOPIC_J)
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
