#ifndef PLAN_H
#define PLAN_H

#include "essential.h"

#include "traj_gen.hpp"

#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <sophus/se3.hpp>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pthread.h>


namespace alan
{

    class PlannerNodelet : public nodelet::Nodelet
    {
        public:
        private:

            //publisher
            ros::Publisher pub_traj_pos, pub_traj_vel;

            geometry_msgs::PoseStamped uav_traj_pose;
            
            virtual void onInit()
            {
                ros::NodeHandle& nh = getNodeHandle();
                
                //load POT_extract config  
                cout<<"sup, we now at planner"<<endl;  

                pub_traj_pos = nh.advertise<geometry_msgs::PoseStamped>("/desire_position", 1);

                

            }

            public:
                static void* PubMainLoop(void* tmp);

    };

    PLUGINLIB_EXPORT_CLASS(alan::PlannerNodelet, nodelet::Nodelet)
}

#endif