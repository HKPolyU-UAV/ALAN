#ifndef PLAN_H
#define PLAN_H

#include "essential.h"

#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
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
            
            virtual void onInit()
            {
                ros::NodeHandle& nh = getNodeHandle();
                
                //load POT_extract config  
                cout<<"sup, we now at planner"<<endl;              
                

            }

            public:
                static void* PubMainLoop(void* tmp);

    };

    PLUGINLIB_EXPORT_CLASS(alan::PlannerNodelet, nodelet::Nodelet)
}

#endif