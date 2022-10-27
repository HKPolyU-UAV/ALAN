//sync all message including position, velocity, accleration
#include "tools/essential.h"

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "alan/AlanPlannerMsg.h"

namespace alan
{
    class MsgSyncNodelet : public nodelet::Nodelet
    {
        private:
            pthread_t tid;

            //publisher
            ros::Publisher pub_AlanPlannerMsg;
            ros::Publisher pubpose;
            

            //subscribe
            void msg_callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::Imu::ConstPtr& imu);
            
            message_filters::Subscriber<nav_msgs::Odometry> sub_odom;
            message_filters::Subscriber<sensor_msgs::Imu> sub_imu;

            typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> MySyncPolicy;
            typedef message_filters::Synchronizer<MySyncPolicy> sync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<sync> sync_;     

            //private variables 
            nav_msgs::Odometry uav_odom;
            sensor_msgs::Imu uav_imu;   

            alan::AlanPlannerMsg uav_alan_msg;

            bool uav_odom_initiated;
            bool uav_imu_initiated;

            Eigen::Isometry3d uavOdomPose;
            Eigen::Vector3d acc_world;
            Eigen::Vector3d acc_body;
                    

            virtual void onInit() 
            {
                ros::NodeHandle& nh = getNodeHandle();
                
                //initialize publisher

                //subscribe
                sub_odom.subscribe(nh, "/mavros/local_position/odom", 1);
                sub_imu.subscribe(nh, "/mavros/imu/data", 1);
                sync_.reset(new sync( MySyncPolicy(10), sub_odom, sub_imu));            
                sync_->registerCallback(boost::bind(&MsgSyncNodelet::msg_callback, this, _1, _2));



                pub_AlanPlannerMsg = nh.advertise<alan::AlanPlannerMsg>
                                    ("/AlanPlannerMsg/data", 1);

                ROS_INFO("IMU Nodelet Initiated...");
            }     

            public:
                static void* PubMainLoop(void* tmp);   

    };
    
    PLUGINLIB_EXPORT_CLASS(alan::MsgSyncNodelet, nodelet::Nodelet)

}