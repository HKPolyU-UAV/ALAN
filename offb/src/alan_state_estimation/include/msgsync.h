//sync all message including position, velocity, accleration
//of both uav and ugv

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
            ros::Publisher uav_pub_AlanPlannerMsg;
            ros::Publisher ugv_pub_AlanPlannerMsg;
            

            //subscribe
            void uav_msg_callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::Imu::ConstPtr& imu);
            void ugv_msg_callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::Imu::ConstPtr& imu);

            message_filters::Subscriber<nav_msgs::Odometry> uav_sub_odom;
            message_filters::Subscriber<sensor_msgs::Imu> uav_sub_imu;

            message_filters::Subscriber<nav_msgs::Odometry> ugv_sub_odom;
            message_filters::Subscriber<sensor_msgs::Imu> ugv_sub_imu;

            typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> uavMySyncPolicy;
            typedef message_filters::Synchronizer<uavMySyncPolicy> uavsync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<uavsync> uavsync_;     

            typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> ugvMySyncPolicy;
            typedef message_filters::Synchronizer<uavMySyncPolicy> ugvsync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<uavsync> ugvsync_;  

            //private variables 
            nav_msgs::Odometry uav_odom;
            sensor_msgs::Imu uav_imu;  

            nav_msgs::Odometry ugv_odom;
            sensor_msgs::Imu ugv_imu; 

            alan::AlanPlannerMsg uav_alan_msg;
            alan::AlanPlannerMsg ugv_alan_msg;


            bool uav_odom_initiated;
            bool uav_imu_initiated;

            bool ugv_odom_initiated;
            bool ugv_imu_initiated;

            Eigen::Isometry3d uavOdomPose;
            Eigen::Vector3d uav_acc_world;
            Eigen::Vector3d uav_acc_body;

            Eigen::Isometry3d ugvOdomPose;
            Eigen::Vector3d ugv_acc_world;
            Eigen::Vector3d ugv_acc_body;
                    

            virtual void onInit() 
            {
                ros::NodeHandle& nh = getNodeHandle();
                
                //initialize publisher

                //subscribe
                uav_sub_odom.subscribe(nh, "/uav/mavros/local_position/odom", 1);
                uav_sub_imu.subscribe(nh, "/uav/mavros/imu/data", 1);

                uavsync_.reset(new uavsync( uavMySyncPolicy(10), uav_sub_odom, uav_sub_imu));            
                uavsync_->registerCallback(boost::bind(&MsgSyncNodelet::uav_msg_callback, this, _1, _2));


                ugv_sub_odom.subscribe(nh, "/uav/mavros/local_position/odom", 1);
                ugv_sub_imu.subscribe(nh, "/uav/mavros/imu/data", 1);

                ugvsync_.reset(new ugvsync( ugvMySyncPolicy(10), ugv_sub_odom, ugv_sub_imu));            
                ugvsync_->registerCallback(boost::bind(&MsgSyncNodelet::ugv_msg_callback, this, _1, _2));


                uav_pub_AlanPlannerMsg = nh.advertise<alan::AlanPlannerMsg>
                                    ("/AlanPlannerMsg/uav/data", 1);

                ugv_pub_AlanPlannerMsg = nh.advertise<alan::AlanPlannerMsg>
                                    ("/AlanPlannerMsg/ugv/data", 1);

                ROS_INFO("IMU Nodelet Initiated...");
            }     

            public:
                static void* PubMainLoop(void* tmp);   

    };
    
    PLUGINLIB_EXPORT_CLASS(alan::MsgSyncNodelet, nodelet::Nodelet)

}