

//sync all message including position, velocity, accleration
//of both uav and ugv
#ifndef MSGSYNC_H
#define MSGSYNC_H

#include "tools/essential.h"

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "alan_landing_planning/AlanPlannerMsg.h"
// #include "alan_visualization/Polyhedron.h"
#include "alan_visualization/PolyhedronArray.h"

namespace alan
{
    class MsgSyncNodelet : public nodelet::Nodelet
    {
        private:
            pthread_t tid;

            //publisher
            ros::Publisher uav_pub_AlanPlannerMsg;
            ros::Publisher ugv_pub_AlanPlannerMsg;

            ros::Publisher alan_sfc_pub;            

            //subscribe
            void uav_msg_callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::Imu::ConstPtr& imu);
            void ugv_msg_callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::Imu::ConstPtr& imu);

            void cam_msg_callback(const geometry_msgs::PoseStamped::ConstPtr& imu);

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

            ros::Subscriber cam_pose_sub;

            //private variables 
            nav_msgs::Odometry uav_odom;
            sensor_msgs::Imu uav_imu;  

            nav_msgs::Odometry ugv_odom;
            sensor_msgs::Imu ugv_imu; 

            alan_landing_planning::AlanPlannerMsg uav_alan_msg;
            alan_landing_planning::AlanPlannerMsg ugv_alan_msg;


            bool uav_odom_initiated;
            bool uav_imu_initiated;

            bool ugv_odom_initiated;
            bool ugv_imu_initiated;

            Eigen::Isometry3d uavOdomPose;
            Eigen::Vector3d uav_pos_world;
            Eigen::Vector3d uav_acc_world;
            Eigen::Vector3d uav_acc_body;

            Eigen::Isometry3d ugvOdomPose;
            Eigen::Vector3d ugv_pos_world;
            Eigen::Vector3d ugv_acc_world;
            Eigen::Vector3d ugv_acc_body;

            geometry_msgs::PoseStamped cam_current_PoseMsg;
            Eigen::Isometry3d camPose;
            
            
            double FOV_H = 0, FOV_V = 0;//fov horizontal & vertical


            Eigen::Quaterniond q1, q2, q3, q4;
            Eigen::Vector3d cam_center_vector = Eigen::Vector3d(1,0,0),
                            cam_1axis_vector, 
                            cam_2axis_vector, 
                            cam_3axis_vector,
                            cam_4axis_vector;

            double temp_i = 0;

            // alan_visualization:
            // Polyhedron 
            alan_visualization::Polyhedron polyh_pub_object;

            //private functions
            Eigen::Vector3d q2rpy(Eigen::Quaterniond q);
            Eigen::Quaterniond rpy2q(Eigen::Vector3d rpy);

            Eigen::Vector3d q_rotate_vector(Eigen::Quaterniond q, Eigen::Vector3d v);

            alan_visualization::Tangent construct_tangent_plane(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d pt);
            alan_visualization::Tangent set_plane_bound(Eigen::Vector3d v, Eigen::Vector3d pt);

            void construct_sfc();
            Eigen::Vector3d get_outer_product(Eigen::Vector3d v1, Eigen::Vector3d v2);            
                    

            virtual void onInit() 
            {
                ros::NodeHandle& nh = getNodeHandle();    

                //param
                nh.getParam("/alan_master/cam_FOV_H", FOV_H);     
                nh.getParam("/alan_master/cam_FOV_V", FOV_V);                             

                FOV_H = FOV_H / 180 * M_PI;
                FOV_V = FOV_V / 180 * M_PI;

                // cout<<FOV_H<<endl;
                // cout<<FOV_V<<endl;

                Eigen::Vector3d rpy_temp;

                rpy_temp = Eigen::Vector3d(0, -FOV_V/2, -FOV_H/2);                
                q1 = rpy2q(rpy_temp);
                cam_1axis_vector = q_rotate_vector(q1, cam_center_vector);

                rpy_temp = Eigen::Vector3d(0, -FOV_V/2, FOV_H/2);                
                q2 = rpy2q(rpy_temp);
                cam_2axis_vector = q_rotate_vector(q2, cam_center_vector);

                rpy_temp = Eigen::Vector3d(0, FOV_V/2, FOV_H/2);                
                q3 = rpy2q(rpy_temp); 
                cam_3axis_vector = q_rotate_vector(q3, cam_center_vector);

                rpy_temp = Eigen::Vector3d(0, FOV_V/2, -FOV_H/2);                
                q4 = rpy2q(rpy_temp);
                // cout<<endl<<q4.toRotationMatrix() * Eigen::Vector3d(1,0,0)<<endl<<endl;
                cam_4axis_vector = q_rotate_vector(q4, cam_center_vector);

                // cout<<"\nhere are the camera axises..."<<endl;
                // cout<<cam_center_vector<<endl<<endl;
                // cout<<cam_1axis_vector<<endl<<endl;
                // cout<<cam_2axis_vector<<endl<<endl;
                // cout<<cam_3axis_vector<<endl<<endl;
                // cout<<cam_4axis_vector<<endl<<endl<<endl;




                //subscriber
                uav_sub_odom.subscribe(nh, "/uav/mavros/local_position/odom", 1);
                uav_sub_imu.subscribe(nh, "/uav/mavros/imu/data", 1);

                uavsync_.reset(new uavsync( uavMySyncPolicy(10), uav_sub_odom, uav_sub_imu));            
                uavsync_->registerCallback(boost::bind(&MsgSyncNodelet::uav_msg_callback, this, _1, _2));


                ugv_sub_odom.subscribe(nh, "/uav/mavros/local_position/odom", 1);
                ugv_sub_imu.subscribe(nh, "/uav/mavros/imu/data", 1);

                ugvsync_.reset(new ugvsync( ugvMySyncPolicy(10), ugv_sub_odom, ugv_sub_imu));            
                ugvsync_->registerCallback(boost::bind(&MsgSyncNodelet::ugv_msg_callback, this, _1, _2));                

                cam_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                    ("/imu_pose", 1, &MsgSyncNodelet::cam_msg_callback, this);
                

                //publisher
                uav_pub_AlanPlannerMsg = nh.advertise<alan_landing_planning::AlanPlannerMsg>
                                    ("/AlanPlannerMsg/uav/data", 1);

                ugv_pub_AlanPlannerMsg = nh.advertise<alan_landing_planning::AlanPlannerMsg>
                                    ("/AlanPlannerMsg/ugv/data", 1);

                alan_sfc_pub = nh.advertise<alan_visualization::Polyhedron>
                                    ("/alan/sfc/total_bound", 1);
                                    
                
                ROS_INFO("MSGSYNC Nodelet Initiated...");
            }     

            public:
                static void* PubMainLoop(void* tmp);   

    };
    
    PLUGINLIB_EXPORT_CLASS(alan::MsgSyncNodelet, nodelet::Nodelet)

}

#endif