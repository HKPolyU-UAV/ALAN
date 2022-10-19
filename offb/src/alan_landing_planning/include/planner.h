

#include "essential.h"

#include "traj_gen.hpp"

#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>


#include <sophus/se3.hpp>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pthread.h>

#include "alan/AlanPlanner.h"
#include "alan/StateMachine.h"




namespace alan
{
    enum fsm
    {
        IDLE,
        READY,
        TAKEOFF,
        RENDEZVOUS,
        FOLLOW,
        LAND,
        REPLAN,
        SHUTDOWN,

    };

    class PlannerNodelet : public nodelet::Nodelet
    {
        public:
        private:
            double temp = 0;;

            fsm state = IDLE;
            pthread_t tid;


            //publisher
            ros::Publisher pub_traj_pos, pub_traj_vel;

            geometry_msgs::PoseStamped uav_traj_desi;

            //subscriber
            void uavStateCallback(const mavros_msgs::State::ConstPtr& msg);

            void uavOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);            

            void uavImuCallback(const sensor_msgs::Imu::ConstPtr& msg);

            void uavStateMachineCallback(const alan::StateMachine::ConstPtr& msg);

            ros::Subscriber sub_uav_state;
            ros::Subscriber sub_uav_odom;
            ros::Subscriber sub_uav_imu;

            Eigen::Isometry3d uavOdomPose;
            Eigen::Vector3d uavAcc;

            mavros_msgs::State uav_current_state;
            nav_msgs::Odometry uav_odom;
            sensor_msgs::Imu uav_imu;
            alan::StateMachine uav_fsm;

            bool uavOdomInitiated = false, uavAccInitiated = false;
                    

            //client
            ros::ServiceClient arming_client;
            ros::ServiceClient set_mode_client;

            mavros_msgs::SetMode offb_set_mode;
            mavros_msgs::CommandBool arm_cmd;


            
            //fsms
            void fsm_manager(fsm& state);

            void arm_uav(fsm& state);

            void ready_uav(fsm& state);

            void takeoff_uav(fsm& state);


            double last_request;

            //traj plan tools
            int n_order = 0;
            int m = 0;
            int d_order = 0;
            
            vector<double> trajseg_t;
            alan_traj::endpt start, end;
            vector<alan_traj::corridor> cube_list;

            double vel_avg = 0;

            void traj_setup(
                alan_traj::endpt start_pt, 
                alan_traj::endpt end_pt, 
                vector<alan_traj::corridor> cube_list
                );

            
            virtual void onInit()
            {
                ros::NodeHandle& nh = getNodeHandle();
            
                sub_uav_state = nh.subscribe<mavros_msgs::State>
                        ("/mavros/state", 1, &PlannerNodelet::uavStateCallback, this);
                
                sub_uav_odom = nh.subscribe<nav_msgs::Odometry>
                        ("/mavros/local_position/odom", 1, &PlannerNodelet::uavOdometryCallback, this);
                
                sub_uav_imu = nh.subscribe<sensor_msgs::Imu>
                        ("/mavros/imu/data", 1, &PlannerNodelet::uavImuCallback, this);
                
                set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("/mavros/set_mode");

                arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("mavros/cmd/arming");

                //load POT_extract config  
                cout<<"sup, we now at planner"<<endl;  

                pub_traj_pos = nh.advertise<geometry_msgs::PoseStamped>
                        ("/mavros/setpoint_position/local", 1);

                last_request = ros::Time::now().toSec();

                //create thread for publisher
                pthread_create(&tid, NULL, PlannerNodelet::PubMainLoop, (void*)this);


            }

            public:
                static void* PubMainLoop(void* tmp);

    };

    PLUGINLIB_EXPORT_CLASS(alan::PlannerNodelet, nodelet::Nodelet)
}
