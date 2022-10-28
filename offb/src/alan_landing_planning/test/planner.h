

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
            

            void uavStateMachineCallback(const alan::StateMachine::ConstPtr& msg);

            ros::Subscriber sub_uav_Alan

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
