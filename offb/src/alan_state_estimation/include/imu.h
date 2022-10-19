#include "tools/essential.h"

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

namespace alan
{
    class ImuNodelet : public nodelet::Nodelet
    {
        private:
            pthread_t tid;

            //publisher
            ros::Publisher nodelet_pub;
            ros::Publisher pubpose;
            

            //private variables
            cv::Mat frame;
            vector<Eigen::Vector3d> body_frame_pts;
            geometry_msgs::PoseStamped pose_estimated;
            bool add_noise = false;       
            int temp_i = 0;     


            //functions


            virtual void onInit() 
            {
                ros::NodeHandle& nh = getNodeHandle();

                //load camera intrinsics
                Eigen::Vector4d intrinsics_value;
                XmlRpc::XmlRpcValue intrinsics_list;
                nh.getParam("/alan_pose/cam_intrinsics_455", intrinsics_list);                
                                
                for(int i = 0; i < 4; i++)
                {
                    intrinsics_value[i] = intrinsics_list[i];
                }

  

                // cout<<cameraMat.inverse()<<endl;


                //load LED potisions in body frame
                XmlRpc::XmlRpcValue LED_list;
                nh.getParam("/alan_pose/LED_positions", LED_list); 
                for(int i = 0; i < LED_list.size(); i++)
                {
                    Eigen::Vector3d temp(LED_list[i]["x"], LED_list[i]["y"], LED_list[i]["z"]);
                    body_frame_pts.push_back(temp);
                }
                
                //initialize publisher
                nodelet_pub = nh.advertise<std_msgs::Bool>("/obj_found",1);
                pubpose = nh.advertise<geometry_msgs::PoseStamped>("/alan_pose/pose", 1);
                // test_pub = nh.advertise<std_msgs::Bool>("/ob_found",1);

                // pthread_create(&tid, NULL, ImuNodelet::PubMainLoop, (void*)this);

                ROS_INFO("IMU Nodelet Initiated...");
            }     

            public:
                static void* PubMainLoop(void* tmp);   

    };
    
    PLUGINLIB_EXPORT_CLASS(alan::ImuNodelet, nodelet::Nodelet)

}