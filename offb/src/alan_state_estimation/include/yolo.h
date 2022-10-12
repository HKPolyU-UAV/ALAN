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

#include <opencv2/dnn.hpp>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pthread.h>

namespace alan{

    typedef struct objectinfo
    {
        float confidence = 0;
        string classnameofdetection;
        cv::Rect boundingbox;
        cv::Mat frame;
        double depth;
    }objectinfo;

    class CnnNodelet : public nodelet::Nodelet
    {
        private:

            cv::Mat frame;
            bool intiated = false;

            message_filters::Subscriber<sensor_msgs::CompressedImage> subimage;
            message_filters::Subscriber<sensor_msgs::Image> subdepth;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
            typedef message_filters::Synchronizer<MySyncPolicy> sync;//(MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<sync> sync_;

            // cv::String cfg_file;
            // cv::String weights_file;
            // cv::String obj_file;
            cv::String weightpath; 
            cv::String cfgpath; 
            cv::String classnamepath;

            cv::Mat depthdata;

            float set_confidence;
            vector<std::string> classnames;

            void findboundingboxes(cv::Mat &frame);
            void findwhichboundingboxrocks(vector<cv::Mat> &netOut, cv::Mat &frame);
            void getclassname(vector<std::string> &classnames);
            chrono::time_point <chrono::steady_clock> total_start, total_end, dnn_start, dnn_end;
            float total_fps;
            objectinfo obj;
            cv::dnn::Net mydnn;
            
            void CnnNodeletInitiate(const cv::String cfgfile, const cv::String weightfile, const cv::String objfile, const float confidence);


            void rundarknet(cv::Mat &frame);
            void display(cv::Mat frame);
            void getdepthdata(cv::Mat depthdata);
            float appro_fps;
            vector<objectinfo> obj_vector;

            void camera_callback(
                const sensor_msgs::CompressedImageConstPtr & rgbimage,
                const sensor_msgs::ImageConstPtr & depth
            );
            
            virtual void onInit() 
            {
                ros::NodeHandle& nh = getNodeHandle();

                nh.getParam("/cnn/weightpath", weightpath);
                nh.getParam("/cnn/cfgpath", cfgpath); 
                nh.getParam("/cnn/classnamepath", classnamepath);
                CnnNodeletInitiate(cfgpath, weightpath, classnamepath, 0.1);

                // cout<<weightpath<<endl;
                // cout<<cfgpath<<endl;
                // cout<<classnamepath<<endl;

                //subscribe
                subimage.subscribe(nh, "/camera/color/image_raw/compressed", 1);
                subdepth.subscribe(nh, "/camera/aligned_depth_to_color/image_raw", 1);                
                sync_.reset(new sync( MySyncPolicy(10), subimage, subdepth));            
                sync_->registerCallback(boost::bind(&CnnNodelet::camera_callback, this, _1, _2));

                ROS_INFO("CNN Nodelet Initiated...");
            }

    };

   PLUGINLIB_EXPORT_CLASS(alan::CnnNodelet, nodelet::Nodelet)

}