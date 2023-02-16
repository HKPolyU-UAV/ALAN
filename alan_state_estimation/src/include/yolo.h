/*
    This file is part of ALan - the non-robocentric dynamic landing system for quadrotor

    ALan is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ALan is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ALan.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file yolo.h
 * \date 01/07/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief //legacy of AUTO (https://github.com/HKPolyU-UAV/AUTO) & previous 1st gen ALan (https://www.mdpi.com/1424-8220/22/1/404)
 */

#ifndef YOLO_H
#define YOLO_H

#include "tools/essential.h"

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

#include "tools/RosTopicConfigs.h"
// map definition for convinience
#define COLOR_SUB_TOPIC CAMERA_SUB_TOPIC_A
#define DEPTH_SUB_TOPIC CAMERA_SUB_TOPIC_B

namespace alan{

    typedef struct objectinfo
    {
        float confidence = 0;
        std::string classnameofdetection;
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
            std::vector<std::string> classnames;

            void findboundingboxes(cv::Mat &frame);
            void findwhichboundingboxrocks(std::vector<cv::Mat> &netOut, cv::Mat &frame);
            void getclassname(std::vector<std::string> &classnames);
            std::chrono::time_point <std::chrono::steady_clock> total_start, total_end, dnn_start, dnn_end;
            float total_fps;
            objectinfo obj;
            cv::dnn::Net mydnn;
            
            void CnnNodeletInitiate(const cv::String cfgfile, const cv::String weightfile, const cv::String objfile, const float confidence);


            void rundarknet(cv::Mat &frame);
            void display(cv::Mat frame);
            void getdepthdata(cv::Mat depthdata);
            float appro_fps;
            std::vector<objectinfo> obj_vector;

            void camera_callback(
                const sensor_msgs::CompressedImageConstPtr & rgbimage,
                const sensor_msgs::ImageConstPtr & depth
            );
            
            virtual void onInit() 
            {
                ros::NodeHandle& nh = getNodeHandle();

                RosTopicConfigs configs(nh, "/cnn");

                nh.getParam("/cnn/weightpath", weightpath);
                nh.getParam("/cnn/cfgpath", cfgpath); 
                nh.getParam("/cnn/classnamepath", classnamepath);
                CnnNodeletInitiate(cfgpath, weightpath, classnamepath, 0.1);

                std::cout<<weightpath<<std::endl;
                std::cout<<cfgpath<<std::endl;
                std::cout<<classnamepath<<std::endl;

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

#endif