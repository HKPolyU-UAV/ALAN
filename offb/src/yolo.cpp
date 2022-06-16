#include "include/yolo.h"

static cv::Mat frame, res, gt;

static char* parampath = "/home/patty/alan_ws/src/alan/offb/src/include/yolo/uav-opt.param";
//                       /home/patty/alan_ws/src/alan/offb/src/include/yolo
static char* binpath = "/home/patty/alan_ws/src/alan/offb/src/include/yolo/uav-opt.bin";

int target_size = 608;

void callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth)
{
    cv_bridge::CvImageConstPtr depth_ptr;
    try
    {
        depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat image_dep = depth_ptr->image;
    
    try
    {
        frame = cv::imdecode(cv::Mat(rgbimage->data),1);
        res   = cv::imdecode(cv::Mat(rgbimage->data),1);
        gt    = cv::imdecode(cv::Mat(rgbimage->data),1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    // cout<<frame.size<<endl;
}

static bool foundornot;

void found_cb(const std_msgs::Bool::ConstPtr& msg)
{
    foundornot = msg->data;
    cout<<foundornot<<endl;;

}

int main(int argc, char** argv)
{
    static run_ncnn yolonet(parampath, binpath, target_size);

    cout<<"Object detection..."<<endl;

    ros::init(argc, argv, "yoy");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::CompressedImage> subimage(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<sensor_msgs::Image> subdepth(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subimage, subdepth);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Subscriber sub_found= nh.subscribe<std_msgs::Bool>
                              ("/obj_found", 1, found_cb);
    
    static double t1;
    static double t2;

    ros::Rate rate_manager(4);

    while(ros::ok())
    {
        
        // if(!frame.empty())
        // {
        //     cout<<"hi"<<endl;
        //     t1 = ros::Time::now().toSec();
        //     yolonet.detect_yolo(frame);
        //     t2 = ros::Time::now().toSec();
        //     cout << 1000 * (t2 - t1) << " ms" <<endl;
        //     cout << 1 / (t2 - t1) << " Hz" <<endl<<endl;;

            
        // }
        // cout<<foundornot<<endl;
        cout<<"hi"<<endl;
        ros::spinOnce();
        rate_manager.sleep();
        
    }
    ros::spin();
    return 0;
}
