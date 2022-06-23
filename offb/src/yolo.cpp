// This ros node process 

#include "include/camera.h"

static cv::Mat frame, res, gt;

static cv::String weightpath ="/home/patty/alan_ws/src/alan/offb/src/include/yolo/uav_new.weights";
static cv::String cfgpath ="/home/patty/alan_ws/src/alan/offb/src/include/yolo/uav_new.cfg";
static cv::String classnamepath = "/home/patty/alan_ws/src/alan/offb/src/include/yolo/uav.names";

static run_yolo Yolonet(cfgpath, weightpath, classnamepath, float(0.5));

static int counter = 0;

veh_pose car_info, uav_info;

void car_position_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    car_info.x = pose->pose.position.x;
    car_info.y = pose->pose.position.y;
    car_info.z = pose->pose.position.z;
    car_info.ow = pose->pose.orientation.w;
    car_info.ox = pose->pose.orientation.x;
    car_info.oy = pose->pose.orientation.y;
    car_info.oz = pose->pose.orientation.z;
    // cout<<"hi"<<endl;
    // ROS_INFO("car pose");
}

void camera_callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth)
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

    Yolonet.getdepthdata(image_dep);

    try
    {
        frame = cv::imdecode(cv::Mat(rgbimage->data),1);
        // res   = cv::imdecode(cv::Mat(rgbimage->data),1);
        // gt    = cv::imdecode(cv::Mat(rgbimage->data),1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    Yolonet.rundarknet(frame);
    // Yolonet.display(frame);
    // cv::waitKey(20);

    // ros::Duration(5.0).sleep();
    // cout<<frame.size<<endl;
}

int main(int argc, char** argv)
{

    cout<<"Object detection..."<<endl;

    ros::init(argc, argv, "yolotiny");
    ros::NodeHandle nh;


   
    ros::Publisher publish_obj_w = nh.advertise<geometry_msgs::Point>("/pose_w",1);
    ros::Publisher publish_obj_c = nh.advertise<geometry_msgs::PointStamped>("/pose_camera",1);
    ros::Publisher publish_found = nh.advertise<std_msgs::Bool>("/obj_found",1);
    ros::Publisher publish_obj_vel = nh.advertise<offb::obj>("/obj_v", 1);

   

    ros::Rate rate_manager(40);
    ros::AsyncSpinner spinner_manager(0);

    ros::Subscriber sub_car_info = nh.subscribe<geometry_msgs::PoseStamped>
                                    ("/vrpn_client_node/gh034_car/pose", 1, car_position_callback);
    

    message_filters::Subscriber<sensor_msgs::CompressedImage> subimage(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<sensor_msgs::Image> subdepth(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subimage, subdepth);
    sync.registerCallback(boost::bind(&camera_callback, _1, _2));

    std_msgs::Bool obj;
    obj.data = true;
    
    spinner_manager.start();
    int counter = 0;

    while(ros::ok())
    {
        cout<<"why"<<endl;
        obj.data = !obj.data;
        if(counter%2 == 0)
            publish_found.publish(obj);
        counter++;
        ROS_INFO("processed 1 frame");

        if(!frame.empty())
            Yolonet.display(frame);

        ros::spinOnce();
        rate_manager.sleep();

    }
    ros::waitForShutdown();


    
    cout<<"end"<<endl;

    // while(ros::ok())
    // {
    //     spinner_manager.start();
    //     ros::spinOnce();
    // }
    return 0;
}