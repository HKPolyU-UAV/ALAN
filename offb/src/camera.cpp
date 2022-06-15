#include "include/camera.h"

static cv::Mat frame, res, gt;

static cv::String weightpath ="/home/patty/alan_ws/src/alan/offb/src/include/yolo/uav_new.weights";
static cv::String cfgpath ="/home/patty/alan_ws/src/alan/offb/src/include/yolo/uav_new.cfg";
static cv::String classnamepath = "/home/patty/alan_ws/src/alan/offb/src/include/yolo/uav.names";

static run_yolo Yolonet(cfgpath, weightpath, classnamepath, float(0.1));

static int counter = 0;

veh_pose car_info, uav_info;

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

    Yolonet.getdepthdata(image_dep);

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

void car_position_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    car_info.x = pose->pose.position.x;
    car_info.y = pose->pose.position.y;
    car_info.z = pose->pose.position.z;
    car_info.ow = pose->pose.orientation.w;
    car_info.ox = pose->pose.orientation.x;
    car_info.oy = pose->pose.orientation.y;
    car_info.oz = pose->pose.orientation.z;
}

void uav_position_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uav_info.x = pose->pose.position.x;
    uav_info.y = pose->pose.position.y;
    uav_info.z = pose->pose.position.z;
    uav_info.ow = pose->pose.orientation.w;
    uav_info.ox = pose->pose.orientation.x;
    uav_info.oy = pose->pose.orientation.y;
    uav_info.oz = pose->pose.orientation.z;
}

void c2w(Eigen::Vector4d& camera_pt)
{
    double depth = camera_pt(2);
    double z = depth;
    double x = camera_pt
    Eigen::Matrix<double, 4, 4> c2b_transformation;
    c2b_transformation << 
        0.0342161,   -0.334618,  -0.941732, 0.567003,
        0.999403,     0.0159477,  0.0306448, -0.018069,
        0.00476414,  -0.942219,   0.334964, 0.0174849,
        0, 0, 0, 1;
    
    Eigen::Quaterniond q2r_matrix(car_info.ow, car_info.ox, car_info.oy, car_info.oz);

    Eigen::Matrix<double, 4, 4> b2w_transformation;
    
    b2w_transformation <<
    q2r_matrix.toRotationMatrix()(0,0), q2r_matrix.toRotationMatrix()(0,1), q2r_matrix.toRotationMatrix()(0,2), car_info.x,
    q2r_matrix.toRotationMatrix()(1,0), q2r_matrix.toRotationMatrix()(1,1), q2r_matrix.toRotationMatrix()(1,2), car_info.y,
    q2r_matrix.toRotationMatrix()(2,0), q2r_matrix.toRotationMatrix()(2,1), q2r_matrix.toRotationMatrix()(2,2), car_info.z,
    0, 0, 0, 1;

    camera_pt = b2w_transformation * c2b_transformation * camera_pt;
}

int main(int argc, char** argv)
{

    cout<<"Object detection..."<<endl;

    ros::init(argc, argv, "yolotiny");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::CompressedImage> subimage(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<sensor_msgs::Image> subdepth(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subimage, subdepth);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Subscriber sub_car_info = nh.subscribe<geometry_msgs::PoseStamped>
                                    ("/vrpn_client_node/gh034_car/pose", 1, car_position_callback);
    
    ros::Subscriber sub_uav_info = nh.subscribe<geometry_msgs::PoseStamped>
                                    ("/vrpn_client_node/gh034_nano2/pose", 1, uav_position_callback);

    ros::Publisher publish_obj_w = nh.advertise<geometry_msgs::Point>("/pose_w",1);
    ros::Publisher publish_obj_c = nh.advertise<geometry_msgs::PointStamped>("/pose_camera",1);
    ros::Publisher publish_found = nh.advertise<std_msgs::Bool>("/obj_found",1);
    ros::Publisher publish_obj_vel = nh.advertise<offb::obj>("/obj_v", 1);

    Eigen::MatrixXd F, P, Q, H, R;
    Eigen::Vector2i dim(6,3);

    cout<<dim[0]<<endl;
    // return 0;

    Eigen::Vector4d z_state;
    cv::Rect z_rect_detect;
    double z_depth;

    while(ros::ok())
    {
        if(!frame.empty())
        {
            Yolonet.rundarknet(frame);
            Yolonet.display(frame);
            // cout<<endl<<car_info.x<<endl<<endl;
            z_rect_detect = Yolonet.obj_vector[0].boundingbox;
            z_depth = Yolonet.obj_vector[0].depth;
            z_state = Eigen::Vector4d(z_rect_detect.x + z_rect_detect.width/2, 
                                      z_rect_detect.y + z_rect_detect.height/2,
                                      z_depth,
                                      1);

            z_state = c2w(z_state);
        }
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}