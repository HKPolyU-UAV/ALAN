#include "include/camera.h"

using namespace std;

double fx = 911.2578125,
       fy = 911.5093994140625,
       cx = 639.192626953125,
       cy = 361.2229919433594;

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
}

void c2w(Eigen::Vector4d& camera_pt)
{
    double depth = camera_pt(2);
    double z = camera_pt(2);
    double x = z * (camera_pt(0) - cx) / fx;
    double y = z * (camera_pt(1) - cy) / fy;
    Eigen::Vector4d cam(x,y,z,1);

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

    camera_pt = b2w_transformation * c2b_transformation * cam;
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
    ROS_INFO("uav_pose");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kf");
    ros::NodeHandle nh;

    ros::Subscriber sub_car_info = nh.subscribe<geometry_msgs::PoseStamped>
                                    ("/vrpn_client_node/gh034_car/pose", 1, car_position_callback);
    
    ros::Subscriber sub_uav_info = nh.subscribe<geometry_msgs::PoseStamped>
                                    ("/vrpn_client_node/gh034_nano2/pose", 1, uav_position_callback);

    Eigen::MatrixXd F, P, Q, H, R;
    KalmanFilter KF(F, P, Q, H, R, linear, 6, 3);



    // return 0;
    // cout<<F<<endl<<endl;
    // cout<<P<<endl<<endl;
    // cout<<Q<<endl<<endl;
    // cout<<H<<endl<<endl;
    // cout<<R<<endl<<endl;
    // return 0;

    Eigen::Vector4d z_state;
    cv::Rect z_rect_detect;
    double z_depth;
    // Yolonet.rundarknet(frame);
            // Yolonet.display(frame);
            // cout<<endl<<car_info.x<<endl<<endl;
            // z_rect_detect = Yolonet.obj_vector[0].boundingbox;
            // z_depth = Yolonet.obj_vector[0].depth;
            // z_state = Eigen::Vector4d(z_rect_detect.x + z_rect_detect.width/2, 
            //                           z_rect_detect.y + z_rect_detect.height/2,
            //                           z_depth,
            //                           1);

            c2w(z_state);
            // cout<<z_state<<endl;
            // cout<<
            // Eigen::Vector4d(
            //     z_state(0) - uav_info.x,
            //     z_state(1) - uav_info.y,
            //     z_state(2) - uav_info.z,
            //     1 - 1
            // ).norm()
            // <<endl;;

   
    ros::spin();
    return 0;
}




