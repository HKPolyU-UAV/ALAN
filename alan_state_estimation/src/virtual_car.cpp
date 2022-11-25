/*
    virtual_car for simulation in RVIZ
    do not launch this in VICON!
*/

#include "./include/tools/essential.h"
#include <cmath>
#include <random>


static nav_msgs::Odometry virtual_car_odom;
static sensor_msgs::Imu virtual_car_imu;
static geometry_msgs::PoseStamped virtual_camera_pose;

Eigen::Vector3d q2rpy(Eigen::Quaterniond q) {
    return q.toRotationMatrix().eulerAngles(2,1,0);
};

Eigen::Quaterniond rpy2q(Eigen::Vector3d rpy){
    Eigen::AngleAxisd rollAngle(rpy(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rpy(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rpy(2), Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    return q;

};

Eigen::Vector3d q_rotate_vector(Eigen::Quaterniond q, Eigen::Vector3d v){
    return q * v;
}

void set_virtual_car_odom()
{        
    std::default_random_engine generator;
    std::normal_distribution<double> dist(0, 0.0005);

    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    

    virtual_car_odom.pose.pose.position.x = 0.0 + dist(generator);
    virtual_car_odom.pose.pose.position.y = 0.0 + dist(generator);
    virtual_car_odom.pose.pose.position.z = 0.0 + dist(generator);

    Eigen::Quaterniond attitude = rpy2q(
        Eigen::Vector3d(
            0.0 + dist(generator),
            -25.0 + dist(generator),
            0.0 + dist(generator))
    ) ;

    virtual_car_odom.pose.pose.orientation.w = attitude.w();    
    virtual_car_odom.pose.pose.orientation.x = attitude.x();
    virtual_car_odom.pose.pose.orientation.y = attitude.y();
    virtual_car_odom.pose.pose.orientation.z = attitude.z();

}

void set_virtual_car_imu()
{        
    std::default_random_engine generator;
    std::normal_distribution<double> dist(0, 0.001);

    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());

    virtual_car_imu.orientation.w = 0.0;
    virtual_car_imu.orientation.x = 0.0;
    virtual_car_imu.orientation.y = 0.0;
    virtual_car_imu.orientation.z = 0.0;

    virtual_car_imu.angular_velocity.x = 0.0;
    virtual_car_imu.angular_velocity.y = 0.0;
    virtual_car_imu.angular_velocity.z = 0.0;

    virtual_car_imu.linear_acceleration.x = 0.08603277722031888 + dist(generator);
    virtual_car_imu.linear_acceleration.y = -8.430162922124731 + dist(generator);
    virtual_car_imu.linear_acceleration.z = 3.920432808594682 + dist(generator);

}

void set_virtual_camera_pose()
{        
    std::default_random_engine generator;
    std::normal_distribution<double> dist(0, 0.001);

    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());

    virtual_camera_pose.pose.position.x = 0.0 + dist(generator);
    virtual_camera_pose.pose.position.y = 0.0 + dist(generator);
    virtual_camera_pose.pose.position.z = 0.0 + dist(generator);

    Eigen::Quaterniond attitude = rpy2q(
        Eigen::Vector3d(
            0.0 + dist(generator),
            -25.0 + dist(generator),
            0.0 + dist(generator))
    ) ;

    virtual_camera_pose.pose.orientation.w = attitude.w();
    virtual_camera_pose.pose.orientation.x = attitude.x();
    virtual_camera_pose.pose.orientation.y = attitude.y();
    virtual_camera_pose.pose.orientation.z = attitude.z();

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_car");
    ros::NodeHandle nh;

    ros::Publisher virtual_car_pose_pub = nh.advertise<nav_msgs::Odometry>
                        ("/ugv/mavros/local_position/odom", 1, true);
    
    ros::Publisher virtual_car_imu_pub = nh.advertise<sensor_msgs::Imu>
                        ("/camera/imu", 1, true);
    
    ros::Publisher virtual_car_cam_pub = nh.advertise<geometry_msgs::PoseStamped>
                        ("/imu_pose", 1, true);

    ros::Rate virtual_car_rate(200);

    while (ros::ok())    
    {
        set_virtual_car_odom();
        set_virtual_car_imu();
        set_virtual_camera_pose();

        virtual_car_pose_pub.publish(virtual_car_odom);
        // virtual_car_imu_pub.publish(virtual_car_imu);
        virtual_car_cam_pub.publish(virtual_camera_pose);

        virtual_car_rate.sleep();
        /* code */
    }

    return 0;    
}