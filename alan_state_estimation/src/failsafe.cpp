#include "./include/tools/essential.h"

static geometry_msgs::PoseStamped uav_final_pose;

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

void set_uav_final_pose()
{        
    std::default_random_engine generator;
    std::normal_distribution<double> dist(0, 0.001);

    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());

    uav_final_pose.pose.position.x = 0.0 + dist(generator);
    uav_final_pose.pose.position.y = 0.0 + dist(generator);
    uav_final_pose.pose.position.z = 0.0 + dist(generator);

    Eigen::Quaterniond attitude = rpy2q(
        Eigen::Vector3d(
            0.0 + dist(generator),
            -20.0  / 180.0 * M_PI + dist(generator),
            0.0 + dist(generator))
    ) ;
    
    uav_final_pose.pose.orientation.w = attitude.w();
    uav_final_pose.pose.orientation.x = attitude.x();
    uav_final_pose.pose.orientation.y = attitude.y();
    uav_final_pose.pose.orientation.z = attitude.z();

    uav_final_pose.header.frame_id = "world";
    // uav_final_pose.header.seq
    uav_final_pose.header.stamp.sec = ros::Time::now().toSec();
    uav_final_pose.header.stamp.nsec = ros::Time::now().toNSec();

}

void uav_msg_callback(const geometry_msgs::PoseStamped::ConstPtr &pose)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "failsafe");
    ros::NodeHandle nh;

    ros::Publisher uav_pose_final_pub = nh.advertise<geometry_msgs::PoseStamped>
                    ("/uav/alan_estimation/final_pose", 10, true);
    

    ros::Rate failsafeRate(60.0);

    while (ros::ok())
    {
        set_uav_final_pose();
        uav_pose_final_pub.publish(uav_final_pose);

        failsafeRate.sleep();
        /* code */
    }
    


    return 0;
}