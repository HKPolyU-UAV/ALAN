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
 * \file physique.cpp
 * \date 16/02/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes of a simple RViz robot visualization package of UAV, UGV, UUV
 */

#include "./include/essential.h"
#include "./include/RosTopicConfigs.h"
#include "./include/ugvpath.hpp"
#include <cmath>
#include <random>
#include <gazebo_msgs/ModelStates.h>
#include <type_traits>
#include <tf/tf.h>

using namespace std;

#define TURN 0
#define GOST 1

static Eigen::VectorXd current_traj_wp;

static nav_msgs::Odometry physique_car_odom;

static geometry_msgs::PoseStamped physique_car_pose;
static geometry_msgs::PoseStamped physique_car_pose_predicted;
static geometry_msgs::TwistStamped physique_car_twist;
static sensor_msgs::Imu physique_car_imu;
static geometry_msgs::PoseStamped virual_pose;

static vector<vector<string>> ugvpaths;

static int global_wp_counter = 0;
static int global_traj_counter = 0;

static double max_vel_lin = 0;
static double max_vel_ang = 0;
static int pub_freq = 0;
static std::string environs;

static bool static_ornot = false;

static double ang_vel;
static double radius;
static double center_x;
static double center_y;

static double starto_x;
static double starto_y;
static double endo_x;
static double endo_y;
static double straight_vel;
static bool straight_repeat;
static int straight_repeat_time;
static bool yaw_control;

static std::string GAZEBO_ = "GAZEBO";

static int fsm = TURN;

static bool cal_new_traj = false;

static vector<double> yaw_traj;
static vector<Eigen::Vector2d> lin_traj;

static bool notyetfinish = false;

static double PID_last_time;

Eigen::Vector3d q2rpy(Eigen::Quaterniond q) {
    tfScalar yaw, pitch, roll;
    tf::Quaternion q_tf;
    q_tf.setW(q.w());
    q_tf.setX(q.x());
    q_tf.setY(q.y());
    q_tf.setZ(q.z());

    tf::Matrix3x3 mat(q_tf);
    mat.getEulerYPR(yaw, pitch, roll);

    return Eigen::Vector3d(roll, pitch, yaw);
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

static Eigen::VectorXd physique_car_state(7); //x y z r p y
static Eigen::VectorXd physique_car_state_xyyaw(3);
static int indi = 0;
static int state_i = 0;

template<typename T>
void physique_car_state_callback_impl(const T& msg, std::true_type)
{
    gazebo_msgs::ModelStates state = *msg;

    if(indi == 0)
    {
        for(int i = 0; i < state.name.size(); i++)
        {
            if(state.name[i] == "scout/")
            {
                state_i = i;
                indi++;
            }
        }
    }

    physique_car_state(0) = state.pose[state_i].position.x;
    physique_car_state(1) = state.pose[state_i].position.y;
    physique_car_state(2) = state.pose[state_i].position.z;

    physique_car_state(3) = state.pose[state_i].orientation.w;
    physique_car_state(4) = state.pose[state_i].orientation.x;
    physique_car_state(5) = state.pose[state_i].orientation.y;
    physique_car_state(6) = state.pose[state_i].orientation.z;

    physique_car_state_xyyaw(0) = state.pose[state_i].position.x;
    physique_car_state_xyyaw(1) = state.pose[state_i].position.y;

    Eigen::Quaterniond q_(
        state.pose[state_i].orientation.w,
        state.pose[state_i].orientation.x,
        state.pose[state_i].orientation.y,
        state.pose[state_i].orientation.z
    );
    physique_car_state_xyyaw(2) =  atan2(q_.toRotationMatrix()(1,0), q_.toRotationMatrix()(0,0));
}

template<typename T>
void physique_car_state_callback_impl(const T& msg, std::false_type)
{
    geometry_msgs::PoseStamped state = *msg;

    physique_car_state(0) = state.pose.position.x;
    physique_car_state(1) = state.pose.position.y;
    physique_car_state(2) = state.pose.position.z;

    physique_car_state(3) = state.pose.orientation.w;
    physique_car_state(4) = state.pose.orientation.x;
    physique_car_state(5) = state.pose.orientation.y;
    physique_car_state(6) = state.pose.orientation.z;

    
    physique_car_state_xyyaw(0) = state.pose.position.x;
    physique_car_state_xyyaw(1) = state.pose.position.y;

    Eigen::Quaterniond q_(
        state.pose.orientation.w,
        state.pose.orientation.x,
        state.pose.orientation.y,
        state.pose.orientation.z
    );
    physique_car_state_xyyaw(2) =  atan2(q_.toRotationMatrix()(1,0), q_.toRotationMatrix()(0,0));
    
    // std::cout<<physique_car_state(2)<<std::endl;

    // std::cout<<physique_car_state_xyyaw<<std::endl;
    
    // std::cout<<"not gazebo"<<std::endl;
        
}

template <typename T>
void physique_car_state_callback(const T& msg)
{
    // std::cout<<std::is_same<T, gazebo_msgs::ModelStates::ConstPtr>()<<std::endl;
    physique_car_state_callback_impl(msg, std::is_same<T, gazebo_msgs::ModelStates::ConstPtr>());        
}

void physique_car_vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    Eigen::Vector3d vel(
        msg->twist.linear.x,
        msg->twist.linear.y,
        msg->twist.linear.z
    );

    // std::cout<<"current velocity..."<<vel.norm()<<std::endl;
}

void set_physique_car_pose(Eigen::VectorXd xyzrpy)
{
    physique_car_pose.pose.position.x = xyzrpy(0);
    physique_car_pose.pose.position.y = xyzrpy(1);
    physique_car_pose.pose.position.z = xyzrpy(2);
    
    physique_car_pose.pose.orientation.w = xyzrpy(3);
    physique_car_pose.pose.orientation.x = xyzrpy(4);
    physique_car_pose.pose.orientation.y = xyzrpy(5);
    physique_car_pose.pose.orientation.z = xyzrpy(6);

    physique_car_pose.header.stamp = ros::Time::now();

}

void set_physique_car_twist(Eigen::VectorXd xyzrpy)
{
    std::default_random_engine generator;
    std::normal_distribution<double> dist(0, 0.0001);

    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());

    physique_car_twist.twist.linear.x = 0 + dist(generator);
    physique_car_twist.twist.linear.y = 0 + dist(generator);
    physique_car_twist.twist.linear.z = 0 + dist(generator);
    
    physique_car_twist.twist.angular.x = 0 + dist(generator);
    physique_car_twist.twist.angular.y = 0 + dist(generator);
    physique_car_twist.twist.angular.z = 0 + dist(generator);
}

void set_physique_car_imu(Eigen::VectorXd xyzrpy)
{        
    std::default_random_engine generator;
    std::normal_distribution<double> dist(0, 0.001);

    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());

    physique_car_imu.orientation.w = 0.0;
    physique_car_imu.orientation.x = 0.0;
    physique_car_imu.orientation.y = 0.0;
    physique_car_imu.orientation.z = 0.0;

    physique_car_imu.angular_velocity.x = 0.0;
    physique_car_imu.angular_velocity.y = 0.0;
    physique_car_imu.angular_velocity.z = 0.0;

    physique_car_imu.linear_acceleration.x = 0.0 + dist(generator);
    physique_car_imu.linear_acceleration.y = 0.0 + dist(generator);
    physique_car_imu.linear_acceleration.z = 9.8 + dist(generator);

}

void set_physique_pose_twist_imu(Eigen::VectorXd xyzrpy)
{        
    set_physique_car_pose(xyzrpy);
    set_physique_car_twist(xyzrpy);
    set_physique_car_imu(xyzrpy);
}

void set_physique_pose_predicted(Eigen::Vector2d vel)
{
    // apply unicycle model
    // x1' = u_s * cos (theta)
    // x2' = u_s * sin (theta)
    // theta' = u_w
    // x' = [x1' x2' theta']^T 

    // x = x0 + x't

    double yaw = physique_car_state_xyyaw(2);

    double x1_v = vel(0) * cos(yaw);
    double x2_v = vel(0) * sin(yaw);
    double omega = vel(1);

    double delta_time = 1.0 / 50.0; // 0.02

    Eigen::Vector3d x_v(
        x1_v,
        x2_v,
        omega
    );

    Eigen::Vector3d t_temp;
    Eigen::Quaterniond q_temp;
    Eigen::Vector3d physique_car_state_xyyaw_predicted 
        = physique_car_state_xyyaw;

    for(int i = 0; i < 10; i++)
    {
        physique_car_state_xyyaw_predicted = 
        physique_car_state_xyyaw_predicted + 
        x_v * delta_time;

        x_v(0) = vel(0) * cos(physique_car_state_xyyaw_predicted(2));
        x_v(1) = vel(0) * sin(physique_car_state_xyyaw_predicted(2));
    }

    t_temp = Eigen::Vector3d(
        physique_car_state_xyyaw_predicted.x(),
        physique_car_state_xyyaw_predicted.y(),
        physique_car_state(2)
    );

    q_temp = rpy2q(
        Eigen::Vector3d(
            0,
            0,
            physique_car_state_xyyaw_predicted(2)
        )
    );

    

    physique_car_pose_predicted.header = physique_car_pose.header;
    physique_car_pose_predicted.header.frame_id = "world";
    physique_car_pose_predicted.pose.position.x = t_temp.x();
    physique_car_pose_predicted.pose.position.y = t_temp.y();
    physique_car_pose_predicted.pose.position.z = t_temp.z();

    physique_car_pose_predicted.pose.orientation.w = q_temp.w();
    physique_car_pose_predicted.pose.orientation.x = q_temp.x();
    physique_car_pose_predicted.pose.orientation.y = q_temp.y();
    physique_car_pose_predicted.pose.orientation.z = q_temp.z();

}

Eigen::Vector2d ugv_poistion_controller_PID(Eigen::Vector3d pose_XYyaw, Eigen::Vector2d setpoint)
{ // From VRPN XY position
    
    double err_dist = sqrt(pow((setpoint(0) - pose_XYyaw(0)), 2) +
                           pow((setpoint(1) - pose_XYyaw(1)), 2));
    
    Eigen::Vector2d diff_XY =  Eigen::Vector2d(setpoint(0) - pose_XYyaw(0), setpoint(1) - pose_XYyaw(1));
    
    double des_yaw = atan2(diff_XY(1), diff_XY(0));
    double err_yaw = des_yaw - pose_XYyaw(2);

    if (err_yaw >= M_PI){
        err_yaw -= 2 * M_PI;
    }

    if (err_yaw <= -M_PI){
        err_yaw += 2 * M_PI;
    }
    // cout << "err_yaw: " << err_yaw << endl;
    if (err_dist < 0.2){
        err_dist = 0;
        err_yaw = 0;
        // Mission_stage++;
    }            // Stop if the error is within 10 cm

    if(yaw_control)
    {
        if (err_yaw > M_PI * 0.05 || err_yaw < M_PI * -0.05){ 
            std::cout<<"turn before going straight"<<endl;
            err_dist = 0; 
        }   //Turn before going straight
    }
    
    
    Eigen::Vector2d error, last_error, u_p, u_i, u_d, output; // Dist Yaw Error

    double iteration_time = ros::Time::now().toSec() - PID_last_time;
    Eigen::Vector2d K_p(0.8, 1);
    Eigen::Vector2d K_i(0.2, 0);
    Eigen::Vector2d K_d(0  , 0);

    error =  Eigen::Vector2d(err_dist, err_yaw);

    last_error = error;

    Eigen::Vector2d integral = integral + (error * iteration_time);
    Eigen::Vector2d derivative = (error - last_error) / (iteration_time + 1e-10);

    for (int i = 0; i < 2; i++){                //i = err_dist,err_yaw
        u_p(i) = error(i) * K_p(i);           //P controller
        u_i(i) = integral(i) * K_i(i);        //I controller
        u_d(i) = derivative(i) * K_d(i);      //D controller
        // cout << "u_p[" << i << "]=" << u_p(i) << " u_i[" << i << "]=" << u_i(i) << " u_d[" << i << "]=" << u_d(i) << endl;
        output(i) = u_p(i) + u_i(i) + u_d(i);
    }
    
    if(output(0) >  max_vel_lin){ 
        output(0) = max_vel_lin;
    }  //Clamp the forward speed to MaxVelocity

    if(output(1) >  max_vel_ang){ 
        output(1) = max_vel_ang;
    }

    if(output(1) <  max_vel_ang * -1){ 
        output(1) = max_vel_ang * -1;
    }

    PID_last_time = ros::Time::now().toSec();

    return output;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "physique_car");
    ros::NodeHandle nh;

    nh.getParam("/physique_car/max_vel_lin", max_vel_lin);
    nh.getParam("/physique_car/max_vel_ang", max_vel_ang);
    nh.getParam("/physique_car/pub_freq", pub_freq);

    nh.getParam("/physique_car/static_ornot", static_ornot);

    
    nh.getParam("/physique_car/ang_vel", ang_vel);
    nh.getParam("/physique_car/radius", radius);
    nh.getParam("/physique_car/center_x", center_x);
    nh.getParam("/physique_car/center_y", center_y);

    nh.getParam("/physique_car/starto_x", starto_x);
    nh.getParam("/physique_car/starto_y", starto_y);
    nh.getParam("/physique_car/endo_x", endo_x);
    nh.getParam("/physique_car/endo_y", endo_y);
    nh.getParam("/physique_car/straight_vel", straight_vel);
    nh.getParam("/physique_car/straight_repeat", straight_repeat);
    nh.getParam("/physique_car/straight_repeat_time", straight_repeat_time);
    nh.getParam("/physique_car/yaw_control", yaw_control);




    nh.getParam("/physique_car/environs", environs);
    

    RosTopicConfigs configs(nh, "/physique_car");
 
    const char* temp1 = environs.c_str();
    const char* temp2 = GAZEBO_.c_str();

    // std::cout<<temp1<<std::endl;
    // std::cout<<temp2<<std::endl;
    // std::cout<<std::strcmp(temp1, temp2)<<std::endl;
    
    ros::Subscriber physique_car_state_sub;

    if(environs == GAZEBO_)
    {
        physique_car_state_sub = 
            nh.subscribe<gazebo_msgs::ModelStates>(
                "/gazebo/model_states", 
                1, 
                physique_car_state_callback<gazebo_msgs::ModelStates::ConstPtr>
            );

    }
    else
    {
        std::cout<<"hi...here...lala"<<std::endl;
        physique_car_state_sub = 
            nh.subscribe<geometry_msgs::PoseStamped>(
                "/vrpn_client_node/gh034_car/pose", 
                1, 
                physique_car_state_callback<geometry_msgs::PoseStamped::ConstPtr>
            );

    }

    ros::Subscriber physique_car_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
                        ("/vrpn_client_node/gh034_car/twist", 1, &physique_car_vel_callback);
    
    ros::Publisher physique_car_vel_pub = nh.advertise<geometry_msgs::Twist>
                        ("/cmd_vel", 1, true);

    ros::Publisher physique_car_posi_predicted_pub = nh.advertise<geometry_msgs::PoseStamped>
                        ("/alan_ugv/pose_predicted", 1, true);

    ros::Publisher physique_car_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
                        (configs.getTopicName(POSE_PUB_TOPIC_A), 1, true);

    ros::Publisher physique_car_twist_pub = nh.advertise<geometry_msgs::TwistStamped>
                        (configs.getTopicName(TWIST_PUB_TOPIC_A), 1, true);
    
    ros::Publisher physique_car_imu_pub = nh.advertise<sensor_msgs::Imu>
                        (configs.getTopicName(IMU_PUB_TOPIC_A), 1, true);


    ros::Rate physique_car_rate(pub_freq);

    Eigen::Vector2d center = Eigen::Vector2d(center_x,center_y);
    Eigen::Vector2d target_posi(2, 2);    

    ugv::ugvpath circle_traj(
        center,
        radius,
        pub_freq,
        ang_vel,
        100,
        true
    );

    // ugv::ugvpath block_traj(
    //     center,
    //     10.0,
    //     1.0,
    //     0.0,
    //     pub_freq,
    //     1.0,
    //     10,
    //     true
    // );

    ugv::ugvpath straight_traj(
        Eigen::Vector2d(starto_x, starto_y),
        Eigen::Vector2d(endo_x, endo_y),
        pub_freq,
        straight_vel,
        straight_repeat,
        straight_repeat_time
    );

    // ugv::ugvpath eight_traj(
    //     center,
    //     4.0,
    //     0.0,
    //     pub_freq,
    //     12.0,
    //     10,
    //     true
    // );
    

    cal_new_traj = true;
    current_traj_wp.resize(6);

    current_traj_wp(0) = 0.0;
    current_traj_wp(1) = 0.0;
    current_traj_wp(2) = 0.0;

    current_traj_wp(3) = 0.0;
    current_traj_wp(4) = 0.0;
    current_traj_wp(5) = 0.0;

    Eigen::Vector2d twist_final;
    geometry_msgs::Twist twist_pub_object;

    int traj_i = 0;
    

    while (ros::ok())    
    {         
        // set_current_traj_wp(current_traj_wp);
        set_physique_pose_twist_imu(physique_car_state);
        

        physique_car_pose_pub.publish(physique_car_pose);
        physique_car_twist_pub.publish(physique_car_twist);
        physique_car_imu_pub.publish(physique_car_imu);

        target_posi = straight_traj.get_traj_straight_wp(physique_car_state_xyyaw, traj_i);
        // target_posi = Eigen::Vector2d(0,0);
        std::cout<<target_posi<<std::endl<<std::endl<<std::endl;

        if(static_ornot)
        {
            twist_final.setZero();
        }
        else
        {
            twist_final = ugv_poistion_controller_PID(physique_car_state_xyyaw, target_posi);
            
            set_physique_pose_predicted(twist_final);
        }
            
        

        twist_pub_object.linear.x = twist_final(0);
        twist_pub_object.angular.z = twist_final(1);

        
        physique_car_vel_pub.publish(twist_pub_object);
        physique_car_posi_predicted_pub.publish(physique_car_pose_predicted);

    
                  
        ros::spinOnce();
        physique_car_rate.sleep();
        /* code */
    }

    return 0;    
}