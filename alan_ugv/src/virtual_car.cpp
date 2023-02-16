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
 * \file virtual_car.cpp
 * \date 16/12/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief ros node for virtual car creation
 */

#include "./include/essential.h"
#include "./include/RosTopicConfigs.h"
#include <cmath>
#include <random>

using namespace std;

#define TURN 0
#define GOST 1

static Eigen::VectorXd current_traj_wp;

static nav_msgs::Odometry virtual_car_odom;

static geometry_msgs::PoseStamped virtual_car_pose;
static geometry_msgs::TwistStamped virtual_car_twist;
static sensor_msgs::Imu virtual_car_imu;
static geometry_msgs::PoseStamped virual_pose;

static vector<vector<string>> ugvpaths;

static int global_wp_counter = 0;
static int global_traj_counter = 0;

static double max_vel_lin = 0;
static double max_vel_ang = 0;
static int pub_freq = 0;

static int fsm = TURN;

static bool cal_new_traj = false;

static vector<double> yaw_traj;
static vector<Eigen::Vector2d> lin_traj;

static bool notyetfinish = false;

vector<vector<string>> parseCSV(string file_localtion)
{
    ifstream  data(file_localtion);
    string line;
    vector<vector<string> > parsedCsv;

    while(getline(data,line))
    {
        stringstream lineStream(line);
        string cell;
        vector<string> parsedRow;

        while(getline(lineStream,cell,','))
        {
            // cout<<1<<endl;
            // cout<<cell<<endl;
            parsedRow.push_back(cell);
        }

        parsedCsv.push_back(parsedRow);
    }

    return parsedCsv;
};

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

void set_virtual_car_odom(Eigen::VectorXd xyzrpy)
{        
    std::default_random_engine generator;
    std::normal_distribution<double> dist(0, 0.0001);

    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    

    virtual_car_odom.pose.pose.position.x = xyzrpy(0) + dist(generator);
    virtual_car_odom.pose.pose.position.y = xyzrpy(1) + dist(generator);
    virtual_car_odom.pose.pose.position.z = xyzrpy(2) + dist(generator);

    Eigen::Quaterniond attitude = rpy2q(
        Eigen::Vector3d(
            0.0 / 180.0 * M_PI + dist(generator),
            0.0 / 180.0 * M_PI + dist(generator),
            xyzrpy(5) / 180.0 * M_PI + dist(generator)
        )
    );

    virtual_car_odom.pose.pose.orientation.w = attitude.w();    
    virtual_car_odom.pose.pose.orientation.x = attitude.x();
    virtual_car_odom.pose.pose.orientation.y = attitude.y();
    virtual_car_odom.pose.pose.orientation.z = attitude.z();
}

void set_virtual_car_pose(Eigen::VectorXd xyzrpy)
{
    std::default_random_engine generator;
    std::normal_distribution<double> dist(0, 0.0001);

    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());


    virtual_car_pose.pose.position.x = xyzrpy(0) + dist(generator);
    virtual_car_pose.pose.position.y = xyzrpy(1) + dist(generator);
    virtual_car_pose.pose.position.z = xyzrpy(2) + dist(generator) + 0.259;

    Eigen::Quaterniond attitude = rpy2q(
        Eigen::Vector3d(
            xyzrpy(3)  / 180.0 * M_PI + dist(generator),
            xyzrpy(4)  / 180.0 * M_PI + dist(generator),
            xyzrpy(5)  / 180.0 * M_PI + dist(generator))
    ) ;
    
    virtual_car_pose.pose.orientation.w = attitude.w();
    virtual_car_pose.pose.orientation.x = attitude.x();
    virtual_car_pose.pose.orientation.y = attitude.y();
    virtual_car_pose.pose.orientation.z = attitude.z();

    virtual_car_pose.header.stamp = ros::Time::now();

}

void set_virtual_car_twist(Eigen::VectorXd xyzrpy)
{
    std::default_random_engine generator;
    std::normal_distribution<double> dist(0, 0.0001);

    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());

    virtual_car_twist.twist.linear.x = 0 + dist(generator);
    virtual_car_twist.twist.linear.y = 0 + dist(generator);
    virtual_car_twist.twist.linear.z = 0 + dist(generator);
    
    virtual_car_twist.twist.angular.x = 0 + dist(generator);
    virtual_car_twist.twist.angular.y = 0 + dist(generator);
    virtual_car_twist.twist.angular.z = 0 + dist(generator);
}

void set_virtual_car_imu(Eigen::VectorXd xyzrpy)
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

    virtual_car_imu.linear_acceleration.x = 0.0 + dist(generator);
    virtual_car_imu.linear_acceleration.y = 0.0 + dist(generator);
    virtual_car_imu.linear_acceleration.z = 9.8 + dist(generator);

}

void set_virtual_pose_twist_imu(Eigen::VectorXd xyzrpy)
{        
    set_virtual_car_pose(xyzrpy);
    set_virtual_car_twist(xyzrpy);
    set_virtual_car_imu(xyzrpy);
}

void calculate_heading_traj(double start_yaw, double end_yaw, vector<double>& yaw_traj)
{
    yaw_traj.clear();

    while (start_yaw > end_yaw)
    {
        end_yaw = end_yaw + 360;
    }
    

    double delta_yaw = end_yaw - start_yaw;

    int time_steps = abs(delta_yaw / max_vel_ang * pub_freq);

    for(int i = 0 ; i < time_steps; i++)
    {
        // cout<<start_yaw + delta_yaw / time_steps * (i + 1)<<endl;
        yaw_traj.emplace_back(start_yaw + delta_yaw / time_steps * (i + 1));
    }
        
}

void calculate_go_traj(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt, vector<Eigen::Vector2d>& lin_traj)
{
    lin_traj.clear();

    Eigen::Vector2d delta_pt = end_pt - start_pt;

    int time_steps = abs(delta_pt.norm() / max_vel_lin * pub_freq);

    for(int i = 0; i < time_steps; i++)
        lin_traj.emplace_back(start_pt + delta_pt / time_steps * (i + 1));

}

void set_current_traj_wp(Eigen::VectorXd& xyzrpy)
{
    // cout<<1<<endl;

    if(cal_new_traj)
    {
        cout<<"cal_new_traj here!"<<global_wp_counter<<endl;   
        if(global_wp_counter + 1 == ugvpaths.size())
        {
            notyetfinish = false;     
            cal_new_traj = false;
            ROS_INFO("END PATH...");
            // ros::shutdown();
        }

        if(notyetfinish)
        {
            //lin_traj
            Eigen::Vector2d pt_start, pt_end;
            pt_start = Eigen::Vector2d(
                stod(ugvpaths[global_wp_counter][0]),
                stod(ugvpaths[global_wp_counter][1])
                );
            
            pt_end = Eigen::Vector2d(
                stod(ugvpaths[global_wp_counter + 1][0]),
                stod(ugvpaths[global_wp_counter + 1][1])
                );
            calculate_go_traj(pt_start, pt_end, lin_traj);

            //yaw
            double yaw_start, yaw_end;
            Eigen::Vector2d vector_start_end = pt_end - pt_start;
            yaw_start = current_traj_wp(5);
            yaw_end = atan2(
                vector_start_end(1),
                vector_start_end(0)
            ) / M_PI * 180;
            // cout<<"yaw_start:"<<yaw_start<<endl;
            // cout<<"yaw_end:  "<<yaw_end<<endl;
            calculate_heading_traj(yaw_start, yaw_end, yaw_traj);

            fsm = TURN;
            global_wp_counter++;                                    
            cal_new_traj = false;

            // cout<<yaw_traj.size()<<endl;
            // cout<<lin_traj.size()<<endl;
        }                    
    }

    // cout<<2<<endl;
    if(notyetfinish)
    {
        if(fsm == TURN)
        {
            // cout<<"TURN"<<endl;
            xyzrpy(0) = current_traj_wp(0);
            xyzrpy(1) = current_traj_wp(1);
            xyzrpy(2) = 0;

            xyzrpy(3) = 0.0;//r
            xyzrpy(4) = 0.0;//p
            xyzrpy(5) = yaw_traj[global_traj_counter];//y

            global_traj_counter++;

            if(global_traj_counter == yaw_traj.size())
            {
                fsm = GOST;
                global_traj_counter = 0;
            }

        }
        else if(fsm == GOST)
        {
            // cout<<"GOST"<<endl;
            xyzrpy(0) = lin_traj[global_traj_counter](0);
            xyzrpy(1) = lin_traj[global_traj_counter](1);
            xyzrpy(2) = 0;

            xyzrpy(3) = 0.0;
            xyzrpy(4) = 0.0;//p
            xyzrpy(5) = current_traj_wp(5);

            global_traj_counter++;

            if(global_traj_counter == lin_traj.size())
            {
                fsm = TURN;
                global_traj_counter = 0;
                cal_new_traj = true;
            }

        }
        else
        {
            ROS_ERROR("PLEASE CHECK...FSM");
        }

    }
    else
    {
        xyzrpy(0) = current_traj_wp(0);
        xyzrpy(1) = current_traj_wp(1);
        xyzrpy(2) = 0;

        xyzrpy(3) = 0.0;//r
        xyzrpy(4) = 0.0;//p
        xyzrpy(5) = current_traj_wp(5);//y
    }        

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_car");
    ros::NodeHandle nh;

    RosTopicConfigs configs(nh, "/virtual_car");

    string csv_file_location;
    nh.getParam("/virtual_car/pathcsv", csv_file_location);

    nh.getParam("/virtual_car/max_vel_lin", max_vel_lin);
    nh.getParam("/virtual_car/max_vel_ang", max_vel_ang);
    nh.getParam("/virtual_car/pub_freq", pub_freq);

    ugvpaths = parseCSV(csv_file_location);    
    notyetfinish = true;

    // ros::Publisher virtual_car_odom_pub = nh.advertise<nav_msgs::Odometry>
    //                     ("/uav/alan_estimation/final_odom", 1, true);
    
    ros::Publisher virtual_car_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
                        (configs.getTopicName(POSE_PUB_TOPIC_A), 1, true);

    ros::Publisher virtual_car_twist_pub = nh.advertise<geometry_msgs::TwistStamped>
                        (configs.getTopicName(TWIST_PUB_TOPIC_A), 1, true);
    
    ros::Publisher virtual_car_imu_pub = nh.advertise<sensor_msgs::Imu>
                        (configs.getTopicName(IMU_PUB_TOPIC_A), 1, true);
    

    ros::Rate virtual_car_rate(pub_freq);
    

    cal_new_traj = true;
    current_traj_wp.resize(6);

    current_traj_wp(0) = 0.0;
    current_traj_wp(1) = 0.0;
    current_traj_wp(2) = 0.0;

    current_traj_wp(3) = 0.0;
    current_traj_wp(4) = 0.0;
    current_traj_wp(5) = 0.0;


    while (ros::ok())    
    {         
        set_current_traj_wp(current_traj_wp);
        set_virtual_pose_twist_imu(current_traj_wp);

        virtual_car_pose_pub.publish(virtual_car_pose);
        virtual_car_twist_pub.publish(virtual_car_twist);
        virtual_car_imu_pub.publish(virtual_car_imu);
                  
        
        virtual_car_rate.sleep();
        /* code */
    }

    return 0;    
}