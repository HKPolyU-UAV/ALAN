#include <ros/ros.h>
#include <numeric>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <deque>
#include "utils/common.hpp"
#include "utils/ugv_mission.hpp"
#include "utils/kinetic_math.hpp"
#define PI 3.14159265

/* FSM */
int    Mission_state = 0;
int    Mission_stage = 0;


int    Current_Mission_stage = 0;
Vec4   Current_stage_mission;
double PID_duration;
double PID_InitTime;
double MaxTurnrate = 0.45;      // radius per sec
double MaxVelocity = 0.6;    // meters per sec
/* System */
geometry_msgs::Twist UGV_twist_pub;
geometry_msgs::PoseStamped UGV_pose_vicon,UGV_pose_desire;
Vec3 pose_XYyaw;
Vec2 DesUGVpose;
bool System_init = true;
bool External_pos_setpoint = false;
bool FSM_mission = true;
int coutcounter;
double PID_last_time;

bool getUGVpose = false;

void UGVPose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    getUGVpose = true;
    // cout<<"lala"<<endl;
    UGV_pose_vicon.pose.position.x = pose->pose.position.x;
    UGV_pose_vicon.pose.position.y = pose->pose.position.y;
    UGV_pose_vicon.pose.position.z = pose->pose.position.z;
    UGV_pose_vicon.pose.orientation.w = pose->pose.orientation.w;
    UGV_pose_vicon.pose.orientation.x = pose->pose.orientation.x;
    UGV_pose_vicon.pose.orientation.y = pose->pose.orientation.y;
    UGV_pose_vicon.pose.orientation.z = pose->pose.orientation.z;
    Quaterniond localq(UGV_pose_vicon.pose.orientation.w,
                       UGV_pose_vicon.pose.orientation.x,
                       UGV_pose_vicon.pose.orientation.y,
                       UGV_pose_vicon.pose.orientation.z);
    Vec3 localrpy = Q2rpy(localq);
    pose_XYyaw = Vec3(UGV_pose_vicon.pose.position.x,UGV_pose_vicon.pose.position.y,localrpy[2]);
}
void UGVdesPose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    UGV_pose_desire.pose.position.x = pose->pose.position.x;
    UGV_pose_desire.pose.position.y = pose->pose.position.y;
}
void ugv_twist_pub(Vec2 VA){
    UGV_twist_pub.linear.x  = VA[0];
    UGV_twist_pub.angular.z = VA[1];
}
Vec2 ugv_poistion_controller_PID(Vec3 pose_XYyaw, Vec2 setpoint){ // From VRPN XY position
    // cout << "pose_XYyaw:  " << pose_XYyaw[0] << " " << pose_XYyaw[1] << " " << pose_XYyaw[2] << endl;
    // cout << "setpoint: " << setpoint[0] << " " << setpoint[1] << endl;
    double err_dist = sqrt(pow((setpoint[0]-pose_XYyaw[0]),2)+
                           pow((setpoint[1]-pose_XYyaw[1]),2));
    // cout << "err_dist: " << err_dist << endl;
    Vec2 diff_XY = Vec2(setpoint[0]-pose_XYyaw[0],setpoint[1]-pose_XYyaw[1]);
    double des_yaw = atan2(diff_XY[1],diff_XY[0]);
    // cout << "car_yaw: " << pose_XYyaw[2] << endl;
    // cout << "des_yaw: " << des_yaw << endl;
    double err_yaw = des_yaw-pose_XYyaw[2];
    // cout << "err_yaw: " << err_yaw << endl;
    if (err_yaw>=PI){    err_yaw-=2*PI;}
    if (err_yaw<=-PI){   err_yaw+=2*PI;}
    // cout << "err_dist: " << err_dist << endl;

    if (err_dist<0.2){err_dist = 0;err_yaw = 0;
        // cout<<"mission_state++"<<endl;
    Mission_stage++;}          
    
      // Stop if the error is within 10 cm
    if (err_yaw>PI*0.1||err_yaw<PI*-0.1){ err_dist = 0; }   //Turn before going straight
    Vec2 error,last_error,u_p,u_i,u_d,output; // Dist Yaw Error
    double iteration_time = ros::Time::now().toSec() - PID_last_time;
    Vec2 K_p(0.8,1);
    Vec2 K_i(0.2  ,0);
    Vec2 K_d(0  ,0);
    error = Vec2(err_dist,err_yaw);
    last_error = error;
    Vec2 integral = integral+(error*iteration_time);
    Vec2 derivative = (error - last_error)/(iteration_time + 1e-10);
    for (int i=0; i<2; i++){                //i = err_dist,err_yaw
        u_p[i] = error[i]*K_p[i];           //P controller
        u_i[i] = integral[i]*K_i[i];        //I controller
        u_d[i] = derivative[i]*K_d[i];      //D controller
        // cout << "u_p[" << i << "]=" << u_p[i] << " u_i[" << i << "]=" << u_i[i] << " u_d[" << i << "]=" << u_d[i] << endl;
        output[i] = u_p[i]+u_i[i]+u_d[i];
    }
    
    if(output[0] >  MaxVelocity){ output[0]= MaxVelocity;}  //Clamp the forward speed to MaxVelocity
    if(output[1] >  MaxTurnrate){ output[1] = MaxTurnrate;}
    if(output[1] <  MaxTurnrate*-1){ output[1] = MaxTurnrate*-1;}
    // cout << "iteration_time: " << iteration_time << endl;
    // cout << "output____ v: " << output[0] << " av: " << output[1] << endl;
    PID_last_time = ros::Time::now().toSec();
    return(output);
}
void ugv_pub(){
    ugv_twist_pub(ugv_poistion_controller_PID(pose_XYyaw,DesUGVpose));
    if (PID_InitTime + PID_duration < ros::Time::now().toSec()){
        // cout<<PID_InitTime + PID_duration - ros::Time::now().toSec()<<endl;
            Mission_stage++;
            ugv_twist_pub(Vec2(0,0));
        }
}
void Finite_state_machine(){ 
    // cout<<"mission_stage here..."<<Mission_stage<<endl;
    //     cout<<"current mission_stage here..."<<Current_Mission_stage<<endl<<endl;;

    if (Mission_stage != Current_Mission_stage){
        


        // Generate trajectory while mission stage change
        Current_Mission_stage = Mission_stage;//Update Current_Mission_stage
        Current_stage_mission = waypoints.at(Mission_stage-1);
        Mission_state = Current_stage_mission[0];
        if (Mission_state == 1){ //state = 1
            DesUGVpose = Vec2(Current_stage_mission[1],Current_stage_mission[2]);
            PID_duration = Current_stage_mission[3];
            PID_InitTime = ros::Time::now().toSec();
        }
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "scout_vicon");
    ros::NodeHandle nh;
    ros::Subscriber ugvpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_car/pose", 20, UGVPose_cb);
    ros::Subscriber ugvdespose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/scout_wp/pose", 20, UGVdesPose_cb);
    ros::Publisher  pub_twist =nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    ros::Rate ros_rate(50);
    // nh.getParam("/scout_vicon_node/FSM_mission", FSM_mission);
    // nh.getParam("/scout_vicon_node/External_pos_setpoint", External_pos_setpoint);
    nh.getParam("/vicon_car/MaxVelocity", MaxVelocity);
    nh.getParam("/vicon_car/MaxTurnrate", MaxTurnrate);
    
    cout << " System maximun velocity : " << MaxVelocity << " m/s  Angular Velocity: " <<  MaxTurnrate << " rad/s" << endl;
    if (FSM_mission){
        External_pos_setpoint=false;
        cout << " FSM activated" << endl;
    }
    
    while(ros::ok())
    {
        if(getUGVpose)
            break;

        ros::spinOnce();
        ros_rate.sleep();
    }

    while(ros::ok())
    {   
        if(System_init){
            cout<<"hi"<<endl;
            Finite_stage_mission();
            System_init = false;
            // if (pose_XYyaw[0] == 0 && pose_XYyaw[1] == 0){
            //     cout << "------------------------------------------------------------------------------" << endl;
            //     cout << "Check VRPN, not recieving position" << endl;
            //     cout << "------------------------------------------------------------------------------" << endl;
            //     FSM_mission = false;
            // }
        }
        if(External_pos_setpoint){
            // cout<<"hello1"<<endl;
            DesUGVpose = Vec2(UGV_pose_desire.pose.position.x,UGV_pose_desire.pose.position.y);
            ugv_twist_pub(ugv_poistion_controller_PID(pose_XYyaw,DesUGVpose));
        }
        if(FSM_mission){
            // cout<<"hello2"<<endl;
            Finite_state_machine();
            ugv_pub();
        }
        pub_twist.publish(UGV_twist_pub);
        /*Mission information cout**********************************************/
        if(coutcounter > 10){ //reduce cout rate
            cout << "------------------------------------------------------------------------------" << endl;
            cout << "Mission_Stage: " << Mission_stage << "    Mission_total_stage: " << waypoints.size() << endl;
            cout << "vicon__pos_x: " << pose_XYyaw[0] << " y: " << pose_XYyaw[1] << " Yaw: "<< pose_XYyaw[2] << endl;
            cout << "desiredpos_x: " << DesUGVpose[0] << " y: " << DesUGVpose[1]<< endl;
            cout << "desiredtwist_x: " << UGV_twist_pub.linear.x << " az: " << UGV_twist_pub.angular.z << endl;
            cout << "ROS_time: " << fixed << ros::Time::now().toSec() << endl;
            cout << "------------------------------------------------------------------------------" << endl;
            coutcounter = 0;
        }else{coutcounter++;}
        // cout<<Mission_stage<<endl;
        ros::spinOnce();
        ros_rate.sleep();
    }

    return 0;
}