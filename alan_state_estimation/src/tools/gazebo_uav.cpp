#include "../include/tools/essential.h"
#include <gazebo_msgs/ModelStates.h>

static int indi = 0;
static int state_i = 0;
static geometry_msgs::PoseStamped pose_object_forwarded;
void uav_pose_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    if(indi == 0)
    {
        for(int i = 0; i < msg->name.size(); i++)
        {
            if(msg->name[i] == "iris")
            {
                state_i = i;
                indi ++;
            }
        }
    }

    pose_object_forwarded.header.stamp = ros::Time::now();

    pose_object_forwarded.pose.position.x = msg->pose[state_i].position.x;
    pose_object_forwarded.pose.position.y = msg->pose[state_i].position.y;
    pose_object_forwarded.pose.position.z = msg->pose[state_i].position.z;

    pose_object_forwarded.pose.orientation.w = msg->pose[state_i].orientation.w;
    pose_object_forwarded.pose.orientation.x = msg->pose[state_i].orientation.x;
    pose_object_forwarded.pose.orientation.y = msg->pose[state_i].orientation.y;
    pose_object_forwarded.pose.orientation.z = msg->pose[state_i].orientation.z;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gazebo_uav");
    ros::NodeHandle nh;

    ros::Subscriber uav_pose_sub = nh.subscribe<gazebo_msgs::ModelStates>
            ("/gazebo/model_states", 1, &uav_pose_callback);

    ros::Publisher uav_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav_gazebo/mavros/local_position/pose", 1, true);

    ros::Rate gazebo_uav_rate(50);

    while(ros::ok())
    {
        uav_pose_pub.publish(pose_object_forwarded);

        ros::spinOnce();
        gazebo_uav_rate.sleep();

    }



}
