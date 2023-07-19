#include <iostream>
#include "include/essential.h"
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_geometry/polyhedron.h>

#include "alan_visualization/PolyhedronArray.h"
#include "alan_landing_planning/TrajArray.h"

#include "visualization_msgs/Marker.h"

#include "./include/rviz_vehicle.hpp"
#include <tf/transform_broadcaster.h>

using namespace std;

static decomp_ros_msgs::PolyhedronArray sfc_pub_vis_object_polyh;
static decomp_ros_msgs::Polyhedron sfc_pub_vis_object_tangent;
static visualization_msgs::Marker traj_points;
static visualization_msgs::Marker trajArray_points;
static visualization_msgs::Marker ctrl_points;

static geometry_msgs::PoseStamped ugv_pose;


static Eigen::VectorXd c2b_ugv;

static Eigen::Isometry3d ugv_body_pose_in_world;

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

static bool rviz_sfc_initiated = false;
void sfc_msg_callback(const alan_visualization::PolyhedronArray::ConstPtr & msg)
{
    geometry_msgs::Point temp_sfc_p, temp_sfc_n;

    sfc_pub_vis_object_polyh.polyhedrons.clear();   

    // cout<<"size of corridors..."<<msg->a_series_of_Corridor.size()<<endl;
    sfc_pub_vis_object_tangent.points.clear();
    sfc_pub_vis_object_tangent.normals.clear();

    for(auto what : msg->a_series_of_Corridor)
    {
        for(auto whatelse : what.PolyhedronTangentArray)
        {
            // cout<<what.PolyhedronTangentArray.size()<<endl;
            temp_sfc_p.x = whatelse.pt.X;
            temp_sfc_p.y = whatelse.pt.Y;
            temp_sfc_p.z = whatelse.pt.Z;

            temp_sfc_n.x = whatelse.n.X;
            temp_sfc_n.y = whatelse.n.Y;
            temp_sfc_n.z = whatelse.n.Z;

            sfc_pub_vis_object_tangent.points.push_back(temp_sfc_p);
            sfc_pub_vis_object_tangent.normals.push_back(temp_sfc_n);

            sfc_pub_vis_object_polyh.polyhedrons.push_back(sfc_pub_vis_object_tangent);

            // cout<<sf            
        }
        // cout<<sfc_pub_vis_object_tangent.points.size()<<endl;
        // cout<<sfc_pub_vis_object_tangent.normals.size()<<endl;
        sfc_pub_vis_object_tangent.points.clear();
        sfc_pub_vis_object_tangent.normals.clear();
    }

    sfc_pub_vis_object_polyh.header.frame_id = "body";  


    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(
        ugv_pose.pose.position.x,
        ugv_pose.pose.position.y,
        ugv_pose.pose.position.z
        ) 
    );
    tf::Quaternion q;
    q.setW(ugv_pose.pose.orientation.w);
    q.setX(ugv_pose.pose.orientation.x);
    q.setY(ugv_pose.pose.orientation.y);
    q.setZ(ugv_pose.pose.orientation.z);
    
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "body"));
  
    
    

    // cout<<sfc_pub_vis_object_polyh.polyhedrons.size()<<endl;

    rviz_sfc_initiated = true;
}

static bool rviz_traj_initiated = false;
void traj_msg_callback(const alan_landing_planning::Traj::ConstPtr& msg)
{
    geometry_msgs::Point posi_temp;

    // cout<<msg->trajectory.size()<<endl;

    traj_points.points.clear();
    
    for(auto what : msg->trajectory)
    {
        posi_temp.x = what.position.x;
        posi_temp.y = what.position.y;
        posi_temp.z = what.position.z;
        traj_points.points.push_back(posi_temp);
    }      
        
    rviz_traj_initiated = true;

}

static bool rviz_traj_array_initiated = false;
void trajArray_msg_callback(const alan_landing_planning::TrajArray::ConstPtr& msg)
{
    geometry_msgs::Point posi_temp;

    trajArray_points.points.clear();

    for(auto what : msg->trajectory_array)
    {
        for(auto whatelse : what.trajectory)
        {
            posi_temp.x = whatelse.position.x;
            posi_temp.y = whatelse.position.y;
            posi_temp.z = whatelse.position.z;
            trajArray_points.points.push_back(posi_temp);            
        }
    }
    rviz_traj_array_initiated = true;

}

static bool ctrlPts_initiated = false;
void ctrlPts_msg_callback(const alan_landing_planning::Traj::ConstPtr& msg)
{
    geometry_msgs::Point posi_temp;

    // cout<<msg->trajectory.size()<<endl;

    ctrl_points.points.clear();
    
    for(auto what : msg->trajectory)
    {
        posi_temp.x = what.position.x;
        posi_temp.y = what.position.y;
        posi_temp.z = what.position.z;
        ctrl_points.points.push_back(posi_temp);
    }      

    ctrlPts_initiated = true;

}

static bool rviz_uav_initiated = false;
static geometry_msgs::PoseStamped uav_pose;
void uavAlanMsgCallback(const alan_landing_planning::AlanPlannerMsg::ConstPtr& msg)
{
    uav_pose.pose.position.x = msg->position.x;
    uav_pose.pose.position.y = msg->position.y;
    uav_pose.pose.position.z = msg->position.z;

    uav_pose.pose.orientation.w = msg->orientation.ow;
    uav_pose.pose.orientation.x = msg->orientation.ox;
    uav_pose.pose.orientation.y = msg->orientation.oy;
    uav_pose.pose.orientation.z = msg->orientation.oz;

    rviz_uav_initiated = true;

}

static bool rviz_ugv_initiated = false;
void ugvAlanMsgCallback(const alan_landing_planning::AlanPlannerMsg::ConstPtr& msg)
{
    //when ugv is directly received from vicon,
    //or gps
    //or other info directly received from body frame...
    ugv_pose.pose.position.x = msg->position.x;
    ugv_pose.pose.position.y = msg->position.y;
    ugv_pose.pose.position.z = msg->position.z;

    ugv_pose.pose.orientation.w = msg->orientation.ow;
    ugv_pose.pose.orientation.x = msg->orientation.ox;
    ugv_pose.pose.orientation.y = msg->orientation.oy;
    ugv_pose.pose.orientation.z = msg->orientation.oz;

    ugv_body_pose_in_world =
        Eigen::Translation3d(
            ugv_pose.pose.position.x,
            ugv_pose.pose.position.y,
            ugv_pose.pose.position.z
        ) * Eigen::Quaterniond(
            ugv_pose.pose.orientation.w,
            ugv_pose.pose.orientation.x,
            ugv_pose.pose.orientation.y,
            ugv_pose.pose.orientation.z
    );

    // cout<<ugv_pose.pose.orientation.w<<endl;

    //when ugv is received from imu_pose (in GH034)
    // cout<<c2b_ugv(4)<<endl;

    // Eigen::Quaterniond c2b_local = rpy2q(
    //     Eigen::Vector3d(    
    //         c2b_ugv(3),
    //         c2b_ugv(4),
    //         c2b_ugv(5)            
    //     )
    // );//cam_to_body
    

    // c2b_local = Eigen::Quaterniond(
    //     msg->orientation.ow,
    //     msg->orientation.ox,
    //     msg->orientation.oy,
    //     msg->orientation.oz
    // ) * c2b_local ;//order matters
    //got cam, 

    // Eigen::Vector3d p_temp = q_rotate_vector(
    //     c2b_local, 
    //     Eigen::Vector3d(c2b_ugv(0), c2b_ugv(1), c2b_ugv(2))
    // );


    // ugv_pose.pose.orientation.w = c2b_local.w();
    // ugv_pose.pose.orientation.x = c2b_local.x();
    // ugv_pose.pose.orientation.y = c2b_local.y();
    // ugv_pose.pose.orientation.z = c2b_local.z();

    // ugv_pose.pose.position.x = msg->position.x - p_temp(0);
    // ugv_pose.pose.position.y = msg->position.y - p_temp(1);
    // ugv_pose.pose.position.z = msg->position.z - p_temp(2);    

    rviz_ugv_initiated = true;

}

int main(int argc, char** argv)
{    
	ros::init(argc, argv, "alan_rviz");
    ros::NodeHandle nh;
    
    ros::Subscriber sfc_sub = nh.subscribe<alan_visualization::PolyhedronArray>
            ("/alan_state_estimation/msgsync/polyhedron_array", 1, &sfc_msg_callback);
    ros::Subscriber traj_sub = nh.subscribe<alan_landing_planning::Traj>
            ("/alan_visualization/traj", 1, &traj_msg_callback);
    ros::Subscriber trajArray_sub = nh.subscribe<alan_landing_planning::TrajArray>
            ("/alan_visualization/trajArray", 1, &trajArray_msg_callback);
    ros::Subscriber ctrlPts_sub = nh.subscribe<alan_landing_planning::Traj>
            ("/alan_visualization/ctrlPts", 1, &ctrlPts_msg_callback);
    ros::Subscriber uav_pose_sub = nh.subscribe<alan_landing_planning::AlanPlannerMsg>
            ("/alan_state_estimation/msgsync/uav/alan_planner_msg", 1, &uavAlanMsgCallback);
    ros::Subscriber ugv_pose_sub = nh.subscribe<alan_landing_planning::AlanPlannerMsg>
            ("/alan_state_estimation/msgsync/ugv/alan_planner_msg", 1, &ugvAlanMsgCallback);
        
    ros::Publisher polyh_vis_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("/polyhedron_array", 1, true);
    ros::Publisher traj_vis_pub = nh.advertise <visualization_msgs::Marker>("/gt_points/traj", 1, true);
    ros::Publisher trajArray_vis_pub = nh.advertise <visualization_msgs::Marker>("/gt_points/trajSamples", 1, true);
    ros::Publisher ctrl_pts_vis_pub = nh.advertise <visualization_msgs::Marker>("/gt_points/trajCtrlpts", 1, true);

    traj_points.header.frame_id = "body";
    
    traj_points.ns = "GT_points";

    traj_points.id = 0;
    traj_points.action = visualization_msgs::Marker::ADD;
    traj_points.pose.orientation.w = 1.0;
    traj_points.type = visualization_msgs::Marker::SPHERE_LIST;
    traj_points.scale.x = traj_points.scale.y = traj_points.scale.z = 0.02;
    traj_points.color.a=1;
    traj_points.color.g=1;
    traj_points.color.r=0;
    traj_points.color.b=0;


    trajArray_points.header.frame_id = "body";

    trajArray_points.ns = "GT_points";

    trajArray_points.id = 0;
    trajArray_points.action = visualization_msgs::Marker::ADD;
    trajArray_points.pose.orientation.w = 1.0;
    trajArray_points.type = visualization_msgs::Marker::SPHERE_LIST;
    trajArray_points.scale.x = trajArray_points.scale.y = trajArray_points.scale.z = 0.008;
    trajArray_points.color.a=1;
    trajArray_points.color.g=0;
    trajArray_points.color.r=0;
    trajArray_points.color.b=1;


    ctrl_points.header.frame_id = "body";

    ctrl_points.ns = "GT_points";

    ctrl_points.id = 0;
    ctrl_points.action = visualization_msgs::Marker::ADD;
    ctrl_points.pose.orientation.w = 1.0;
    ctrl_points.type = visualization_msgs::Marker::SPHERE_LIST;
    ctrl_points.scale.x = ctrl_points.scale.y = ctrl_points.scale.z = 0.04;
    ctrl_points.color.a=1;
    ctrl_points.color.g=1;
    ctrl_points.color.r=1;
    ctrl_points.color.b=0;

    rviz_vehicle uav_rviz = rviz_vehicle(nh, UAV, false);



    Eigen::VectorXd cameraEX;

    cameraEX.resize(6);
    XmlRpc::XmlRpcValue extrinsics_list;
    
    nh.getParam("/alan_master/cam_ugv_extrinsics_d455", extrinsics_list);                
    
    for(int i = 0; i < 6; i++)
    {                    
        cameraEX(i) = extrinsics_list[i];                    
    }
    
    //x, y, z
    //r, p, y
    c2b_ugv.resize(6);

    c2b_ugv(0) = cameraEX(0);
    c2b_ugv(1) = cameraEX(1);
    c2b_ugv(2) = cameraEX(2);

    c2b_ugv(3) = cameraEX(3) / 180.0 * M_PI;;//r
    c2b_ugv(4) = cameraEX(4) / 180.0 * M_PI;//p
    c2b_ugv(5) = cameraEX(5) / 180.0 * M_PI;//y

    // cout<<c2b_ugv<<endl;
    ROS_INFO("RVIZ for ALan...\n");
    

    rviz_vehicle ugv_rviz = rviz_vehicle(nh, UGV, true, c2b_ugv);

    ros::Rate visrate(60);

    while(ros::ok())
    {
        
        if(rviz_uav_initiated)
            uav_rviz.rviz_pub_vehicle(uav_pose);

        if(rviz_ugv_initiated)
            ugv_rviz.rviz_pub_vehicle(ugv_pose);

        if(rviz_sfc_initiated)
            polyh_vis_pub.publish(sfc_pub_vis_object_polyh);
        
        // cout<<sfc_pub_vis_object_polyh.polyhedrons.size()<<endl;

        if(
            rviz_traj_initiated
            // rviz_traj_initiated && 
            // rviz_traj_array_initiated &&
            // ctrlPts_initiated
        )
        {
            traj_points.header.stamp = ros::Time::now();
            trajArray_points.header.stamp = ros::Time::now();

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(
                ugv_pose.pose.position.x,
                ugv_pose.pose.position.y,
                ugv_pose.pose.position.z
                ) 
            );
            tf::Quaternion q;
            q.setW(ugv_pose.pose.orientation.w);
            q.setX(ugv_pose.pose.orientation.x);
            q.setY(ugv_pose.pose.orientation.y);
            q.setZ(ugv_pose.pose.orientation.z);
            
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "body"));
            traj_vis_pub.publish(traj_points);  
            // trajArray_vis_pub.publish(trajArray_points);
            // ctrl_pts_vis_pub.publish(ctrl_points);
        }                                                 
            
        ros::spinOnce();
        visrate.sleep();

    }

	return 0;
}