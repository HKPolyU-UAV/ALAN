#include "include/msgsync.h"

void alan::MsgSyncNodelet::uav_msg_callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::Imu::ConstPtr& imu)
{
    uav_odom = *odom;
    uav_imu = *imu;

    uav_alan_msg.position.x = uav_odom.pose.pose.position.x;
    uav_alan_msg.position.y = uav_odom.pose.pose.position.y;
    uav_alan_msg.position.z = uav_odom.pose.pose.position.z;

    uav_pos_world = Eigen::Vector3d(
        uav_alan_msg.position.x,
        uav_alan_msg.position.y,
        uav_alan_msg.position.z
    );

    uav_alan_msg.velocity.x = uav_odom.twist.twist.linear.x;
    uav_alan_msg.velocity.y = uav_odom.twist.twist.linear.y;
    uav_alan_msg.velocity.z = uav_odom.twist.twist.linear.z;

    Eigen::Translation3d t_(
        uav_odom.pose.pose.position.x, 
        uav_odom.pose.pose.position.y, 
        uav_odom.pose.pose.position.z
        );

    Eigen::Quaterniond q_(
        uav_odom.pose.pose.orientation.w,
        uav_odom.pose.pose.orientation.x,
        uav_odom.pose.pose.orientation.y,
        uav_odom.pose.pose.orientation.z
        );
    
    uavOdomPose = t_ * q_;

    uav_odom_initiated = true;
    
    uav_acc_body(0) = uav_imu.linear_acceleration.x;
    uav_acc_body(1) = uav_imu.linear_acceleration.y;
    uav_acc_body(2) = uav_imu.linear_acceleration.z;

    uav_acc_world = uavOdomPose.rotation() * uav_acc_body;
    uav_alan_msg.acceleration.x = uav_acc_body(0);
    uav_alan_msg.acceleration.y = uav_acc_body(1);
    uav_alan_msg.acceleration.z = uav_acc_body(2) - 9.8066;

    uav_alan_msg.orientation.ow = uav_odom.pose.pose.orientation.w;
    uav_alan_msg.orientation.ox = uav_odom.pose.pose.orientation.x;
    uav_alan_msg.orientation.oy = uav_odom.pose.pose.orientation.y;
    uav_alan_msg.orientation.oz = uav_odom.pose.pose.orientation.z;

    // uav_alan_msg
    uav_alan_msg.frame = "world";

    uav_pub_AlanPlannerMsg.publish(uav_alan_msg);

    Eigen::Quaterniond q;
    q.w() = uav_alan_msg.orientation.ow;
    q.x() = uav_alan_msg.orientation.ox;
    q.y() = uav_alan_msg.orientation.oy;
    q.z() = uav_alan_msg.orientation.oz;

    // double yaw = atan2(q.toRotationMatrix()(1,0), q.toRotationMatrix()(0,0));


    // Eigen::Vector3d rpy = q2rpy(q);

    // cout<<q._transformVector(Eigen::Vector3d(1,0,0))<<endl;;

//     x = cos(yaw)*cos(pitch)
// y = sin(yaw)*cos(pitch)
// z = sin(pitch)

    // cout<<rpy<<endl<<endl;;
    // cout<<yaw<<endl<<endl<<endl;

}

void alan::MsgSyncNodelet::ugv_msg_callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::Imu::ConstPtr& imu)
{
    ugv_odom = *odom;
    ugv_imu = *imu;

    ugv_alan_msg.position.x = ugv_odom.pose.pose.position.x;
    ugv_alan_msg.position.y = ugv_odom.pose.pose.position.y;
    ugv_alan_msg.position.z = ugv_odom.pose.pose.position.z;

    ugv_pos_world = Eigen::Vector3d(
        ugv_alan_msg.position.x,
        ugv_alan_msg.position.y,
        ugv_alan_msg.position.z
    );

    ugv_alan_msg.velocity.x = ugv_odom.twist.twist.linear.x;
    ugv_alan_msg.velocity.y = ugv_odom.twist.twist.linear.y;
    ugv_alan_msg.velocity.z = ugv_odom.twist.twist.linear.z;

    Eigen::Translation3d t_(
        ugv_odom.pose.pose.position.x, 
        ugv_odom.pose.pose.position.y, 
        ugv_odom.pose.pose.position.z
        );

    Eigen::Quaterniond q_(
        ugv_odom.pose.pose.orientation.w,
        ugv_odom.pose.pose.orientation.x,
        ugv_odom.pose.pose.orientation.y,
        ugv_odom.pose.pose.orientation.z
        );
    
    ugvOdomPose = t_ * q_;

    ugv_odom_initiated = true;
    
    ugv_acc_body(0) = ugv_imu.linear_acceleration.z;
    ugv_acc_body(1) = ugv_imu.linear_acceleration.x * (-1);
    ugv_acc_body(2) = ugv_imu.linear_acceleration.y * (-1);

    // ugv_acc_body(0) = ugv_imu.linear_acceleration.x;
    // ugv_acc_body(1) = ugv_imu.linear_acceleration.y;
    // ugv_acc_body(2) = ugv_imu.linear_acceleration.z;

    ugv_acc_world = ugvOdomPose.rotation() * ugv_acc_body;
    ugv_alan_msg.acceleration.x = ugv_acc_world(0);
    ugv_alan_msg.acceleration.y = ugv_acc_world(1);
    ugv_alan_msg.acceleration.z = ugv_acc_world(2) - 9.8066;

    ugv_alan_msg.orientation.ow = ugv_odom.pose.pose.orientation.w;
    ugv_alan_msg.orientation.ox = ugv_odom.pose.pose.orientation.x;
    ugv_alan_msg.orientation.oy = ugv_odom.pose.pose.orientation.y;
    ugv_alan_msg.orientation.oz = ugv_odom.pose.pose.orientation.z;

    // ugv_alan_msg
    ugv_alan_msg.frame = "world";

    ugv_pub_AlanPlannerMsg.publish(ugv_alan_msg);

}

void alan::MsgSyncNodelet::cam_msg_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    if(uav_odom_initiated && ugv_odom_initiated || true)
    {
        cam_current_PoseMsg = *pose;

        Eigen::Translation3d t_(
            cam_current_PoseMsg.pose.position.x,
            cam_current_PoseMsg.pose.position.y,
            cam_current_PoseMsg.pose.position.z
        );

        Eigen::Quaterniond q_(
            cam_current_PoseMsg.pose.orientation.w,
            cam_current_PoseMsg.pose.orientation.x,
            cam_current_PoseMsg.pose.orientation.y,
            cam_current_PoseMsg.pose.orientation.z
        );

        // cout<<cam_current_PoseMsg.pose.orientation.x<<endl;

        camPose = t_ * q_;

        set_total_bound(t_, q_);
        set_all_sfc(t_, q_);
        
        //remember to add ugv camera translation                
        // alan_sfc_pub.publish(polyh_total_bound);
        // cout<<"publisher here..."<<polyh_array_pub_object.a_series_of_Corridor.size()<<endl;

        alan_all_sfc_pub.publish(polyh_array_pub_object);

        ros::Duration(0.02).sleep();

    }
    
}

Eigen::Vector3d alan::MsgSyncNodelet::q2rpy(Eigen::Quaterniond q)
{
    return q.toRotationMatrix().eulerAngles(2,1,0);
}

Eigen::Quaterniond alan::MsgSyncNodelet::rpy2q(Eigen::Vector3d rpy)
{
    Eigen::AngleAxisd rollAngle(rpy(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rpy(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rpy(2), Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

Eigen::Vector3d alan::MsgSyncNodelet::q_rotate_vector(Eigen::Quaterniond q, Eigen::Vector3d v)
{
    return q * v;
}

alan_visualization::Tangent alan::MsgSyncNodelet::construct_tangent_plane(
    Eigen::Vector3d v1,
    Eigen::Vector3d v2,
    Eigen::Vector3d pt
)
{
    //order matters!
    Eigen::Vector3d normal_vector = get_outer_product(v1, v2);

    alan_visualization::Tangent tangent_plane;
    
    tangent_plane.n.X = normal_vector(0);
    tangent_plane.n.Y = normal_vector(1);
    tangent_plane.n.Z = normal_vector(2);

    tangent_plane.pt.X = pt.x();
    tangent_plane.pt.Y = pt.y();
    tangent_plane.pt.Z = pt.z();

    //ax + by + cz = d
    return tangent_plane;    

}

Eigen::Vector3d alan::MsgSyncNodelet::get_outer_product(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
    return v1.cross(v2);
}

alan_visualization::Tangent alan::MsgSyncNodelet::set_plane_bound(Eigen::Vector3d v, Eigen::Vector3d pt)
{
    alan_visualization::Tangent tangent_plane;
    
    // v.z() = 0;
    v.normalize();

    tangent_plane.n.X = v.x();
    tangent_plane.n.Y = v.y();
    tangent_plane.n.Z = v.z();

    tangent_plane.pt.X = pt.x();
    tangent_plane.pt.Y = pt.y();
    tangent_plane.pt.Z = pt.z();
        
    return tangent_plane;
}

void alan::MsgSyncNodelet::set_total_bound(Eigen::Translation3d t_current,Eigen::Quaterniond q_current)
{
    
    //(R, P, Y)center of cam    
    cam_center_vector = Eigen::Vector3d(1,0,0);

    // //(R, P-FOV_V/2, Y-FOV_H/2) 1st edge of the pyramid FOV
    cam_1axis_vector = q_rotate_vector(q_current, q_rotate_vector(q1, cam_center_vector));
    
    // //(R, P-FOV_V/2, Y+FOV_H/2) 2nd edge of the pyramid FOV
    cam_2axis_vector = q_rotate_vector(q_current, q_rotate_vector(q2, cam_center_vector));

    // //(R, P+FOV_V/2, Y+FOV_H/2) 3rd edge of the pyramid FOV
    cam_3axis_vector = q_rotate_vector(q_current, q_rotate_vector(q3, cam_center_vector));

    // //(R, P+FOV_V/2, Y-FOV_H/2) 4th edge of the pyramid FOV
    cam_4axis_vector = q_rotate_vector(q_current, q_rotate_vector(q4, cam_center_vector));

    
    cam_center_vector = q_rotate_vector(q_current, Eigen::Vector3d(1,0,0));


    //constuct tangent planes
    //remeber normal vector pointing outward from the polyhedron
    alan_visualization::Tangent plane1 = construct_tangent_plane(
        cam_1axis_vector, cam_2axis_vector, t_current.translation()
    );

    alan_visualization::Tangent plane2 = construct_tangent_plane(
        cam_2axis_vector, cam_3axis_vector, t_current.translation()
    );

    // alan_visualization::Tangent plane3 = construct_tangent_plane(
    //     cam_3axis_vector, cam_4axis_vector, t_.translation()
    // );

    alan_visualization::Tangent plane3 = set_plane_bound(
        Eigen::Vector3d(0,0,-1),
        t_current.translation()
    );

    alan_visualization::Tangent plane4 = construct_tangent_plane(
        cam_4axis_vector, cam_1axis_vector, t_current.translation()
    );      
                

    Eigen::Vector3d ugv2uav_vector = uav_pos_world - ugv_pos_world;

    ugv2uav_vector.normalize();


    // alan_visualization::Tangent plane_bound = set_plane_bound(
    //     cam_center_vector, 
    //     uav_pos_world + ugv2uav_vector                
    // );

    Eigen::Vector3d temp_bound;
    temp_bound = t_current.translation() + q_rotate_vector(q_current, Eigen::Vector3d(4,0,0));
    cam_center_vector.z() = 0;

    alan_visualization::Tangent plane_bound1 = set_plane_bound(
        cam_center_vector, 
        temp_bound                
    );

    temp_bound = t_current.translation() + q_rotate_vector(q_current, Eigen::Vector3d(0.2,0,0));
    cam_center_vector = cam_center_vector * -1;
    // cam_center_vector.z() = 0;

    alan_visualization::Tangent plane_bound2 = set_plane_bound(
        cam_center_vector, 
        temp_bound                
    );

    polyh_total_bound.PolyhedronTangentArray.clear();
        
    polyh_total_bound.PolyhedronTangentArray.push_back(plane1);

    polyh_total_bound.PolyhedronTangentArray.push_back(plane2);

    polyh_total_bound.PolyhedronTangentArray.push_back(plane3);

    polyh_total_bound.PolyhedronTangentArray.push_back(plane4);

    polyh_total_bound.PolyhedronTangentArray.push_back(plane_bound1);

    polyh_total_bound.PolyhedronTangentArray.push_back(plane_bound2);

}

void alan::MsgSyncNodelet::set_all_sfc(Eigen::Translation3d t_current,Eigen::Quaterniond q_current)
{
    polyh_array_pub_object.a_series_of_Corridor.clear();

    alan_visualization::Tangent tangent_plane_temp;
    alan_visualization::Polyhedron polyhedron_temp;

    polyhedron_temp.PolyhedronTangentArray.clear();

    //first corridor 

    //1st plane
    polyhedron_temp.PolyhedronTangentArray.push_back(polyh_total_bound.PolyhedronTangentArray[0]);
    
    //2nd plane
    polyhedron_temp.PolyhedronTangentArray.push_back(polyh_total_bound.PolyhedronTangentArray[1]);

    //3rd plane
    polyhedron_temp.PolyhedronTangentArray.push_back(polyh_total_bound.PolyhedronTangentArray[2]);
    
    //4th plane
    polyhedron_temp.PolyhedronTangentArray.push_back(polyh_total_bound.PolyhedronTangentArray[3]);

    
    Eigen::Vector3d temp_normal = q_rotate_vector(q_current, Eigen::Vector3d(1,0,0) * -1);
    temp_normal.z() = 0;

    alan_visualization::Tangent temp_tangent = set_plane_bound(
        temp_normal,
         t_current.translation() + q_rotate_vector(q_current,Eigen::Vector3d(final_corridor_length,0,0))
        );


    //5th plane
    polyhedron_temp.PolyhedronTangentArray.push_back(temp_tangent);
    
    //6th plane
    polyhedron_temp.PolyhedronTangentArray.push_back(polyh_total_bound.PolyhedronTangentArray[4]);

    polyh_array_pub_object.a_series_of_Corridor.push_back(polyhedron_temp);





    //second corridor
    polyhedron_temp.PolyhedronTangentArray.clear();

    //1st plane

    polyhedron_temp.PolyhedronTangentArray.push_back(polyh_total_bound.PolyhedronTangentArray[0]);
    temp_normal = Eigen::Vector3d(0,0,1);

    alan_visualization::Tangent ceil_tangent = set_plane_bound(
        temp_normal,
        t_current.translation() + q_rotate_vector(q_current, Eigen::Vector3d(0,0,final_corridor_height))
        );
    
    polyhedron_temp.PolyhedronTangentArray.push_back(ceil_tangent);
        
    //2nd plane
    polyhedron_temp.PolyhedronTangentArray.push_back(polyh_total_bound.PolyhedronTangentArray[1]);

    //3rd plane
    polyhedron_temp.PolyhedronTangentArray.push_back(polyh_total_bound.PolyhedronTangentArray[2]);
    
    //4th plane
    polyhedron_temp.PolyhedronTangentArray.push_back(polyh_total_bound.PolyhedronTangentArray[3]);
        
    //5th plane
    polyhedron_temp.PolyhedronTangentArray.push_back(polyh_total_bound.PolyhedronTangentArray[5]);

    temp_normal = q_rotate_vector(q_current, Eigen::Vector3d(1,0,0) * 1);
    temp_normal.z() = 0;

    temp_tangent = set_plane_bound(
        temp_normal,
        t_current.translation() + q_rotate_vector(q_current, Eigen::Vector3d(final_corridor_length,0,0))
        );

    
    //6th plane
    polyhedron_temp.PolyhedronTangentArray.push_back(temp_tangent);

    polyh_array_pub_object.a_series_of_Corridor.push_back(polyhedron_temp);

}
