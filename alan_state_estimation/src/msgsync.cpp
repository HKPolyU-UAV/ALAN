#include "include/msgsync.h"

void alan::MsgSyncNodelet::led_odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
    led_odom = *odom;
    led_odom_inititated = true;


    if(!failsafe_on)
    //if no failsafe,
    //no need to go through uav_vrpn 
    {
        uav_pos_world = Eigen::Vector3d(
            led_odom.pose.pose.position.x,
            led_odom.pose.pose.position.y,
            led_odom.pose.pose.position.z
        );

        uavOdomPose =
            Eigen::Translation3d(uav_pos_world) * 
            Eigen::Quaterniond(
                led_odom.pose.pose.orientation.w,
                led_odom.pose.pose.orientation.x,
                led_odom.pose.pose.orientation.y,
                led_odom.pose.pose.orientation.z
        );
        
        if(
            led_odom_inititated &&
            uav_imu_initiated
        )
        {
            uav_alan_msg.position.x = led_odom.pose.pose.position.x;
            uav_alan_msg.position.y = led_odom.pose.pose.position.y;
            uav_alan_msg.position.z = led_odom.pose.pose.position.z;

            uav_alan_msg.orientation.ow = led_odom.pose.pose.orientation.w;
            uav_alan_msg.orientation.ox = led_odom.pose.pose.orientation.x;
            uav_alan_msg.orientation.oy = led_odom.pose.pose.orientation.y;
            uav_alan_msg.orientation.oz = led_odom.pose.pose.orientation.z;

            uav_alan_msg.velocity.x = led_odom.twist.twist.linear.x;
            uav_alan_msg.velocity.y = led_odom.twist.twist.linear.y;
            uav_alan_msg.velocity.z = led_odom.twist.twist.linear.z;

            uav_acc_world = uavOdomPose.rotation() * uav_acc_body;

            uav_alan_msg.acceleration.x = uav_acc_world.x();
            uav_alan_msg.acceleration.y = uav_acc_world.y();
            uav_alan_msg.acceleration.z = uav_acc_world.z() - 9.8066;

            uav_alan_msg.time_stamp = led_odom.header.stamp;
            uav_alan_msg.good2fly = true;
            uav_alan_msg.frame = "world";

            uav_pub_AlanPlannerMsg.publish(uav_alan_msg);
        }
    }
}

void alan::MsgSyncNodelet::uav_vrpn_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uav_vrpn_pose = *pose;
    uav_vrpn_pose_initiated = true;

    bool use_led = false;
    // cout<<"hi"<<endl;
    // cout<<failsafe_on<<endl;

    if(failsafe_on)
    {
        if(
            uav_vrpn_pose_initiated &&
            uav_vrpn_twist_initiated &&
            uav_imu_initiated
        )
        {
            if(led_odom_inititated)
            {
                double delta = 
                    pow(
                        led_odom.pose.pose.position.x - uav_vrpn_pose.pose.position.x, 2
                    )
                + pow(
                        led_odom.pose.pose.position.y - uav_vrpn_pose.pose.position.y, 2
                    )
                + pow(
                        led_odom.pose.pose.position.z - uav_vrpn_pose.pose.position.z, 2
                    );

                delta = sqrt(delta);

                // cout<<delta<<endl;

                if(delta < 0.14)
                {
                    uav_pos_world = Eigen::Vector3d(
                        led_odom.pose.pose.position.x,
                        led_odom.pose.pose.position.y,
                        led_odom.pose.pose.position.z
                    );

                    uavOdomPose =
                        Eigen::Translation3d(uav_pos_world) * 
                        Eigen::Quaterniond(
                            led_odom.pose.pose.orientation.w,
                            led_odom.pose.pose.orientation.x,
                            led_odom.pose.pose.orientation.y,
                            led_odom.pose.pose.orientation.z
                    );

                    uav_alan_msg.position.x = led_odom.pose.pose.position.x;
                    uav_alan_msg.position.y = led_odom.pose.pose.position.y;
                    uav_alan_msg.position.z = led_odom.pose.pose.position.z;

                    uav_alan_msg.orientation.ow = led_odom.pose.pose.orientation.w;
                    uav_alan_msg.orientation.ox = led_odom.pose.pose.orientation.x;
                    uav_alan_msg.orientation.oy = led_odom.pose.pose.orientation.y;
                    uav_alan_msg.orientation.oz = led_odom.pose.pose.orientation.z;

                    uav_alan_msg.velocity.x = led_odom.twist.twist.linear.x;
                    uav_alan_msg.velocity.y = led_odom.twist.twist.linear.y;
                    uav_alan_msg.velocity.z = led_odom.twist.twist.linear.z;

                    uav_acc_world = uavOdomPose.rotation() * uav_acc_body;

                    uav_alan_msg.acceleration.x = uav_acc_world.x();
                    uav_alan_msg.acceleration.y = uav_acc_world.y();
                    uav_alan_msg.acceleration.z = uav_acc_world.z() - 9.8066;

                    uav_alan_msg.time_stamp = led_odom.header.stamp;
                    uav_alan_msg.good2fly = true;
                    uav_alan_msg.frame = "world";                    
                }
                else
                {
                    uav_pos_world = Eigen::Vector3d(
                        uav_vrpn_pose.pose.position.x,
                        uav_vrpn_pose.pose.position.y,
                        uav_vrpn_pose.pose.position.z
                    );

                    uavOdomPose =
                        Eigen::Translation3d(uav_pos_world) * 
                        Eigen::Quaterniond(
                            uav_vrpn_pose.pose.orientation.w,
                            uav_vrpn_pose.pose.orientation.x,
                            uav_vrpn_pose.pose.orientation.y,
                            uav_vrpn_pose.pose.orientation.z
                    );

                    uav_alan_msg.position.x = uav_vrpn_pose.pose.position.x;
                    uav_alan_msg.position.y = uav_vrpn_pose.pose.position.y;
                    uav_alan_msg.position.z = uav_vrpn_pose.pose.position.z;

                    uav_alan_msg.orientation.ow = uav_vrpn_pose.pose.orientation.w;
                    uav_alan_msg.orientation.ox = uav_vrpn_pose.pose.orientation.x;
                    uav_alan_msg.orientation.oy = uav_vrpn_pose.pose.orientation.y;
                    uav_alan_msg.orientation.oz = uav_vrpn_pose.pose.orientation.z;

                    uav_alan_msg.velocity.x = uav_vrpn_twist.twist.linear.x;
                    uav_alan_msg.velocity.y = uav_vrpn_twist.twist.linear.y;
                    uav_alan_msg.velocity.z = uav_vrpn_twist.twist.linear.z;

                    uav_acc_world = uavOdomPose.rotation() * uav_acc_body;

                    uav_alan_msg.acceleration.x = uav_acc_world.x();
                    uav_alan_msg.acceleration.y = uav_acc_world.y();
                    uav_alan_msg.acceleration.z = uav_acc_world.z() - 9.8066;

                    uav_alan_msg.time_stamp = uav_vrpn_pose.header.stamp;
                    uav_alan_msg.good2fly = true;
                    uav_alan_msg.frame = "world";                    
                }                
            }
            else
            {
                uav_pos_world = Eigen::Vector3d(
                    uav_vrpn_pose.pose.position.x,
                    uav_vrpn_pose.pose.position.y,
                    uav_vrpn_pose.pose.position.z
                );

                uavOdomPose =
                    Eigen::Translation3d(uav_pos_world) * 
                    Eigen::Quaterniond(
                        uav_vrpn_pose.pose.orientation.w,
                        uav_vrpn_pose.pose.orientation.x,
                        uav_vrpn_pose.pose.orientation.y,
                        uav_vrpn_pose.pose.orientation.z
                );


                uav_alan_msg.position.x = uav_vrpn_pose.pose.position.x;
                uav_alan_msg.position.y = uav_vrpn_pose.pose.position.y;
                uav_alan_msg.position.z = uav_vrpn_pose.pose.position.z;

                uav_alan_msg.orientation.ow = uav_vrpn_pose.pose.orientation.w;
                uav_alan_msg.orientation.ox = uav_vrpn_pose.pose.orientation.x;
                uav_alan_msg.orientation.oy = uav_vrpn_pose.pose.orientation.y;
                uav_alan_msg.orientation.oz = uav_vrpn_pose.pose.orientation.z;

                uav_alan_msg.velocity.x = uav_vrpn_twist.twist.linear.x;
                uav_alan_msg.velocity.y = uav_vrpn_twist.twist.linear.y;
                uav_alan_msg.velocity.z = uav_vrpn_twist.twist.linear.z;

                uav_acc_world = uavOdomPose.rotation() * uav_acc_body;

                uav_alan_msg.acceleration.x = uav_acc_world.x();
                uav_alan_msg.acceleration.y = uav_acc_world.y();
                uav_alan_msg.acceleration.z = uav_acc_world.z() - 9.8066;

                uav_alan_msg.time_stamp = uav_vrpn_pose.header.stamp;
                uav_alan_msg.good2fly = true;
                uav_alan_msg.frame = "world";                
            }

            Eigen::Vector2d cam2uav_vector = 
                Eigen::Vector2d(
                    uav_alan_msg.position.x - cam_pos_world.x(),
                    uav_alan_msg.position.y - cam_pos_world.y()
            );

            if(cam2uav_vector.norm() > 2.5)
                uav_alan_msg.good2fly = false;
            else
                uav_alan_msg.good2fly = true;
            
            uav_pub_AlanPlannerMsg.publish(uav_alan_msg);
        }
    }
}

void alan::MsgSyncNodelet::uav_vrpn_twist_callback(const geometry_msgs::TwistStamped::ConstPtr& twist)
{
    uav_vrpn_twist = *twist;
    uav_vrpn_twist_initiated = true;
}

void alan::MsgSyncNodelet::uav_imu_callback(const sensor_msgs::Imu::ConstPtr& imu)
{
    uav_imu = *imu;
    uav_acc_body(0) = uav_imu.linear_acceleration.x; 
    uav_acc_body(1) = uav_imu.linear_acceleration.y;
    uav_acc_body(2) = uav_imu.linear_acceleration.z;

    uav_imu_initiated = true;
}

void alan::MsgSyncNodelet::ugv_vrpn_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    ugv_vrpn_pose = *pose;
    ugv_vrpn_pose_initiated = true;

    if(
        ugv_vrpn_pose_initiated &&
        ugv_vrpn_twist_initiated &&
        ugv_imu_initiated
    )
    {
        ugv_pos_world = Eigen::Vector3d(
            ugv_vrpn_pose.pose.position.x,
            ugv_vrpn_pose.pose.position.y,
            ugv_vrpn_pose.pose.position.z
        );

        ugvOdomPose = 
            Eigen::Translation3d(ugv_pos_world) *
            Eigen::Quaterniond(
                ugv_vrpn_pose.pose.orientation.w,
                ugv_vrpn_pose.pose.orientation.x,
                ugv_vrpn_pose.pose.orientation.y,
                ugv_vrpn_pose.pose.orientation.z
        );

        ugv_alan_msg.position.x = ugv_vrpn_pose.pose.position.x;
        ugv_alan_msg.position.y = ugv_vrpn_pose.pose.position.y;
        ugv_alan_msg.position.z = ugv_vrpn_pose.pose.position.z;

        ugv_alan_msg.orientation.ow = ugv_vrpn_pose.pose.orientation.w;
        ugv_alan_msg.orientation.ox = ugv_vrpn_pose.pose.orientation.x;
        ugv_alan_msg.orientation.oy = ugv_vrpn_pose.pose.orientation.y;
        ugv_alan_msg.orientation.oz = ugv_vrpn_pose.pose.orientation.z;

        ugv_alan_msg.velocity.x = ugv_vrpn_twist.twist.linear.x;
        ugv_alan_msg.velocity.y = ugv_vrpn_twist.twist.linear.y;
        ugv_alan_msg.velocity.z = ugv_vrpn_twist.twist.linear.z;

        ugv_acc_world = ugvOdomPose.rotation() * ugv_acc_body;

        ugv_alan_msg.acceleration.x = ugv_acc_world.x();
        ugv_alan_msg.acceleration.y = ugv_acc_world.y();
        ugv_alan_msg.acceleration.z = ugv_acc_world.z() - 9.8066;

        ugv_alan_msg.time_stamp = ugv_vrpn_pose.header.stamp;
        ugv_alan_msg.good2fly = true;
        ugv_alan_msg.frame = "world";

        ugv_pub_AlanPlannerMsg.publish(ugv_alan_msg);

        //below for sfc
        camPose = ugvOdomPose * body_to_cam_Pose;

        set_total_bound(
            Eigen::Translation3d(
                camPose.translation().x(),
                camPose.translation().y(),
                camPose.translation().z()
            ),
            Eigen::Quaterniond(
                camPose.rotation()
            )
        );
        set_all_sfc(
            Eigen::Translation3d(
                camPose.translation().x(),
                camPose.translation().y(),
                camPose.translation().z()
            ),
            Eigen::Quaterniond(
                camPose.rotation()
            )
        );

        cam_pos_world = Eigen::Vector3d(
            camPose.translation().x(),
            camPose.translation().y(),
            camPose.translation().z()
        );
        
        //remember to add ugv camera translation                
        // alan_sfc_pub.publish(polyh_total_bound);
        // cout<<"publisher here..."<<polyh_array_pub_object.a_series_of_Corridor.size()<<endl;

        alan_all_sfc_pub.publish(polyh_array_pub_object);

    }
}

void alan::MsgSyncNodelet::ugv_vrpn_twist_callback(const geometry_msgs::TwistStamped::ConstPtr& twist)
{
    ugv_vrpn_twist = *twist;
    ugv_vrpn_twist_initiated = true;
}

void alan::MsgSyncNodelet::ugv_imu_callback(const sensor_msgs::Imu::ConstPtr& imu)
{
    ugv_acc_body(0) = imu->linear_acceleration.x; 
    ugv_acc_body(1) = imu->linear_acceleration.y;
    ugv_acc_body(2) = imu->linear_acceleration.z;

    ugv_imu_initiated = true;
}

//////////////////below defines sfc////////////////////////

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

    // cout<<v<<endl;
    // cout<<pt<<endl<<endl;

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
                

    Eigen::Vector2d cam2uav_vector = 
        Eigen::Vector2d(
            uav_pos_world.x() - cam_pos_world.x(),
            uav_pos_world.y() - cam_pos_world.y()
        );

    // cout<<ugv2uav_vector.norm()<<endl;


    // alan_visualization::Tangent plane_bound = set_plane_bound(
    //     cam_center_vector, 
    //     uav_pos_world + ugv2uav_vector                
    // );

    double cam2uav_xy_d = 0;

    if(cam2uav_vector.norm() > 2.5)
        cam2uav_xy_d = cam2uav_vector.norm() + 1.0;
    else
        cam2uav_xy_d = 2.5;


    Eigen::Vector3d temp_bound;
    temp_bound = t_current.translation() + q_rotate_vector(
        q_current, 
        Eigen::Vector3d(cam2uav_xy_d,0,0));

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
        t_current.translation() + q_rotate_vector(q_current,Eigen::Vector3d(final_corridor_length ,0,0))
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
        // Eigen::Vector3d(0,0,final_corridor_height));t_current.translation() +  
        Eigen::Vector3d(0,0,final_corridor_height + t_current.translation().z())
    );

    // cout<<temp_normal<<endl;
    // cout<<t_current.translation() +  Eigen::Vector3d(0,0,final_corridor_height)<<endl<<endl;
    
    polyhedron_temp.PolyhedronTangentArray.push_back(ceil_tangent);

    // cout<<ceil_tangent.
        
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
        t_current.translation() + q_rotate_vector(q_current, Eigen::Vector3d(final_corridor_length ,0,0))
    );

    
    //6th plane
    polyhedron_temp.PolyhedronTangentArray.push_back(temp_tangent);

    // cout<<polyhedron_temp.PolyhedronTangentArray.size()<<endl;


    polyh_array_pub_object.a_series_of_Corridor.push_back(polyhedron_temp);

    // for(auto what : polyh_array_pub_object.a_series_of_Corridor)
    // {
    //     cout<<what.PolyhedronTangentArray.size()<<endl;
    // }

}

void alan::MsgSyncNodelet::setup_camera_config(ros::NodeHandle& nh)
{
//param

    nh.getParam("/alan_master/cam_FOV_H", FOV_H);     
    nh.getParam("/alan_master/cam_FOV_V", FOV_V);   

    nh.getParam("/alan_master/final_corridor_height", final_corridor_height);
    nh.getParam("/alan_master/final_corridor_length", final_corridor_length);                          

//set sfc visualization
    FOV_H = FOV_H / 180 * M_PI * 0.75;
    FOV_V = FOV_V / 180 * M_PI * 0.75;        

    Eigen::Vector3d rpy_temp;

    rpy_temp = Eigen::Vector3d(0, -FOV_V/2, -FOV_H/2);                
    q1 = rpy2q(rpy_temp);
    cam_1axis_vector = q_rotate_vector(q1, cam_center_vector);

    rpy_temp = Eigen::Vector3d(0, -FOV_V/2, FOV_H/2);                
    q2 = rpy2q(rpy_temp);
    cam_2axis_vector = q_rotate_vector(q2, cam_center_vector);

    rpy_temp = Eigen::Vector3d(0, FOV_V/2, FOV_H/2);                
    q3 = rpy2q(rpy_temp); 
    cam_3axis_vector = q_rotate_vector(q3, cam_center_vector);

    rpy_temp = Eigen::Vector3d(0, FOV_V/2, -FOV_H/2);                
    q4 = rpy2q(rpy_temp);
    cam_4axis_vector = q_rotate_vector(q4, cam_center_vector);



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



    q_c2b = rpy2q(
        Eigen::Vector3d(
            c2b_ugv(3),
            c2b_ugv(4),
            c2b_ugv(5)
        )
    );

    t_c2b = Eigen::Translation3d(
        c2b_ugv(0),
        c2b_ugv(1),
        c2b_ugv(2)
    );

    body_to_cam_Pose = t_c2b * q_c2b;

}
