#include "include/plan.h"

void* alan::PlannerNodelet::PubMainLoop(void* tmp)
{
    PlannerNodelet* pub = (PlannerNodelet*) tmp;

    ros::Rate loop_rate(50);
    while (ros::ok()) 
    {
        // ROS_INFO("%d,publish!", num++);
        pub->pub_traj_pos.publish(pub->uav_traj_pose);

        ros::spinOnce();
        loop_rate.sleep();
    }

    void* result;

    return result;
}
