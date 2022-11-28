#include "./include/tools/essential.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>


static double c2b_x;
static double c2b_y;
static double c2b_z;

static double c2b_r;
static double c2b_p;
static double c2b_y;

void ugv_cam_callback(const geometry_msgs::PoseStamped::ConstPtr& ugv_pose, const geometry_msgs::PoseStamped::ConstPtr& cam_pose)
{
    


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;

    message_filters::Subscriber<geometry_msgs::PoseStamped> ugv_pose_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> cam_pose_sub;

    typedef message_filters::sync_policies::ApproximateTime
        <geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> camMysyncPolicy;
    typedef message_filters::Synchronizer<camMysyncPolicy> camsync;
    boost::shared_ptr<camsync> camsync_;

    ugv_pose_sub.subscribe(nh, "/vrpn_client_node/gh034_car/pose", 1);
    cam_pose_sub.subscribe(nh, "/vrpn_client_node/gh034_camera/pose", 1);

    camsync_.reset(new camsync(camMysyncPolicy(10), ugv_pose_sub, cam_pose_sub));
    camsync_->registerCallback(boost::bind(&ugv_cam_callback, _1, _2));

    return 0;
}