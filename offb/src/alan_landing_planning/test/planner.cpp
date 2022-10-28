#ifndef PLAN_H
#define PLAN_H
#include "include/planner.h"

void* alan::PlannerNodelet::PubMainLoop(void* tmp)
{
    PlannerNodelet* pub = (PlannerNodelet*) tmp;

    ros::Rate loop_rate(50);

    while (ros::ok()) 
    {        
        ros::spinOnce();
        loop_rate.sleep();
    }

    void* result;

    return result;
}


void alan::PlannerNodelet::uavStateMachineCallback(const alan::StateMachine::ConstPtr& msg)
{
    uav_fsm = *msg;

    cout<<uav_fsm.finite_state_machine<<endl;
}

void alan::PlannerNodelet::traj_setup(
    alan_traj::endpt start_pt,
    alan_traj::endpt end_pt,
    vector<alan_traj::corridor> cube_list
    )
{
    start = start_pt;
    end = end_pt;

    double dis = 
          pow(end_pt.posi.x - start_pt.posi.x, 2)
        + pow(end_pt.posi.y - start_pt.posi.y, 2)
        + pow(end_pt.posi.z - start_pt.posi.z, 2);

    dis = sqrt(dis);

    double t = (dis / vel_avg) / m;

    trajseg_t.clear();

    for(int i = 0; i < m; i++)
    {
        trajseg_t.push_back(t);
    }


}


#endif