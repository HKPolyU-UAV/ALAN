#include "include/planner_server.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh("~");

    planner_server alan_planner_server(nh, 50);
    alan_planner_server.mainserver();

    return 0;
}