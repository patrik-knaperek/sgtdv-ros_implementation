/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/SLAMSimInterface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slamSimInterface");
    ros::NodeHandle handle;

    SLAMSimInterface simInterface;

    ros::Publisher mapPub = handle.advertise<sgtdv_msgs::ConeArr>("slam_map", 1, true);
    ros::Publisher posePub = handle.advertise<sgtdv_msgs::CarPose>("slam_pose", 1);
    //ros::Publisher velPub = handle.advertise<sgtdv_msgs::CarVel>("velocity_estimate", 1);
    simInterface.setPublishers(mapPub, posePub/*, velPub*/);

    ros::Subscriber mapSub = handle.subscribe("estimation/slam/map", 1, &SLAMSimInterface::DoMap, &simInterface);
    ros::Subscriber poseSub = handle.subscribe("estimation/slam/state", 1, &SLAMSimInterface::DoState, &simInterface);
    
    ros::Rate rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        simInterface.PublishMap();
        rate.sleep();
    }

    return 0;
}