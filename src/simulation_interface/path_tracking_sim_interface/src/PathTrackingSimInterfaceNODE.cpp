/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/PathTrackingSimInterface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathTrackingSimInterface");
    ros::NodeHandle handle;

    PathTrackingSimInterface simInterface;

    ros::Publisher cmdPub = handle.advertise<fsd_common_msgs::ControlCommand>("sgt/control_command",10); // this topic name has to be set in fssim_interface/config/config.yaml as parameter fsd/cmd to obtain SGT control commands by FSSIM
    ros::Publisher trajectoryPub = handle.advertise<sgtdv_msgs::Point2DArr>("pathplanning_trajectory",10,true);
    ros::Publisher posePub = handle.advertise<sgtdv_msgs::CarPose>("pose_estimate",10);
    ros::Publisher velPub = handle.advertise<sgtdv_msgs::CarVel>("velocity_estimate",10);
    simInterface.setPublishers(cmdPub, trajectoryPub, posePub, velPub);

    ros::Subscriber cmdSub = handle.subscribe("pathtracking_commands", 10, &PathTrackingSimInterface::DoCmd, &simInterface);
    ros::Subscriber trajectorySub = handle.subscribe("control/pure_pursuit/center_line", 10, &PathTrackingSimInterface::DoTrajectory, &simInterface);
    ros::Subscriber poseSub = handle.subscribe("estimation/slam/state", 10, &PathTrackingSimInterface::DoPose, &simInterface);
    ros::Subscriber velSub = handle.subscribe("estimation/velocity_estimation/velocity_estimation", 10, &PathTrackingSimInterface::DoVelocity, &simInterface);

    ros::spin();

    return 0;
}