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

    ros::Publisher cmdPub = handle.advertise<fsd_common_msgs::ControlCommand>("sgt/control_command",1); // this topic name has to be set in fssim_interface/config/config.yaml as parameter fsd/cmd to obtain SGT control commands by FSSIM
    ros::Publisher trajectoryPub = handle.advertise<sgtdv_msgs::Point2DArr>("pathplanning_trajectory",1,true);
    ros::Publisher posePub = handle.advertise<sgtdv_msgs::CarPose>("pose_estimate",1);
    simInterface.setPublishers(cmdPub, trajectoryPub, posePub);

    ros::Subscriber cmdSub = handle.subscribe("pathtracking_commands", 1, &PathTrackingSimInterface::DoCmd, &simInterface);
    ros::Subscriber trajectorySub = handle.subscribe("control/pure_pursuit/center_line", 1, &PathTrackingSimInterface::DoTrajectory, &simInterface);
    ros::Subscriber poseSub = handle.subscribe("estimation/slam/state", 1, &PathTrackingSimInterface::DoPose, &simInterface);

    ros::spin();

    return 0;
}