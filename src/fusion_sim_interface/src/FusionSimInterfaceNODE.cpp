/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/FusionSimInterface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion_sim_interface");
    ros::NodeHandle handle;
    
    SimInterface simInterface;

    ros::Subscriber cameraSub = handle.subscribe("fssim/camera/cones", 1, &SimInterface::DoCamera, &simInterface);
    ros::Subscriber lidarSub =  handle.subscribe("fssim/lidar/cones", 1, &SimInterface::DoLidar, &simInterface);
    
    ros::Publisher lidarPub = handle.advertise<sgtdv_msgs::Point2DArr>("lidar_cones", 1);
    ros::Publisher cameraPub = handle.advertise<sgtdv_msgs::ConeArr>("camera_cones", 1);

    simInterface.setPublishers(cameraPub, lidarPub);

    ros::spin();

    return 0;
}