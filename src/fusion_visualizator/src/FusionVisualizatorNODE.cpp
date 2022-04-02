/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/FusionVisualizator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion_visualizator");
    ros::NodeHandle handle;
    FusionVisualizator visualizatorObj;

    ros::Publisher cameraPub = handle.advertise<visualization_msgs::MarkerArray>("camera_visualize",1);
    ros::Publisher lidarPub = handle.advertise<visualization_msgs::MarkerArray>("lidar_visualize",1);
    ros::Publisher fusionPub = handle.advertise<visualization_msgs::Marker>("fusion_visualize",1);
    visualizatorObj.SetPublishers(cameraPub, lidarPub, fusionPub);

    std::string cameraFrameId;
    if(!handle.getParam("/camera_frame", cameraFrameId))
        ROS_ERROR("Failed to get parameter from server\n");

    std::string lidarFrameId;
    if(!handle.getParam("/lidar_frame", lidarFrameId))
        ROS_ERROR("Failed to get parameter from server\n");

    std::string fusionFrameId;
    if(!handle.getParam("/fixed_frame", fusionFrameId))
        ROS_ERROR("Failed to get parameter from server\n");
    visualizatorObj.SetFrames(cameraFrameId, lidarFrameId, fusionFrameId);

    ros::Subscriber cameraSub = handle.subscribe("/camera_cones", 1, &FusionVisualizator::DoCamera, &visualizatorObj);
    ros::Subscriber lidarSub = handle.subscribe("/lidar_cones", 1, &FusionVisualizator::DoLidar, &visualizatorObj);
    ros::Subscriber fusionSub = handle.subscribe("/fusion_cones", 1, &FusionVisualizator::DoFusion, &visualizatorObj);

    ros::spin();

    return 0;
}