/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/SensorsVisualizator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensorsVisualizator");
    ros::NodeHandle handle;
    SensorsVisualizator visualizatorObj;

    ros::Publisher cameraPub = handle.advertise<visualization_msgs::MarkerArray>("camera_visualize",1);
    ros::Publisher lidarPub = handle.advertise<visualization_msgs::MarkerArray>("lidar_visualize",1);
    ros::Publisher fusionPub = handle.advertise<visualization_msgs::Marker>("fusion_visualize",1);
    visualizatorObj.SetPublishers(cameraPub, lidarPub, fusionPub);

    std::string cameraFrameId;
    handle.param("/camera_frame", cameraFrameId, std::string("camera_left"));

    std::string lidarFrameId;
    handle.param("/lidar_frame", lidarFrameId, std::string("lidar"));
   
    std::string fusionFrameId;
    handle.param("/fixed_frame", fusionFrameId, std::string("base_link"));
    visualizatorObj.SetFrames(cameraFrameId, lidarFrameId, fusionFrameId);

    ros::Subscriber cameraSub = handle.subscribe("/camera_cones", 1, &SensorsVisualizator::DoCamera, &visualizatorObj);
    ros::Subscriber lidarSub = handle.subscribe("/lidar_cones", 1, &SensorsVisualizator::DoLidar, &visualizatorObj);
    ros::Subscriber fusionSub = handle.subscribe("/fusion_cones", 1, &SensorsVisualizator::DoFusion, &visualizatorObj);

    ros::spin();

    return 0;
}
