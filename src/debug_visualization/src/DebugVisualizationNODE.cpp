/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include "../include/DebugVisualization.h"
#include <visualization_msgs/Marker.h>
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
    DebugVisualization debugVisualization;

    ros::init(argc, argv, "debugVisualization");
    ros::NodeHandle handle;

    ros::Publisher publisher = handle.advertise<visualization_msgs::Marker>("debug_visualization_out", 1);
    debugVisualization.SetPublisher(publisher);

   // ros::Subscriber cameraSub = handle.subscribe("camera_cones", 1, &FusionSynch::DoCamera, &synchObj);
    //ros::Subscriber lidarSub = handle.subscribe("lidar_cones", 1, &FusionSynch::DoLidar, &synchObj);

    ros::spin();

    return 0;
}
