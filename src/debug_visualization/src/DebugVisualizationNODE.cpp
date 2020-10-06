/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include "../include/DebugVisualization.h"
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
    DebugVisualization debugVisualization;

    ros::init(argc, argv, "debugVisualization");
    ros::NodeHandle handle;

    ros::Publisher publisher = handle.advertise<visualization_msgs::Marker>("debug_visualization_out", 1);

    //ros::Subscriber cameraSub = handle.subscribe("camera_cones", 1, &FusionSynch::DoCamera, &synchObj);
   // ros::Subscriber lidarSub = handle.subscribe("lidar_cones", 1, &FusionSynch::DoLidar, &synchObj);

    //debugVisualization.InitRViz();

    ros::spin();

    return 0;
}
