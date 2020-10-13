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

    ros::Subscriber cameraSub = handle.subscribe("camera_cone_detection_debug_state", 1, &DebugVisualization::DoCamera, &debugVisualization);
    ros::Subscriber lidarSub = handle.subscribe("lidar_cone_detection_debug_state", 1, &DebugVisualization::DoLidar, &debugVisualization);
    ros::Subscriber fusionSub = handle.subscribe("fusion_debug_state", 1, &DebugVisualization::DoFusion, &debugVisualization);
    ros::Subscriber slamSub = handle.subscribe("slam_debug_state", 1, &DebugVisualization::DoSLAM, &debugVisualization);
    ros::Subscriber pathPlanningSub = handle.subscribe("path_planning_debug_state", 1, &DebugVisualization::DoPathPlanning, &debugVisualization);
    ros::Subscriber pathTrackingSub = handle.subscribe("path_tracking_debug_state", 1, &DebugVisualization::DoPathTracking, &debugVisualization);
    ros::Subscriber jetsonCANInterface = handle.subscribe("jetson_can_interface_debug_state", 1, &DebugVisualization::DoJetsonCANInterface, &debugVisualization);

    //ros::spin();

    while(ros::ok())
    {
        debugVisualization.PublishEverything();
    }

    return 0;
}
