/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#include <ros/ros.h>
#include "../include/DebugVisualization.h"
#include <visualization_msgs/Marker.h>
#include <thread>
#include <chrono>

constexpr int FPS = 120;
constexpr int TIME_PER_FRAME_MS = 1000 / FPS;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "debugVisualization");
    ros::NodeHandle handle;

    DebugVisualization debugVisualization;

    ros::Publisher publisher = handle.advertise<visualization_msgs::MarkerArray>("debug_visualization_out", 1);
    debugVisualization.SetPublisher(publisher);

    ros::Subscriber cameraSub = handle.subscribe("camera_cone_detection_debug_state", 2, &DebugVisualization::DoCamera, &debugVisualization);
    ros::Subscriber lidarSub = handle.subscribe("lidar_cone_detection_debug_state", 2, &DebugVisualization::DoLidar, &debugVisualization);
    ros::Subscriber fusionSub = handle.subscribe("fusion_debug_state", 2, &DebugVisualization::DoFusion, &debugVisualization);
    ros::Subscriber slamSub = handle.subscribe("slam_debug_state", 2, &DebugVisualization::DoSLAM, &debugVisualization);
    ros::Subscriber pathPlanningSub = handle.subscribe("pathplanning_debug_state", 2, &DebugVisualization::DoPathPlanning, &debugVisualization);
    ros::Subscriber pathTrackingSub = handle.subscribe("pathtracking_debug_state", 2, &DebugVisualization::DoPathTracking, &debugVisualization);
    ros::Subscriber jetsonCANInterface = handle.subscribe("jetson_can_interface_debug_state", 2, &DebugVisualization::DoJetsonCANInterface, &debugVisualization);

    ros::Rate loop_rate(FPS);
    
    while (ros::ok())
    {
        debugVisualization.PublishEverythingAsArray();
        
        ros::spinOnce();
        loop_rate.sleep();
    }    

    return 0;
}
