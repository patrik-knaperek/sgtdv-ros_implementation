/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#include <ros/ros.h>
#include "../include/PathPlanningVisualizator.h"
#include <visualization_msgs/Marker.h>
#include <string>

int main(int argc, char** argv)
{
    PathPlanningVisualizator visualizator;

    if (argc != 4)
    {
        std::cout << "Not enough parameters\n";
        return 1;
    }

    std::string coneFilePath(argv[1]);
    std::string pathFilePath(argv[2]);
    float scalingFactor = atof(argv[3]);

    ros::init(argc, argv, "pathPlanningVisualizator");
    ros::NodeHandle handle;

    visualizator.Init(coneFilePath, pathFilePath, scalingFactor);

    ros::Publisher publisher = handle.advertise<visualization_msgs::MarkerArray>("path_planning_visualization", 1);
    visualizator.SetPublisher(publisher);

    while (ros::ok())
    {
        visualizator.Do();
        getchar();
        visualizator.DeleteAll();     
    }    

    return 0;
}
