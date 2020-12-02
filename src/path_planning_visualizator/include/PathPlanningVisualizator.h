/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <fstream>
#include <vector>

constexpr double CYLINDER_SCALING_FACTOR = .7f;
constexpr double CUBE_SCALING_FACTOR = .4f;
constexpr double LINE_SCALING_FACTOR = .2f;

class PathPlanningVisualizator
{
struct FPoint2D
{
    FPoint2D() { x = 0.; y = 0.; }
    FPoint2D(double X, double Y) {x = X; y = Y;}
    geometry_msgs::Point GetPoint(float scalingFactor) const;
    double x;
    double y;
};

public:
    PathPlanningVisualizator();

    void SetPublisher(ros::Publisher publisher) { m_publisher = publisher; }
    void Do();
    void Init(const std::string &conesFile, const std::string &pathFile, float scalingFactor);
    void DeleteAll();

private:
    ros::Publisher m_publisher;
    float m_scalingFactor = 1.f;
    std::string m_conesFilePath = "";
    std::string m_pathFilePath = "";
    std::vector<FPoint2D> m_cones;
    std::vector<FPoint2D> m_path;

    visualization_msgs::MarkerArray m_markerArray;

    void ReadCsvFile(const std::string &pathToFile, std::vector<FPoint2D> &points) const;
    void FillMarkerArray();
};
  