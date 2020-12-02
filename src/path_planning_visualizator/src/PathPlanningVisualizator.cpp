/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#include "../include/PathPlanningVisualizator.h"

geometry_msgs::Point PathPlanningVisualizator::FPoint2D::GetPoint(float scalingFactor) const
{
    geometry_msgs::Point point;

    point.x = x * static_cast<double>(scalingFactor);
    point.y = y * static_cast<double>(scalingFactor);
    point.z = 0.;

    return point;
}

PathPlanningVisualizator::PathPlanningVisualizator()
{ 
}

void PathPlanningVisualizator::Init(const std::string &conesFile, const std::string &pathFile, float scalingFactor)
{
    m_scalingFactor = scalingFactor;
    m_conesFilePath = conesFile;
    m_pathFilePath = pathFile;
    m_cones.reserve(200);
    m_path.reserve(200);
    m_markerArray.markers.reserve(400);
}

void PathPlanningVisualizator::Do()
{
    m_markerArray.markers.clear();
    ReadCsvFile(m_conesFilePath, m_cones);
    ReadCsvFile(m_pathFilePath, m_path);
 
    FillMarkerArray();
    m_publisher.publish(m_markerArray);
}

void PathPlanningVisualizator::ReadCsvFile(const std::string &pathToFile, std::vector<FPoint2D> &points) const
{
    std::ifstream in(pathToFile);
    points.clear();

    if (!in.is_open()) return;
    double x = 0., y = 0.;
    char comma;

    while(!in.eof())
    {
        in >> x;
        in >> comma;
        in >> y;

        points.emplace_back(x, y);
    }

    in.close();
}

void PathPlanningVisualizator::FillMarkerArray()
{
    for(size_t i = 0; i < m_cones.size(); ++i)
    {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "/sgt_frame";
        marker.ns = "coneGeometry";
        marker.id = i;
        marker.action = visualization_msgs::Marker::ADD;

        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.scale.x = CYLINDER_SCALING_FACTOR;
        marker.scale.y = CYLINDER_SCALING_FACTOR;
        marker.scale.z = CYLINDER_SCALING_FACTOR;

        marker.color.a = 1.f;
        marker.color.r = 0.f;
        marker.color.g = .7f;
        marker.color.b = 0.f;

        marker.pose.position = m_cones[i].GetPoint(m_scalingFactor);
        marker.pose.orientation.w = 1.f;
        marker.pose.orientation.x = 0.f;
        marker.pose.orientation.y = 0.f;
        marker.pose.orientation.z = 0.f;

        m_markerArray.markers.push_back(marker);
    }

    for(size_t i = 0; i < m_path.size(); ++i)
    {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "/sgt_frame";
        marker.ns = "pathGeometry";
        marker.id = i;
        marker.action = visualization_msgs::Marker::ADD;

        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = CUBE_SCALING_FACTOR;
        marker.scale.y = CUBE_SCALING_FACTOR;
        marker.scale.z = CUBE_SCALING_FACTOR;

        marker.color.a = 1.f;
        marker.color.r = 0.7f;
        marker.color.g = 0.f;
        marker.color.b = 0.f;

        marker.pose.position = m_path[i].GetPoint(m_scalingFactor);
        marker.pose.orientation.w = 1.f;
        marker.pose.orientation.x = 0.f;
        marker.pose.orientation.y = 0.f;
        marker.pose.orientation.z = 0.f;

        m_markerArray.markers.push_back(marker);
    }    

    visualization_msgs::Marker marker;

    marker.header.frame_id = "/sgt_frame";
    marker.ns = "lineGeometry";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;

    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = LINE_SCALING_FACTOR;

    marker.color.a = 1.f;
    marker.color.r = 0.7f;
    marker.color.g = 0.f;
    marker.color.b = 0.f;

    marker.points.reserve(m_path.size());

    for(size_t i = 0; i < m_path.size(); ++i)
    {
        marker.points.push_back(m_path[i].GetPoint(m_scalingFactor));
    }

    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;

    marker.pose.orientation.w = 1.f;
    marker.pose.orientation.x = 0.f;
    marker.pose.orientation.y = 0.f;
    marker.pose.orientation.z = 0.f;

    m_markerArray.markers.push_back(marker);
}

void PathPlanningVisualizator::DeleteAll()
{
    m_markerArray.markers.clear();

    visualization_msgs::Marker marker1, marker2;

    marker1.header.frame_id = "/sgt_frame";
    marker1.ns = "pathGeometry";
    marker1.id = 0;
    marker1.action = visualization_msgs::Marker::DELETEALL;
    m_markerArray.markers.push_back(marker1);

    marker2.header.frame_id = "/sgt_frame";
    marker2.ns = "coneGeometry";
    marker2.id = 0;
    marker2.action = visualization_msgs::Marker::DELETEALL;
    m_markerArray.markers.push_back(marker2);

    m_publisher.publish(m_markerArray);
}