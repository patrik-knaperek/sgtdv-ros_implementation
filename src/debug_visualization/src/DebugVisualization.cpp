/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#include "../include/DebugVisualization.h"

const DebugVisualization::NodeGeometry DebugVisualization::m_NODE_GEOMETRY[7] = 
{
    NodeGeometry(FPoint2D(0.f + X_OFFSET, 0.f + Y_OFFSET), 2.f, 1.f),         //camera
    NodeGeometry(FPoint2D(0.f + X_OFFSET, -2.f + Y_OFFSET), 2.f, 1.f),       //lidar
    NodeGeometry(FPoint2D(4.f + X_OFFSET, -1.f + Y_OFFSET), 2.f, 1.f),      //fusion
    NodeGeometry(FPoint2D(4.f + X_OFFSET, -3.f + Y_OFFSET), 2.f, 1.f),      //slam
    NodeGeometry(FPoint2D(4.f + X_OFFSET, -5.f + Y_OFFSET), 2.f, 1.f),      //pathPlanning
    NodeGeometry(FPoint2D(4.f + X_OFFSET, -7.f + Y_OFFSET), 2.f, 1.f),      //pathTracking
    NodeGeometry(FPoint2D(4.f + X_OFFSET, -9.f + Y_OFFSET), 2.f, 1.f)       //jetsonCanInterface
};

DebugVisualization::NodeGeometry::NodeGeometry(const FPoint2D &position, float scaleX, float scaleY)
{
    this->position = position;
    this->scaleX = scaleX;
    this->scaleY = scaleY;
}

geometry_msgs::Point DebugVisualization::FPoint2D::GetPoint() const
{
    geometry_msgs::Point point;

    point.x = x;
    point.y = y;
    point.z = 0.;

    return point;
}

DebugVisualization::DebugVisualization()
{
    InitNodes();
}

void DebugVisualization::InitRViz()
{
    InitConnectionLines();
    PublishAllNodes();
}

void DebugVisualization::InitConnectionLines()
{
    visualization_msgs::Marker lineConnections[NUM_OF_CONNECTION_LINES];

    for(int32_t i = 0; i < NUM_OF_CONNECTION_LINES; i++)
    {
        lineConnections[i].header.frame_id = FRAME_ID;
        lineConnections[i].header.stamp = ros::Time::now();
        lineConnections[i].ns = LINE_CONNECTION_NAMESPACE;
        lineConnections[i].id = i;
        lineConnections[i].action = visualization_msgs::Marker::ADD;

        lineConnections[i].type = visualization_msgs::Marker::ARROW;
        lineConnections[i].scale.x = 0.08f;
        lineConnections[i].scale.y = 0.08f;
        lineConnections[i].scale.z = 0.5f;  

        lineConnections[i].color.a = 1.f;
        lineConnections[i].color.r = 1.f;
        lineConnections[i].color.g = 1.f;
        lineConnections[i].color.b = 1.f;

        lineConnections[i].pose.position = FPoint2D(0.f, 0.f).GetPoint();

        lineConnections[i].points.reserve(2);
        lineConnections[i].points.push_back(m_NODE_GEOMETRY[i].position.GetPoint());
    }

    lineConnections[CAMERA].points.push_back(m_NODE_GEOMETRY[FUSION].position.GetPoint());   
    lineConnections[LIDAR].points.push_back(m_NODE_GEOMETRY[FUSION].position.GetPoint());    
    lineConnections[FUSION].points.push_back(m_NODE_GEOMETRY[SLAM].position.GetPoint());
    lineConnections[SLAM].points.push_back(m_NODE_GEOMETRY[PATH_PLANNING].position.GetPoint());   
    lineConnections[PATH_PLANNING].points.push_back(m_NODE_GEOMETRY[PATH_TRACKING].position.GetPoint());
    lineConnections[PATH_TRACKING].points.push_back(m_NODE_GEOMETRY[JETSON_CAN_INTERFACE].position.GetPoint());

    for(int32_t i = 0; i < NUM_OF_CONNECTION_LINES; i++)
    {
        m_publisher.publish(lineConnections[i]);
    }
}

void DebugVisualization::InitNodes()
{
    for(int32_t i = 0; i < NUM_OF_NODES; i++)
    {
        m_markers[i].header.frame_id = FRAME_ID;
        m_markers[i].ns = NODE_RECT_NAMESPACE;
        m_markers[i].id = i;
        m_markers[i].action = visualization_msgs::Marker::ADD;

        m_markers[i].type = visualization_msgs::Marker::CUBE;
        m_markers[i].scale.x = m_NODE_GEOMETRY[i].scaleX;
        m_markers[i].scale.y = m_NODE_GEOMETRY[i].scaleY;
        m_markers[i].scale.z = 0.1f;

        m_markers[i].color.a = 1.f;
        m_markers[i].color.r = 0.3f;
        m_markers[i].color.g = 0.3f;
        m_markers[i].color.b = 0.3f;

        m_markers[i].pose.position = m_NODE_GEOMETRY[i].position.GetPoint();
        // m_markers[i].pose.orientation.w = 1.f;
        // m_markers[i].pose.orientation.x = 1.f;
        // m_markers[i].pose.orientation.y = 0.f;
        // m_markers[i].pose.orientation.z = 0.f;
    }
}

void DebugVisualization::PublishAllNodes()
{
    for(int32_t i = 0; i < NUM_OF_NODES; i++)
    {
        m_markers[i].header.stamp = ros::Time::now();
        m_publisher.publish(m_markers[i]);
    }
}