/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#include "../include/DebugVisualization.h"

DebugVisualization::NodeVisualization::NodeVisualization(const FPoint2D &position, float width, float height)
{
    this->position = position;
    this->width = width;
    this->height = height;
    inConnection.x = position.x;
    inConnection.y = position.y + height / 2.f;
    outConnection.x = position.x + width;
    outConnection.y = position.y + height / 2.f;
}

void DebugVisualization::InitRViz()
{
    visualization_msgs::Marker lineConnections

}