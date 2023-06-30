/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/SensorsVisualizator.h"

SensorsVisualizator::SensorsVisualizator()
{
    m_cameraMarkers.markers.reserve(20);
    m_lidarMarkers.markers.reserve(1000);
    
    m_fusionMarkers.markers.reserve(2);
    visualization_msgs::Marker marker;
    marker.type = marker.POINTS;
    marker.action = marker.MODIFY;
    marker.ns = std::string("fusion");
    marker.lifetime = ros::Duration(0.2);
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.points.reserve(20);
    m_fusionMarkers.markers.push_back(marker);

#ifdef SIMPLE_FUSION
    visualization_msgs::Marker simpleMarker;
    simpleMarker.type = simpleMarker.POINTS;
    simpleMarker.action = simpleMarker.MODIFY;
    simpleMarker.ns = std::string("fusion_simple");
    simpleMarker.lifetime = ros::Duration(0.2);
    simpleMarker.scale.x = 0.1;
    simpleMarker.scale.y = 0.1;
    simpleMarker.color.a = 1.0;
    simpleMarker.points.reserve(20);
    m_fusionMarkers.markers.push_back(simpleMarker);
#endif

}

SensorsVisualizator::~SensorsVisualizator()
{

}

void SensorsVisualizator::DoCamera(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg)
{
    DeleteMarkers(m_cameraMarkers, m_cameraPublisher);
    m_cameraMarkers.markers.clear();
    
    for(int i = 0; i < msg->cones.size(); i++)
    {
        if (std::isnan(msg->cones[i].coords.x) || std::isnan(msg->cones[i].coords.y))
            continue;
        
        visualization_msgs::Marker marker;
        
        marker.header = msg->cones[i].coords.header;
        marker.type = marker.SPHERE;
        marker.action = marker.ADD;
        marker.id = i;
        marker.lifetime = ros::Duration(0.5);
        marker.pose.position.x = msg->cones[i].coords.x;
        marker.pose.position.y = msg->cones[i].coords.y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
       
        marker.color.a = 0.6;
        
        if (msg->cones[i].color == 'b') // blue cone
        {          
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        else if (msg->cones[i].color == 'y') // yellow cone
        {          
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else if (msg->cones[i].color == 's') // orange cone small
        {          
            marker.color.r = 1.0;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
        }
        else if (msg->cones[i].color == 'g') // orange cone big
        {
            marker.color.r = 1.0;
            marker.color.g = 0.3;
            marker.color.b = 0.0;
        }
        else
        {
            marker.color.r = marker.color.g = marker.color.b = 0;
        }
        
        m_cameraMarkers.markers.push_back(marker);
    }
    
    m_cameraPublisher.publish(m_cameraMarkers);  
}

void SensorsVisualizator::DoLidar(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg)
{
    DeleteMarkers(m_lidarMarkers, m_lidarPublisher);
    m_lidarMarkers.markers.clear();
    visualization_msgs::Marker marker;
    
    for (int i = 0; i< msg->points.size(); i++)
    {
        marker.header = msg->points[i].header;
        marker.type = marker.CYLINDER;
        marker.action = marker.ADD;
        marker.id = i;
        marker.lifetime = ros::Duration(0.5);
        marker.pose.position.x = msg->points[i].x;
        marker.pose.position.y = msg->points[i].y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 0.7;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        
        m_lidarMarkers.markers.push_back(marker);
    }

    m_lidarPublisher.publish(m_lidarMarkers); 
}

void SensorsVisualizator::DoFusion(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg)
{
    m_fusionMarkers.markers[0].points.clear();
    m_fusionMarkers.markers[0].colors.clear();
    
    if (msg->cones.size() > 0)
    {
        m_fusionMarkers.markers[0].header = msg->cones[0].coords.header;
    }
    
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;

    for(int i = 0; i < msg->cones.size(); i++)
    {
        point.x = msg->cones[i].coords.x;
        point.y = msg->cones[i].coords.y;
        point.z = 0.0;

        color.a = 1.0;
        if (msg->cones[i].color == 'b') // blue cone
        {          
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
        }
        else if (msg->cones[i].color == 'y') // yellow cone
        {          
            color.r = 1.0;
            color.g = 1.0;
            color.b = 0.0;
        }
        else if (msg->cones[i].color == 's') // orange cone small
        {          
            color.r = 1.0;
            color.g = 0.5;
            color.b = 0.0;
        }
        else if (msg->cones[i].color == 'g') // orange cone big
        {
            color.r = 1.0;
            color.g = 0.3;
            color.b = 0.0;
        }
        else
        {
            color.r = color.g = color.b = 0;
        }
        
        m_fusionMarkers.markers[0].points.push_back(point);
        m_fusionMarkers.markers[0].colors.push_back(color);
    }
    
    m_fusionPublisher.publish(m_fusionMarkers);
}

#ifdef SIMPLE_FUSION
    void SensorsVisualizator::DoSimpleFusion(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg)
    {
        m_fusionMarkers.markers[1].points.clear();
        m_fusionMarkers.markers[1].colors.clear();

        if (msg->cones.size() > 0)
        {
            m_fusionMarkers.markers[1].header = msg->cones[0].coords.header;
        }
        
        geometry_msgs::Point point;
        std_msgs::ColorRGBA color;

        for(int i = 0; i < msg->cones.size(); i++)
        {
            point.x = msg->cones[i].coords.x;
            point.y = msg->cones[i].coords.y;
            point.z = 0.0;

            color.a = 1.0;
            color.r = 0.9;
            color.g = 0.6;
            color.b = 0.9;

            m_fusionMarkers.markers[1].points.push_back(point);
            m_fusionMarkers.markers[1].colors.push_back(color);
        }

        m_fusionPublisher.publish(m_fusionMarkers);
    }
#endif

void SensorsVisualizator::DeleteMarkers(visualization_msgs::MarkerArray markerArray, 
ros::Publisher publisher)
{
    markerArray.markers.clear();
    visualization_msgs::Marker marker;

    marker.id = 0;
    marker.action = marker.DELETEALL;
    markerArray.markers.push_back(marker);

    publisher.publish(markerArray);
}
                        
