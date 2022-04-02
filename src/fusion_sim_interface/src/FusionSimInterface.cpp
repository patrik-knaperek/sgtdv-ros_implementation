/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/


#include "../include/FusionSimInterface.h"

SimInterface::SimInterface()
{
    
}

SimInterface::~SimInterface()
{

}

void SimInterface::DoLidar(const sensor_msgs::PointCloud2::ConstPtr &msg)
{   
    int pointsCount = msg->width;
    sgtdv_msgs::Point2DArrPtr lidarCones(new sgtdv_msgs::Point2DArr);
    lidarCones->points.reserve(pointsCount);

    float const *temp;

    for(int i = 0; i < pointsCount; i++)
    {
        temp = reinterpret_cast<const float*>(&msg->data[i*msg->point_step]);
        sgtdv_msgs::Point2D cone = sgtdv_msgs::Point2D();

        cone.x = *temp;
        cone.y = *(temp + 1);

        lidarCones->points.push_back(cone);
    }

    m_lidarPublisher.publish(lidarCones);
}

void SimInterface::DoCamera(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    int conesCount = msg->width;
    sgtdv_msgs::ConeArrPtr cameraCones(new sgtdv_msgs::ConeArr);
    cameraCones->cones.reserve(conesCount);
    
    float const *temp;

    for(int i = 0; i < conesCount; i++)
    {
        temp = reinterpret_cast<const float*>(&msg->data[i*msg->point_step]);
        sgtdv_msgs::Cone cone;

        cone.coords.x = *temp;
        cone.coords.y = *(temp + 1);

        if(*(temp + 9) > 0.85)
            cone.color = 'b';       // blue cone
        else if(*(temp + 10) > 0.85)
            cone.color = 'y';       // yellow cone
        else if(*(temp + 11) > 0.85)
            cone.color = 'g';       // orange cone big
        else cone.color = 'u';      // unknown color

        cameraCones->cones.push_back(cone);
    }
    
    m_cameraPublisher.publish(cameraCones);
}