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
#ifdef SGT_DEBUG_STATE
    sgtdv_msgs::DebugState state;
    state.workingState = 1;
    m_visLidarDebugPublisher.publish(state);
#endif    
    
    int pointsCount = msg->width;
    sgtdv_msgs::Point2DStampedArrPtr lidarCones(new sgtdv_msgs::Point2DStampedArr);
    lidarCones->points.reserve(pointsCount);

    float const *temp;

    for(int i = 0; i < pointsCount; i++)
    {
        temp = reinterpret_cast<const float*>(&msg->data[i*msg->point_step]);
        sgtdv_msgs::Point2DStamped cone = sgtdv_msgs::Point2DStamped();

        cone.header = msg->header;
        cone.x = *temp;
        cone.y = *(temp + 1);

        lidarCones->points.push_back(cone);
    }

    m_lidarPublisher.publish(lidarCones);

#ifdef SGT_DEBUG_STATE
    state.numOfCones = lidarCones->points.size();
    state.workingState = 0;
	m_visLidarDebugPublisher.publish(state);
#endif
}

void SimInterface::DoCamera(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
#ifdef SGT_DEBUG_STATE
    sgtdv_msgs::DebugState state;
    state.workingState = 1;
    m_visCameraDebugPublisher.publish(state);
#endif
    
    int conesCount = msg->width;
    sgtdv_msgs::ConeStampedArrPtr cameraCones(new sgtdv_msgs::ConeStampedArr);
    cameraCones->cones.reserve(conesCount);
    
    float const *temp;

    for(int i = 0; i < conesCount; i++)
    {
        temp = reinterpret_cast<const float*>(&msg->data[i*msg->point_step]);
        sgtdv_msgs::ConeStamped cone;

        cone.coords.header = msg->header;
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

#ifdef SGT_DEBUG_STATE
    state.numOfCones = cameraCones->cones.size();
    state.workingState = 0;
	m_visCameraDebugPublisher.publish(state);
#endif
}
