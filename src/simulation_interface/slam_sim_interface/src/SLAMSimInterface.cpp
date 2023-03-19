/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/SLAMSimInterface.h"

SLAMSimInterface::SLAMSimInterface()
{

}

void SLAMSimInterface::DoMap(const fsd_common_msgs::Map::ConstPtr &msg) const
{
    sgtdv_msgs::ConeArrPtr map(new sgtdv_msgs::ConeArr);
    map->cones.reserve(msg->cone_blue.size() + msg->cone_yellow.size() + msg->cone_orange.size());
    
    for (auto &i : msg->cone_blue)
    {
        sgtdv_msgs::Cone cone;
        cone.coords.x = i.position.x;
        cone.coords.y = i.position.y;
        cone.color = 'b';
        map->cones.emplace_back(cone);
    }

    for (auto &i : msg->cone_yellow)
    {
        sgtdv_msgs::Cone cone;
        cone.coords.x = i.position.x;
        cone.coords.y = i.position.y;
        cone.color = 'y';
        map->cones.emplace_back(cone);
    }

    for (auto &i : msg->cone_orange)
    {
        sgtdv_msgs::Cone cone;
        cone.coords.x = i.position.x;
        cone.coords.y = i.position.y;
        cone.color = 'g';
        map->cones.emplace_back(cone);
    }
    m_mapPublisher.publish(map);
}
void SLAMSimInterface::DoState(const fsd_common_msgs::CarState::ConstPtr &msg) const
{
    sgtdv_msgs::CarPosePtr carPose(new sgtdv_msgs::CarPose);

    carPose->position.x = msg->car_state.x;
    carPose->position.y = msg->car_state.y;
    carPose->yaw = msg->car_state.theta;

    m_posePublisher.publish(carPose);

    /*sgtdv_msgs::CarVelPtr carVel(new sgtdv_msgs::CarVel);

    carVel->speed = msg->car_state_dt.car_state_dt.x;
    carVel->yawRate = msg->car_state_dt.car_state_dt.theta;

    m_velocityPublisher.publish(carVel);*/
}