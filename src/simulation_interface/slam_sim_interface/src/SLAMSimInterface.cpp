/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/SLAMSimInterface.h"

SLAMSimInterface::SLAMSimInterface()
: m_conesBlueCount(0), m_conesYellowCount(0), m_mapReady(false),  m_loopClosure(false)
{
    ROS_INFO("Initialized");
}

void SLAMSimInterface::DoMap(const fsd_common_msgs::Map::ConstPtr &msg)
{
    if(!m_mapReady)
    {
        m_mapReady = true;
    }

    m_map.cones.reserve(msg->cone_blue.size() + msg->cone_yellow.size() + msg->cone_orange.size());
    

    m_conesBlue = msg->cone_blue;
    m_conesYellow = msg->cone_yellow;

    for (auto &i : msg->cone_orange)
    {
        AddToMap(i);
    }

    if (m_loopClosure)
    {
        for (auto &i : msg->cone_blue)
        {
            AddToMap(i);
        }

        for (auto &i : msg->cone_yellow)
        {
            AddToMap(i);
        }
    }
    
}

void SLAMSimInterface::DoState(const fsd_common_msgs::CarState::ConstPtr &msg)
{
    m_carPose.position.x = msg->car_state.x;
    m_carPose.position.y = msg->car_state.y;
    m_carPose.yaw = msg->car_state.theta;
    
    m_posePublisher.publish(m_carPose);

    if (!m_loopClosure)
    {
        ActualizeMap();
    }
    
    /*sgtdv_msgs::CarVelPtr carVel(new sgtdv_msgs::CarVel);

    carVel->speed = msg->car_state_dt.car_state_dt.x;
    carVel->yawRate = msg->car_state_dt.car_state_dt.theta;

    m_velocityPublisher.publish(carVel);*/
}

void SLAMSimInterface::AddToMap(const fsd_common_msgs::Cone &coneMsg)
{
    sgtdv_msgs::Cone cone;
    cone.coords.x = coneMsg.position.x;
    cone.coords.y = coneMsg.position.y;

    if (coneMsg.color.data == "blue")
    {
        cone.color = 'b';
    } else if (coneMsg.color.data == "yellow")
    {
        cone.color = 'y';
    } else if (coneMsg.color.data == "orange")
    {
        cone.color = 'g';
    }

    m_map.cones.emplace_back(cone);
}

int SLAMSimInterface::FindLookAheadConeIdx(std::vector<fsd_common_msgs::Cone> cones)
{
    const auto closestIt  = std::min_element(cones.begin(), cones.end(),
                                                [&](const fsd_common_msgs::Cone &a,
                                                const fsd_common_msgs::Cone &b) {
                                                const double da = std::hypot(m_carPose.position.x - a.position.x,
                                                                            m_carPose.position.y - a.position.y);
                                                const double db = std::hypot(m_carPose.position.x - b.position.x,
                                                                            m_carPose.position.y - b.position.y);

                                                return da < db;
                                                });
    const auto closestIdx =  std::distance(cones.begin(), closestIt);

    const auto size = cones.size();
    static int offset;
    static int nextIdx;
    double distance;
    
    offset = 0;
    do
    {
        
        nextIdx = (closestIdx + offset++);
        if (nextIdx > size - 1) break;

        distance = std::sqrt(std::pow(cones[nextIdx].position.x - m_carPose.position.x,2) 
                        + std::pow(cones[nextIdx].position.y - m_carPose.position.y,2));
    
    } while (distance < LOOK_AHEAD_DISTANCE);
    return nextIdx;
}

void SLAMSimInterface::ActualizeMap()
{
    int coneBlueLookAheadIdx = FindLookAheadConeIdx(m_conesBlue);
    int coneYellowLookAheadIdx = FindLookAheadConeIdx(m_conesYellow);

    while (m_conesBlueCount < coneBlueLookAheadIdx && m_conesBlueCount < m_conesBlue.size())
    {
        AddToMap(m_conesBlue[m_conesBlueCount++]);
    }

    while (m_conesYellowCount < coneYellowLookAheadIdx && m_conesYellowCount < m_conesYellow.size())
    {
        AddToMap(m_conesYellow[m_conesYellowCount++]);
    }

    if (m_conesBlueCount == m_conesBlue.size() && m_conesYellowCount == m_conesYellow.size())
    {
        LoopClosure();
        return;
    }
}

void SLAMSimInterface::LoopClosure()
{
    m_loopClosure = true;
    m_loopClosePub.publish(std_msgs::Empty());
}

void SLAMSimInterface::PublishMap()
{
    if (m_mapReady)
        m_mapPublisher.publish(m_map);
}