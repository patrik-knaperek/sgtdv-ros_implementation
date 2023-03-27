/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

// ROS
#include <ros/ros.h>

// AMZ FSSIM
#include <fsd_common_msgs/Map.h>
#include <fsd_common_msgs/CarState.h>

// SGT-DV
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
#include <sgtdv_msgs/Point2DArr.h>

class SLAMSimInterface
{
    public:
        SLAMSimInterface();
        ~SLAMSimInterface() = default;

        // Setters
        void setPublishers(const ros::Publisher &mapPub, const ros::Publisher &posePub/*, const ros::Publisher &velPub*/)
        {
            m_mapPublisher = mapPub;
            m_posePublisher = posePub;
            //m_velocityPublisher = velPub;
        };

        // Callbacks
        void DoMap(const fsd_common_msgs::Map::ConstPtr &msg);
        void DoState(const fsd_common_msgs::CarState::ConstPtr &msg) const;

        void PublishMap();

    private:
        // SGT-DV --> FSSIM

        // FSSIM --> SGT-DV
        ros::Publisher m_mapPublisher;
        ros::Publisher m_posePublisher;
        //ros::Publisher m_velocityPublisher;

        sgtdv_msgs::ConeArr m_map;
        bool m_mapReady;
};