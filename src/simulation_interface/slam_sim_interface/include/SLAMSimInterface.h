/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

// ROS
#include <ros/ros.h>
#include <std_msgs/Empty.h>

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
        void setPublishers(const ros::Publisher &mapPub, const ros::Publisher &posePub
                            , const ros::Publisher &loopClosePub/*, const ros::Publisher &velPub*/)
        {
            m_mapPublisher = mapPub;
            m_posePublisher = posePub;
            m_loopClosePub = loopClosePub;
            //m_velocityPublisher = velPub;
        };

        // Callbacks
        void DoMap(const fsd_common_msgs::Map::ConstPtr &msg);
        void DoState(const fsd_common_msgs::CarState::ConstPtr &msg);

        void PublishMap();

    private:
        static constexpr float LOOK_AHEAD_DISTANCE = 15.0;

        // SGT-DV --> FSSIM

        // FSSIM --> SGT-DV
        void AddToMap(const fsd_common_msgs::Cone &cone);
        int FindLookAheadConeIdx(std::vector<fsd_common_msgs::Cone> cones);
        void ActualizeMap();
        void LoopClosure();
        std::vector<fsd_common_msgs::Cone> m_conesBlue, m_conesYellow;
        ros::Publisher m_mapPublisher, m_posePublisher, m_loopClosePub/*, m_velocityPublisher*/;

        sgtdv_msgs::ConeArr m_map;
        sgtdv_msgs::CarPose m_carPose;

        int m_conesBlueCount, m_conesYellowCount;
        bool m_mapReady, m_loopClosure;
};