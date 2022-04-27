/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <cmath>

#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/Point2DArr.h>
#include "../include/Fusion.h"
#include "../include/Messages.h"
#include "../../SGT_Macros.h"

class FusionSynch
{
    public:
        FusionSynch();
        ~FusionSynch();

        // Setters
        void SetFrames(std::string fixedFrame, std::string cameraFrame, std::string lidarFrame)
        {
            m_fixedFrameId = fixedFrame;
            m_cameraFrameId = cameraFrame;
            m_lidarFrameId = lidarFrame;
        };

        void SetPublisher(ros::Publisher publisher) { m_fusion.SetPublisher(publisher); };
        void SetDistanceTol(float tol) { m_fusion.SetDistanceTol(tol); };
        
    #ifdef SGT_DEBUG_STATE
        void SetVisDebugPublisher(ros::Publisher publisher) { m_fusion.SetVisDebugPublisher(publisher); }
    #endif

        void DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg);
        void DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg);
        geometry_msgs::PointStamped TransformCoords(geometry_msgs::PointStamped coordsChildFrame);

    private:
        Fusion m_fusion;
        bool m_cameraReady; 
        bool m_lidarReady;
        FusionMsg m_fusionMsg;

        std::string m_fixedFrameId;
        std::string m_cameraFrameId;
        std::string m_lidarFrameId;

        tf::TransformListener m_listener;
};