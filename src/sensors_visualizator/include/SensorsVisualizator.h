/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/FusionMsg.h>
#include "../../SGT_Macros.h"

class SensorsVisualizator
{
    public:
        SensorsVisualizator();
        ~SensorsVisualizator();

        // Setters
        void SetPublishers(ros::Publisher cameraPub, ros::Publisher lidarPub, ros::Publisher fusionPub)
        {
            m_cameraPublisher = cameraPub;
            m_lidarPublisher = lidarPub;
            m_fusionPublisher = fusionPub;
        };
        
        // Callbacks
        void DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg);
        void DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg);
        void DoFusion(const sgtdv_msgs::ConeArr::ConstPtr &msg);
        void DeleteMarkers(visualization_msgs::MarkerArray markerArray,
                        ros::Publisher publisher);
        
    #ifdef SIMPLE_FUSION
        void DoSimpleFusion(const sgtdv_msgs::ConeArr::ConstPtr &msg);
    #endif

    private:
        ros::Publisher m_cameraPublisher;
        ros::Publisher m_lidarPublisher;
        ros::Publisher m_fusionPublisher;

        visualization_msgs::MarkerArray m_cameraMarkers;
        visualization_msgs::MarkerArray m_lidarMarkers;
        visualization_msgs::MarkerArray m_fusionMarkers;
};
