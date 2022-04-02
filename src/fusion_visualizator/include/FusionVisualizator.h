/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/FusionMsg.h>

class FusionVisualizator
{
    public:
        FusionVisualizator();
        ~FusionVisualizator();

        // Setters
        void SetPublishers(ros::Publisher cameraPub, ros::Publisher lidarPub, ros::Publisher fusionPub)
        {
            m_cameraPublisher = cameraPub;
            m_lidarPublisher = lidarPub;
            m_fusionPublisher = fusionPub;
        };

        void SetFrames(std::string cameraFrameId, std::string lidarFrameId, std::string fusionFrameId)
        {
            m_cameraFrameId = cameraFrameId;
            m_lidarFrameId = lidarFrameId;
            m_fusionFrameId = fusionFrameId;
        }
        
        // Callbacks
        void DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg);
        void DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg);
        void DoFusion(const sgtdv_msgs::ConeArr::ConstPtr &msg);
        void DeleteMarkers(visualization_msgs::MarkerArray markerArray,
                        ros::Publisher publisher, std::string frameId);

    private:
        ros::Publisher m_cameraPublisher;
        ros::Publisher m_lidarPublisher;
        ros::Publisher m_fusionPublisher;

        std::string m_cameraFrameId;
        std::string m_lidarFrameId;
        std::string m_fusionFrameId;

        visualization_msgs::MarkerArray m_cameraMarkers;
        visualization_msgs::MarkerArray m_lidarMarkers;
        visualization_msgs::Marker m_fusionMarker;
};
