/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class VisualOdometry {
    public:
        VisualOdometry();
        ~VisualOdometry();

        void BroadcastTf(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
        void SetBaseFrameId(const std::string baseFrameId)
        {
            this->m_baseFrameId = baseFrameId;
        };

    private:
        
        std::string m_baseFrameId;
        tf::StampedTransform m_cameraToBaseTF;
        tf::TransformBroadcaster m_broadcaster;
        tf::TransformListener m_listener;

};
