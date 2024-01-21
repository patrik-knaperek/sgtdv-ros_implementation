/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>

// AMZ FSSIM
#include <fsd_common_msgs/ControlCommand.h>
#include <fsd_common_msgs/CarState.h>
#include <fsd_common_msgs/CarStateDt.h>

// SGT-DV
#include <sgtdv_msgs/Control.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
#include <sgtdv_msgs/Point2DArr.h>

class PathTrackingSimInterface
{
    public:
        PathTrackingSimInterface();
        ~PathTrackingSimInterface() = default;

        // Setters
        void setPublishers(const ros::Publisher &cmdPub, const ros::Publisher &trajectoryPub, 
                           const ros::Publisher &posePub, const ros::Publisher &velPub)
        {
            m_cmdPublisher = cmdPub;
            m_trajectoryPublisher = trajectoryPub;
            m_posePublisher = posePub;
            m_velocityPublisher = velPub;
        };

        // Callbacks
        void DoCmd(const sgtdv_msgs::Control::ConstPtr &msg) const;
        void DoTrajectory(const geometry_msgs::PolygonStamped::ConstPtr &msg) const;
        void DoPose(const fsd_common_msgs::CarState::ConstPtr &msg) const;
        void DoVelocity(const fsd_common_msgs::CarStateDt::ConstPtr &msg) const;

    private:
        // SGT-DV --> FSSIM
        ros::Publisher m_cmdPublisher;

        // FSSIM --> SGT-DV
        ros::Publisher m_trajectoryPublisher;
        ros::Publisher m_posePublisher;
        ros::Publisher m_velocityPublisher;
};