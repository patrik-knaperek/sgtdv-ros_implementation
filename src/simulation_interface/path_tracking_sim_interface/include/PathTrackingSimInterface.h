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

// SGT-DV
#include <sgtdv_msgs/Control.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
#include <sgtdv_msgs/Point2DArr.h>

class PathTrackingSimInterface
{
    public:
        PathTrackingSimInterface();
        ~PathTrackingSimInterface();

        // Setters
        void setPublishers(ros::Publisher cmdPub, ros::Publisher trajectoryPub, ros::Publisher posePub, ros::Publisher velPub)
        {
            m_cmdPublisher = cmdPub;
            m_trajectoryPublisher = trajectoryPub;
            m_posePublisher = posePub;
            m_velocityPublisher = velPub;
        };

        // Callbacks
        void DoCmd(const sgtdv_msgs::Control::ConstPtr &msg);
        void DoTrajectory(const geometry_msgs::PolygonStamped::ConstPtr &msg);
        void DoState(const fsd_common_msgs::CarState::ConstPtr &msg);

    private:
        // SGT-DV --> FSSIM
        ros::Publisher m_cmdPublisher;

        // FSSIM --> SGT-DV
        ros::Publisher m_trajectoryPublisher;
        ros::Publisher m_posePublisher;
        ros::Publisher m_velocityPublisher;
};