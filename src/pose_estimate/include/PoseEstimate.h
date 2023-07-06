/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include <ros/ros.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
//#include  IMU msg
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

class PoseEstimate
{
public:
    PoseEstimate(const ros::Publisher& posePublisher, const ros::Publisher& velocityPublisher);
    ~PoseEstimate() = default;

    void DoSlamState(const sgtdv_msgs::CarPose::ConstPtr &msg);
    // void DoIMU(/*imu msg*/);
    void DoCameraPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void DoOdometry(const nav_msgs::Odometry::ConstPtr &msg);
private:
    ros::Publisher m_posePublisher, m_velocityPublisher;
    sgtdv_msgs::CarPose m_carPoseMsg;
    sgtdv_msgs::CarVel m_carVelMsg;
};