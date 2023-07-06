/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include "../include/PoseEstimate.h"
#include <tf/tf.h>

PoseEstimate::PoseEstimate(const ros::Publisher& posePublisher, const ros::Publisher& velocityPublisher)
: m_posePublisher(posePublisher)
, m_velocityPublisher(velocityPublisher)
{
}

void PoseEstimate::DoSlamState(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
    m_carPoseMsg.position = msg->position;
    m_carPoseMsg.yaw = msg->yaw;

    m_posePublisher.publish(m_carPoseMsg);
}

// void PoseEstimate::DoIMU()//imu msg)
// {
//     //m_currentState.position +=
//     //m_currentState.yaw = 

//     SendCarPose();
// }

void PoseEstimate::DoCameraPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    m_carPoseMsg.position.x = msg->pose.pose.position.x;
    m_carPoseMsg.position.y = msg->pose.pose.position.y;

    tf::Quaternion q(
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	m_carPoseMsg.yaw = yaw;
    
    m_posePublisher.publish(m_carPoseMsg);
}

void PoseEstimate::DoOdometry(const nav_msgs::Odometry::ConstPtr &msg)
{
    m_carPoseMsg.position.x = msg->pose.pose.position.x;
    m_carPoseMsg.position.y = msg->pose.pose.position.y;

    tf::Quaternion q(
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	m_carPoseMsg.yaw = yaw;

    m_carVelMsg.speed = msg->twist.twist.linear.x;
    m_carVelMsg.yawRate = msg->twist.twist.angular.z;

    m_posePublisher.publish(m_carPoseMsg);
    m_velocityPublisher.publish(m_carVelMsg);
}