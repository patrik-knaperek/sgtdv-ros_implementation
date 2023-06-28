/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include "../include/PoseEstimate.h"
#include <tf/tf.h>

PoseEstimate::PoseEstimate()
{
    m_currentState.position.x = 0.f;
    m_currentState.position.y = 0.f;
    m_currentState.yaw = 0.f;
}

PoseEstimate::~PoseEstimate()
{

}

void PoseEstimate::SetPublisher(ros::Publisher publisher)
{
    m_publisher = publisher;
}

void PoseEstimate::DoSlamState(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
    m_currentState.position = msg->position;
    m_currentState.yaw = msg->yaw;

    SendCarPose();
}

void PoseEstimate::DoIMU()//imu msg)
{
    //m_currentState.position +=
    //m_currentState.yaw = 

    SendCarPose();
}

void PoseEstimate::DoCameraPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    m_currentState.position.x = msg->pose.pose.position.x;
    m_currentState.position.y = msg->pose.pose.position.y;

    tf::Quaternion q(
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	m_currentState.yaw = yaw;
    
    SendCarPose();
}

void PoseEstimate::SendCarPose()
{
    sgtdv_msgs::CarPosePtr msg (new sgtdv_msgs::CarPose);

    msg->position = m_currentState.position;
    msg->yaw = m_currentState.yaw;

    m_publisher.publish(msg);
}