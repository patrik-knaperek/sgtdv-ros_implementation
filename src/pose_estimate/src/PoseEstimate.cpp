/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include "../include/PoseEstimate.h"

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

void PoseEstimate::SendCarPose()
{
    sgtdv_msgs::CarPosePtr msg (new sgtdv_msgs::CarPose);

    msg->position = m_currentState.position;
    msg->yaw = m_currentState.yaw;

    m_publisher.publish(msg);
}