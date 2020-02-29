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

void PoseEstimate::DoSlamState(const sgtdv_msgs::CarState::ConstPtr &msg)
{
    m_currentState.position = msg->position;
    m_currentState.yaw = msg->yaw;

    SendCarState();
}

void PoseEstimate::DoIMU()//imu msg)
{
    //m_currentState.position +=
    //m_currentState.yaw = 

    SendCarState();
}

void PoseEstimate::SendCarState()
{
    sgtdv_msgs::CarStatePtr msg (new sgtdv_msgs::CarState);

    msg->position = m_currentState.position;
    msg->yaw = m_currentState.yaw;

    m_publisher.publish(msg);
}