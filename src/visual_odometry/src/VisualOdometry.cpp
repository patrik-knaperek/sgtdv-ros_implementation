/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/VisualOdometry.h"

VisualOdometry::VisualOdometry()
{

}

VisualOdometry::~VisualOdometry()
{

}

void VisualOdometry::BroadcastTf(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    tf::Transform transform;
    tf::Vector3 cameraPosition(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    tf::Quaternion cameraOrientation(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
                    msg->pose.pose.orientation.z);
    
    transform.setOrigin(cameraPosition);
    transform.setRotation(cameraOrientation);

    m_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "camera_left"));
}