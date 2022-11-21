/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/VisualOdometry.h"

VisualOdometry::VisualOdometry()
{
    try
    {
        m_listener.lookupTransform(m_baseFrameId, "camera_left", ros::Time(0), m_cameraToBaseTF);
    }
    catch(const std::exception& e)
    {
        std::cout << e.what();
    }

}

VisualOdometry::~VisualOdometry()
{

}

void VisualOdometry::BroadcastTf(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    tf::Transform baseToOdomTF;
    
    tf::Vector3 cameraPositionOdomFrame(msg->pose.pose.position.x - m_cameraToBaseTF.getOrigin().getX(),
                                        msg->pose.pose.position.y - m_cameraToBaseTF.getOrigin().getY(),
                                        msg->pose.pose.position.z - m_cameraToBaseTF.getOrigin().getZ());
    tf::Quaternion cameraOrientationOdomFrame(msg->pose.pose.orientation.x,
                                        msg->pose.pose.orientation.y,
                                        msg->pose.pose.orientation.z,
                                        msg->pose.pose.orientation.w);
    
    baseToOdomTF.setOrigin(cameraPositionOdomFrame);
    baseToOdomTF.setRotation(cameraOrientationOdomFrame);

    m_broadcaster.sendTransform(tf::StampedTransform(baseToOdomTF, msg->header.stamp, msg->header.frame_id, m_baseFrameId));
}