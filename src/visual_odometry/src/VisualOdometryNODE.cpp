/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/VisualOdometry.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualOdometry");
    ros::NodeHandle handle;

    VisualOdometry tfBroadcaster;

    std::string baseFrame;
    if(!handle.param("/base_frame", baseFrame, std::string("camera_center")))
        ROS_INFO("Failed to get parameter \"/base_frame\" from server. Setting default value.\n");
    tfBroadcaster.SetBaseFrameId(baseFrame);

    ros::Subscriber subscriber = handle.subscribe("/camera_pose", 1, &VisualOdometry::BroadcastTf, &tfBroadcaster);

    ros::spin();
}