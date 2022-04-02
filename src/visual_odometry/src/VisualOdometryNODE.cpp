/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/VisualOdometry.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_odometry");
    ros::NodeHandle handle;

    VisualOdometry tfBroadcaster;

    ros::Subscriber subscriber = handle.subscribe("/camera_pose", 1, &VisualOdometry::BroadcastTf, &tfBroadcaster);

    ros::spin();
}