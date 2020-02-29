#include <ros/ros.h>
#include "../include/JetsonCanInterface.h"
#include <sgtdv_msgs/Control.h>

int main(int argc, char** argv)
{
    JetsonCanInterface jetsonCanInterface;

    ros::init(argc, argv, "jetson_can_interface");
    ros::NodeHandle handle;

    ros::Subscriber pathTrackingSub = handle.subscribe("pathtracking_commands", 1, &JetsonCanInterface::Do, &jetsonCanInterface);

    ros::spin();

    return 0;
}
