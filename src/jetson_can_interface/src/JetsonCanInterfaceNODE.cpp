/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include "../include/JetsonCanInterface.h"
#include <sgtdv_msgs/Control.h>

int main(int argc, char** argv)
{
    JetsonCanInterface jetsonCanInterface;

    ros::init(argc, argv, "jetsonCanInterface");
    ros::NodeHandle handle;

    ros::Subscriber pathTrackingSub = handle.subscribe("pathtracking_commands", 1, &JetsonCanInterface::Do, &jetsonCanInterface);

    //ros::spin();
    sgtdv_msgs::Control control;

    while(ros::ok())
    {
        std::cin >> control.speed;
        jetsonCanInterface.Do(control);
    }

    return 0;
}
