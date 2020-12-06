/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include "../include/JetsonCanInterface.h"
#include <sgtdv_msgs/Control.h>
#include <thread>

int main(int argc, char** argv)
{
    JetsonCanInterface jetsonCanInterface;
    std::thread listenThread;

    ros::init(argc, argv, "jetsonCanInterface");
    ros::NodeHandle handle;

    ros::Subscriber pathTrackingSub = handle.subscribe("pathtracking_commands", 1, &JetsonCanInterface::Do, &jetsonCanInterface);

    std::vector<int> msgIDListenFilter = {0x230};

    listenThread = std::thread(&JetsonCanInterface::DoListen, msgIDListenFilter);
    listenThread.detach();

    //ros::spin();
    sgtdv_msgs::Control control;

    while(ros::ok())
    {
      //  std::cin >> control.speed;
        jetsonCanInterface.Do(control);
    }

    return 0;
}
