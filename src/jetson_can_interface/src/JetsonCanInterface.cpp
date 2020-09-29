/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tibor Kominko, Juraj Krasňanský
/*****************************************************/


#include "../include/JetsonCanInterface.h"

JetsonCanInterface::JetsonCanInterface()
{

}

JetsonCanInterface::~JetsonCanInterface()
{

}

void JetsonCanInterface::Do(const sgtdv_msgs::Control::ConstPtr &msg)
{
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;
    //TODO: Send to CAN
    //msg->speed
    //msg->steeringAngle
}
