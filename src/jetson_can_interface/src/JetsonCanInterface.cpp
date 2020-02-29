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
    //TODO: Send to CAN
    //msg->speed
    //msg->steeringAngle
}