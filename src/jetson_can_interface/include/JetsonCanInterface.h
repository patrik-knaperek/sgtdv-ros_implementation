/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#pragma once

#include <sgtdv_msgs/Control.h>


class JetsonCanInterface
{
public:
    JetsonCanInterface();
    ~JetsonCanInterface();

    void Do(const sgtdv_msgs::Control::ConstPtr &msg);
private:
};