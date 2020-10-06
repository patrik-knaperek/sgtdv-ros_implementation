/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

  //void DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg);

#pragma once

#include <ros/ros.h>
#include <sgtdv_msgs/DebugState.h>
#include <visualization_msgs/Marker.h>

class DebugVisualization
{
public:

    void InitRViz();
    void SetPublisher(ros::Publisher publisher) { m_publisher = publisher; }
    void DoCamera(const sgtdv_msgs::DebugState::ConstPtr &msg);
    //TODO

private:
    ros::Publisher m_publisher;
};
  