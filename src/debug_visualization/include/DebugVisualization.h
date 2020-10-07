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
enum NODE_TYPE
{
  CAMERA = 0,
  LIDAR,
  FUSION,
  SLAM,
  PATH_PLANNING,
  PATH_TRACKING,
  JETSON_CAN_INTERFACE
};

struct FPoint2D
{
  FPoint2D(float X, float Y) {x = X; y = Y;}
  float x;
  float y;
};

struct NodeVisualization
{
  NodeVisualization(const FPoint2D &position, float width, float height);
  FPoint2D position;
  float width;
  float height;
  FPoint2D inConnection;
  FPoint2D outConnection;
};


public:

    void InitRViz();
    void SetPublisher(ros::Publisher publisher) { m_publisher = publisher; }
    void DoCamera(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoLidar(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoFusion(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoSLAM(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoPathPlanning(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoPathTracking(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoJetsonCANInterface(const sgtdv_msgs::DebugState::ConstPtr &msg);

private:
    ros::Publisher m_publisher;

    void InitCamera();
    void InitLidar();
    void InitFusion();
    void InitSLAM();
    void InitPathPlanning();
    void InitPathTracking();
    void InitJetsonCANInterface();

    // static NodeVisualization m_NODE_GEOMETRY[7] = 
    // {
    //   NodeVisualization(FPoint2D(0.f, 0.f), 10.f, 10.f),
    //   NodeVisualization(FPo)
    // };
};
  