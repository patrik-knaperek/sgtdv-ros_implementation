/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

  //void DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg); 

#pragma once

#include <ros/ros.h>
#include <sgtdv_msgs/DebugState.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

constexpr int32_t NUM_OF_CONNECTION_LINES = 6;
constexpr int32_t NUM_OF_NODES = 7;
constexpr const char* FRAME_ID = "/sgt_frame";
constexpr const char* NODE_RECT_NAMESPACE = "nodeGeometry";
constexpr const char* LINE_CONNECTION_NAMESPACE = "lineConnections";
constexpr const float X_OFFSET = -2.f;
constexpr const float Y_OFFSET = 2.f;

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
    FPoint2D() { x = 0.f; y = 0.f; }
    FPoint2D(float X, float Y) {x = X; y = Y;}
    geometry_msgs::Point GetPoint() const;
    float x;
    float y;
};

struct NodeGeometry
{
  NodeGeometry(const FPoint2D &position, float scaleX, float scaleY);
  FPoint2D position;
  float scaleX;
  float scaleY;
};

public:
    DebugVisualization();
    void InitRViz();
    void SetPublisher(ros::Publisher publisher) { m_publisher = publisher; }
    void DoCamera(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoLidar(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoFusion(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoSLAM(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoPathPlanning(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoPathTracking(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoJetsonCANInterface(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void PublishAllNodes();

private:
    ros::Publisher m_publisher;
    visualization_msgs::Marker m_markers[NUM_OF_NODES];

    void InitConnectionLines();    
    void InitNodes();

    static const NodeGeometry m_NODE_GEOMETRY[NUM_OF_NODES];
};
  