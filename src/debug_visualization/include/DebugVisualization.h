/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#pragma once

#include <ros/ros.h>
#include <sgtdv_msgs/DebugState.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <string>
#include <sstream>

constexpr int32_t NUM_OF_CONNECTION_LINES = 6;
constexpr int32_t NUM_OF_NODES = 7;

#define ENUM_MACRO(name, v1, v2, v3, v4, v5, v6, v7)\
    enum name { v1 = 0, v2, v3, v4, v5, v6, v7};\
    const char *name##_STRINGS[NUM_OF_NODES] = { #v1, #v2, #v3, #v4, #v5, #v6, #v7};

constexpr const char* FRAME_ID = "sgt_frame";
constexpr const char* NODE_RECT_NAMESPACE = "nodeGeometry";
constexpr const char* LINE_CONNECTION_NAMESPACE = "lineConnections";
constexpr const char* NAMES_NAMESPACE = "nodeNames";
constexpr const char* FREQUENCY_NAMESPACE = "nodeFrequency";
constexpr const char* WORKTIME_NAMESPACE = "nodeWorkTime";
constexpr const char* OUTPUTS_NAMESPACE = "nodeOutputs";
constexpr const float X_GLOBAL_OFFSET = -2.f;
constexpr const float Y_GLOBAL_OFFSET = 2.f;

class DebugVisualization
{

ENUM_MACRO(NODE_TYPE, CAMERA, LIDAR, FUSION, SLAM, PATH_PLANNING, PATH_TRACKING, JETSON_CAN_INTERFACE)


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

    void SetPublisher(ros::Publisher publisher) { m_publisher = publisher; }
    
    void DoCamera(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoLidar(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoFusion(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoSLAM(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoPathPlanning(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoPathTracking(const sgtdv_msgs::DebugState::ConstPtr &msg);
    void DoJetsonCANInterface(const sgtdv_msgs::DebugState::ConstPtr &msg);
   
    void PublishAllNodes();
    void PublishAllConnectionLines();
    void PublishAllNames();
    void PublishAllWorkTimes();
    void PublishAllFrequencies();
    void PublishAllOutputs();
    void PublishEverything();
    void PublishEverythingAsArray();

private:
    ros::Publisher m_publisher;
    visualization_msgs::Marker *m_connectionLines = nullptr;
    visualization_msgs::Marker *m_nodeMarkers = nullptr;
    visualization_msgs::Marker *m_nodeNames = nullptr;
    visualization_msgs::Marker *m_nodeFrequency = nullptr;
    visualization_msgs::Marker *m_nodeWorkTime = nullptr;
    visualization_msgs::Marker *m_nodeOutputs = nullptr;
    visualization_msgs::MarkerArray m_markerArray;
    bool m_bStarted[NUM_OF_NODES] = { false, false, false, false, false, false, false };

    std::chrono::time_point<std::chrono::steady_clock> m_startTime[NUM_OF_NODES];
    std::chrono::time_point<std::chrono::steady_clock> m_endTime[NUM_OF_NODES];

    void Do(const sgtdv_msgs::DebugState::ConstPtr &msg, NODE_TYPE type);
    void InitMarkerArray();
    void InitConnectionLines();    
    void InitNodes();
    void InitNodeNames();
    void InitNodeFrequency();
    void InitNodeWorkTime();
    void InitNodeOutputs();

    void PrintError(const char* NODE) const;
    void SetMarkerColor(float r, float g, float b, NODE_TYPE type);
    void SetMarkerColorRed(NODE_TYPE type);
    void SetMarkerColorGreen(NODE_TYPE type);

    static const NodeGeometry m_NODE_GEOMETRY[NUM_OF_NODES];
};
  