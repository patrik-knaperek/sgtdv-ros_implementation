/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#include "../include/DebugVisualization.h"

const DebugVisualization::NodeGeometry DebugVisualization::m_NODE_GEOMETRY[7] = 
{
    NodeGeometry(FPoint2D(0.f + X_GLOBAL_OFFSET, 0.f + Y_GLOBAL_OFFSET), 2.f, 1.f),       //camera
    NodeGeometry(FPoint2D(0.f + X_GLOBAL_OFFSET, -2.f + Y_GLOBAL_OFFSET), 2.f, 1.f),      //lidar
    NodeGeometry(FPoint2D(4.f + X_GLOBAL_OFFSET, -1.f + Y_GLOBAL_OFFSET), 2.f, 1.f),      //fusion
    NodeGeometry(FPoint2D(4.f + X_GLOBAL_OFFSET, -3.f + Y_GLOBAL_OFFSET), 2.f, 1.f),      //slam
    NodeGeometry(FPoint2D(4.f + X_GLOBAL_OFFSET, -5.f + Y_GLOBAL_OFFSET), 2.f, 1.f),      //pathPlanning
    NodeGeometry(FPoint2D(4.f + X_GLOBAL_OFFSET, -7.f + Y_GLOBAL_OFFSET), 2.f, 1.f),      //pathTracking
    NodeGeometry(FPoint2D(4.f + X_GLOBAL_OFFSET, -9.f + Y_GLOBAL_OFFSET), 2.f, 1.f)       //jetsonCanInterface
};

DebugVisualization::NodeGeometry::NodeGeometry(const FPoint2D &position, float scaleX, float scaleY)
{
    this->position = position;
    this->scaleX = scaleX;
    this->scaleY = scaleY;
}

geometry_msgs::Point DebugVisualization::FPoint2D::GetPoint() const
{
    geometry_msgs::Point point;

    point.x = x;
    point.y = y;
    point.z = 0.;

    return point;
}

DebugVisualization::DebugVisualization()
{
    InitMarkerArray();
    InitNodes();
    InitConnectionLines();
    InitNodeNames();
    InitNodeFrequency();
    InitNodeWorkTime();
    InitNodeOutputs();
}

void DebugVisualization::InitConnectionLines()
{
    for(int32_t i = 0; i < NUM_OF_CONNECTION_LINES; i++)
    {
        m_connectionLines[i].header.frame_id = FRAME_ID;
        m_connectionLines[i].ns = LINE_CONNECTION_NAMESPACE;
        m_connectionLines[i].id = i;
        m_connectionLines[i].action = visualization_msgs::Marker::ADD;

        m_connectionLines[i].type = visualization_msgs::Marker::ARROW;
        m_connectionLines[i].scale.x = 0.08f;
        m_connectionLines[i].scale.y = 0.08f;
        m_connectionLines[i].scale.z = 0.5f;

        m_connectionLines[i].pose.orientation.w = 1.f;
        m_connectionLines[i].pose.orientation.x = 0.f;
        m_connectionLines[i].pose.orientation.y = 0.f;
        m_connectionLines[i].pose.orientation.z = 0.f;

        m_connectionLines[i].color.a = 1.f;
        m_connectionLines[i].color.r = 0.5f;
        m_connectionLines[i].color.g = 0.5f;
        m_connectionLines[i].color.b = 0.5f;

        m_connectionLines[i].pose.position = FPoint2D(0.f, 0.f).GetPoint();

        m_connectionLines[i].points.reserve(2);
        m_connectionLines[i].points.push_back(m_NODE_GEOMETRY[i].position.GetPoint());
    }

    m_connectionLines[CAMERA].points.push_back(m_NODE_GEOMETRY[FUSION].position.GetPoint());   
    m_connectionLines[LIDAR].points.push_back(m_NODE_GEOMETRY[FUSION].position.GetPoint());    
    m_connectionLines[FUSION].points.push_back(m_NODE_GEOMETRY[SLAM].position.GetPoint());
    m_connectionLines[SLAM].points.push_back(m_NODE_GEOMETRY[PATH_PLANNING].position.GetPoint());   
    m_connectionLines[PATH_PLANNING].points.push_back(m_NODE_GEOMETRY[PATH_TRACKING].position.GetPoint());
    m_connectionLines[PATH_TRACKING].points.push_back(m_NODE_GEOMETRY[JETSON_CAN_INTERFACE].position.GetPoint());
}

void DebugVisualization::InitNodes()
{
    for(int32_t i = 0; i < NUM_OF_NODES; i++)
    {
        m_nodeMarkers[i].header.frame_id = FRAME_ID;
        m_nodeMarkers[i].ns = NODE_RECT_NAMESPACE;
        m_nodeMarkers[i].id = i;
        m_nodeMarkers[i].action = visualization_msgs::Marker::ADD;

        m_nodeMarkers[i].type = visualization_msgs::Marker::CUBE;
        m_nodeMarkers[i].scale.x = m_NODE_GEOMETRY[i].scaleX;
        m_nodeMarkers[i].scale.y = m_NODE_GEOMETRY[i].scaleY;
        m_nodeMarkers[i].scale.z = 0.1f;

        m_nodeMarkers[i].color.a = 1.f;
        m_nodeMarkers[i].color.r = 0.3f;
        m_nodeMarkers[i].color.g = 0.3f;
        m_nodeMarkers[i].color.b = 0.3f;

        m_nodeMarkers[i].pose.position = m_NODE_GEOMETRY[i].position.GetPoint();
        m_nodeMarkers[i].pose.orientation.w = 1.f;
        m_nodeMarkers[i].pose.orientation.x = 0.f;
        m_nodeMarkers[i].pose.orientation.y = 0.f;
        m_nodeMarkers[i].pose.orientation.z = 0.f;
    }
}

void DebugVisualization::InitNodeOutputs()
{
    for(int32_t i = 0; i < NUM_OF_NODES; i++)
    {
        m_nodeOutputs[i].header.frame_id = FRAME_ID;
        m_nodeOutputs[i].ns = OUTPUTS_NAMESPACE;
        m_nodeOutputs[i].id = i;
        m_nodeOutputs[i].action = visualization_msgs::Marker::ADD;

        m_nodeOutputs[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m_nodeOutputs[i].scale.z = 0.5f;

        m_nodeOutputs[i].pose.orientation.w = 1.f;
        m_nodeOutputs[i].pose.orientation.x = 0.f;
        m_nodeOutputs[i].pose.orientation.y = 0.f;
        m_nodeOutputs[i].pose.orientation.z = 0.f;

        m_nodeOutputs[i].color.a = 1.f;
        m_nodeOutputs[i].color.r = 1.f;
        m_nodeOutputs[i].color.g = 1.f;
        m_nodeOutputs[i].color.b = 1.f;

        m_nodeOutputs[i].pose.position.x = m_NODE_GEOMETRY[i].position.GetPoint().x + 1.f;
        m_nodeOutputs[i].pose.position.y = m_NODE_GEOMETRY[i].position.GetPoint().y - 0.5f;
        m_nodeOutputs[i].pose.position.z = 1.f;
        m_nodeOutputs[i].text = "OUTPUT";
    }
}

void DebugVisualization::PrintError(const char* NODE) const
{
    std::cout << "ERROR: Unknown debug state received - " << NODE << std::endl;
}

void DebugVisualization::Do(const sgtdv_msgs::DebugState::ConstPtr &msg, NODE_TYPE type)
{
    if (msg->workingState == 1)
    {        
        SetMarkerColorRed(type);
        m_startTime[type] = std::chrono::steady_clock::now();
        m_bStarted[type] = true;
    }
    else if (msg->workingState == 0)
    {
        std::stringstream ss;
        SetMarkerColorGreen(type);
        
        auto now = std::chrono::steady_clock::now();
        auto workLoadTime = (std::chrono::duration_cast<std::chrono::milliseconds>(now - m_startTime[type])).count();

        if (!m_bStarted[type]) workLoadTime = 0;

        ss << workLoadTime << " ms";
        m_nodeWorkTime[type].text = ss.str();
        ss.str("");

        auto timeSinceLastRun = (std::chrono::duration_cast<std::chrono::milliseconds>(now - m_endTime[type])).count();
        float seconds = timeSinceLastRun / 1000.f;
        ss << 1.f / seconds << " Hz";
        m_nodeFrequency[type].text = ss.str();
        m_endTime[type] = now;
        ss.str("");

        switch(type)
        {
            case CAMERA:
            case LIDAR:
            case FUSION:
            case SLAM:
                ss << msg->numOfCones << " cones";                
                break;
                
            case PATH_TRACKING:
            case JETSON_CAN_INTERFACE:
                ss << "Speed: " << msg->speed << " Angle: " << msg->angle;                
                break;
            default:
                break;
        }

        m_nodeOutputs[type].text = ss.str();
    }
    else
    {
        PrintError(NODE_TYPE_STRINGS[type]);
        return;
    }

    ros::Time now = ros::Time::now();

    m_nodeMarkers[type].header.stamp = now;
    m_connectionLines[type].header.stamp = now;
    m_nodeWorkTime[type].header.stamp = now;
    m_nodeFrequency[type].header.stamp = now;
    m_nodeOutputs[type].header.stamp = now;
    m_bStarted[type] = false;
}

void DebugVisualization::DoCamera(const sgtdv_msgs::DebugState::ConstPtr &msg)
{
    Do(msg, CAMERA);
}

void DebugVisualization::DoLidar(const sgtdv_msgs::DebugState::ConstPtr &msg)
{
    Do(msg, LIDAR);
}

void DebugVisualization::DoFusion(const sgtdv_msgs::DebugState::ConstPtr &msg)
{
    Do(msg, FUSION);
}

void DebugVisualization::DoSLAM(const sgtdv_msgs::DebugState::ConstPtr &msg)
{
    Do(msg, SLAM);
}

void DebugVisualization::DoPathPlanning(const sgtdv_msgs::DebugState::ConstPtr &msg)
{
    Do(msg, PATH_PLANNING);
}

void DebugVisualization::DoPathTracking(const sgtdv_msgs::DebugState::ConstPtr &msg)
{
    Do(msg, PATH_TRACKING);
}

void DebugVisualization::DoJetsonCANInterface(const sgtdv_msgs::DebugState::ConstPtr &msg)
{
    Do(msg, JETSON_CAN_INTERFACE);
}

void DebugVisualization::SetMarkerColor(float r, float g, float b, NODE_TYPE type)
{
    m_nodeMarkers[type].color.r = r;
    m_nodeMarkers[type].color.g = g;
    m_nodeMarkers[type].color.b = b;
}

void DebugVisualization::SetMarkerColorRed(NODE_TYPE type)
{
    SetMarkerColor(1.f, 0.f, 0.f, type);
}

void DebugVisualization::SetMarkerColorGreen(NODE_TYPE type)
{
    SetMarkerColor(0.f, 1.f, 0.f, type);
}

void DebugVisualization::InitNodeNames()
{
    for(int32_t i = 0; i < NUM_OF_NODES; i++)
    {
        m_nodeNames[i].header.frame_id = FRAME_ID;
        m_nodeNames[i].ns = NAMES_NAMESPACE;
        m_nodeNames[i].id = i;
        m_nodeNames[i].action = visualization_msgs::Marker::ADD;

        m_nodeNames[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m_nodeNames[i].scale.z = 0.5f;

        m_nodeNames[i].pose.orientation.w = 1.f;
        m_nodeNames[i].pose.orientation.x = 0.f;
        m_nodeNames[i].pose.orientation.y = 0.f;
        m_nodeNames[i].pose.orientation.z = 0.f;

        m_nodeNames[i].color.a = 1.f;
        m_nodeNames[i].color.r = 1.f;
        m_nodeNames[i].color.g = 1.f;
        m_nodeNames[i].color.b = 1.f;

        m_nodeNames[i].pose.position = m_NODE_GEOMETRY[i].position.GetPoint();
        m_nodeNames[i].pose.position.z = 1.f;
        m_nodeNames[i].text = NODE_TYPE_STRINGS[i];
    }
}

void DebugVisualization::InitNodeFrequency()
{
    for(int32_t i = 0; i < NUM_OF_NODES; i++)
    {
        m_nodeFrequency[i].header.frame_id = FRAME_ID;
        m_nodeFrequency[i].ns = FREQUENCY_NAMESPACE;
        m_nodeFrequency[i].id = i;
        m_nodeFrequency[i].action = visualization_msgs::Marker::ADD;

        m_nodeFrequency[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m_nodeFrequency[i].scale.z = 0.5f;

        m_nodeFrequency[i].pose.orientation.w = 1.f;
        m_nodeFrequency[i].pose.orientation.x = 0.f;
        m_nodeFrequency[i].pose.orientation.y = 0.f;
        m_nodeFrequency[i].pose.orientation.z = 0.f;

        m_nodeFrequency[i].color.a = 1.f;
        m_nodeFrequency[i].color.r = 0.f;
        m_nodeFrequency[i].color.g = 1.f;
        m_nodeFrequency[i].color.b = 0.f;

        FPoint2D buff;
        buff.x = m_NODE_GEOMETRY[i].position.x + 1.f;
        buff.y = m_NODE_GEOMETRY[i].position.y + 0.5f;

        m_nodeFrequency[i].pose.position = buff.GetPoint();
        m_nodeFrequency[i].pose.position.z = 1.f;
        m_nodeFrequency[i].text = "0 Hz";
        m_endTime[i] = std::chrono::steady_clock::now();
    }
}

void DebugVisualization::InitNodeWorkTime()
{
    for(int32_t i = 0; i < NUM_OF_NODES; i++)
    {
        m_nodeWorkTime[i].header.frame_id = FRAME_ID;
        m_nodeWorkTime[i].ns = WORKTIME_NAMESPACE;
        m_nodeWorkTime[i].id = i;
        m_nodeWorkTime[i].action = visualization_msgs::Marker::ADD;

        m_nodeWorkTime[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m_nodeWorkTime[i].scale.z = 0.5f;

        m_nodeWorkTime[i].pose.orientation.w = 1.f;
        m_nodeWorkTime[i].pose.orientation.x = 0.f;
        m_nodeWorkTime[i].pose.orientation.y = 0.f;
        m_nodeWorkTime[i].pose.orientation.z = 0.f;

        m_nodeWorkTime[i].color.a = 1.f;
        m_nodeWorkTime[i].color.r = 1.f;
        m_nodeWorkTime[i].color.g = 0.f;
        m_nodeWorkTime[i].color.b = 0.f;

        FPoint2D buff;
        buff.x = m_NODE_GEOMETRY[i].position.x - 1.f;
        buff.y = m_NODE_GEOMETRY[i].position.y + 0.5f;

        m_nodeWorkTime[i].pose.position = buff.GetPoint();
        m_nodeWorkTime[i].pose.position.z = 1.f;
        m_nodeWorkTime[i].text = "0 ms";
        m_startTime[i] = std::chrono::steady_clock::now();
    }
}

void DebugVisualization::InitMarkerArray()
{
    m_markerArray.markers.resize(5 * NUM_OF_NODES + NUM_OF_CONNECTION_LINES);

    m_connectionLines = &(m_markerArray.markers[0]);
    m_nodeMarkers = m_connectionLines + NUM_OF_CONNECTION_LINES;
    m_nodeNames = m_nodeMarkers + NUM_OF_NODES;
    m_nodeFrequency = m_nodeNames + NUM_OF_NODES;
    m_nodeWorkTime = m_nodeFrequency + NUM_OF_NODES;
    m_nodeOutputs = m_nodeWorkTime + NUM_OF_NODES;    
}

void DebugVisualization::PublishEverythingAsArray()
{
    m_publisher.publish(m_markerArray);
}