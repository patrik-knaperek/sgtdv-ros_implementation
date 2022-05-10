/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/SensorCalibrationSynch.h"

SensorCalibrationSynch::SensorCalibrationSynch()
{   

}

SensorCalibrationSynch::~SensorCalibrationSynch()
{

}

void SensorCalibrationSynch::SetDataSize(int numOfMeassurements, int numOfCones)
{
    m_numOfMeassurements = numOfMeassurements;
    m_numOfCones = numOfCones;
    
    m_calibrationObj.SetNumOfCones(numOfCones);
    
    this->Init();
}

void SensorCalibrationSynch::Init()
{
    m_cameraObsX = MatrixXd::Zero(m_numOfMeassurements,m_numOfCones);
    m_cameraObsY = MatrixXd::Zero(m_numOfMeassurements,m_numOfCones);
    m_cameraCount = RowVectorXi::Zero(m_numOfCones);

    m_lidarObsX = MatrixXd::Zero(m_numOfMeassurements,m_numOfCones);
    m_lidarObsY = MatrixXd::Zero(m_numOfMeassurements,m_numOfCones);
    m_lidarCount = RowVectorXi::Zero(m_numOfCones);
}

// get meassurement from camera
void SensorCalibrationSynch::DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg)
{
    int msgSize = msg->cones.size();
    if (msgSize == 0) return;

    std::cout << "collected meassurements from camera: " << m_cameraCount << std::endl;
    
    geometry_msgs::PointStamped coordsMsgFrame = geometry_msgs::PointStamped();
    geometry_msgs::PointStamped coordsFixedFrame = geometry_msgs::PointStamped();
    for (int i = 0; i < msgSize; i++)
    {
        if (std::isnan(msg->cones[i].coords.x) || std::isnan(msg->cones[i].coords.y))
            continue;
            
        // transformation to common frame
        coordsMsgFrame.header = msg->cones[i].coords.header;
        coordsMsgFrame.point.x = msg->cones[i].coords.x;
        coordsMsgFrame.point.y = msg->cones[i].coords.y;
        coordsMsgFrame.point.z = 0;

        if (coordsMsgFrame.header.frame_id.compare(m_fixedFrame) == 0)
            coordsFixedFrame = coordsMsgFrame;
        else
            coordsFixedFrame = TransformCoords(coordsMsgFrame);
            
        // data association
        RowVector2d meassuredCoords(coordsFixedFrame.point.x, coordsFixedFrame.point.y);
        int idx = DataAssociation(meassuredCoords, m_cameraObsX, m_cameraObsY, m_cameraCount);
        if (idx < 0) continue;

        if (m_cameraCount(idx) >= m_numOfMeassurements)
        { 
            MatrixX2d meassurementSet(m_numOfMeassurements,2);
            meassurementSet.col(0) = m_cameraObsX.col(idx);
            meassurementSet.col(1) = m_cameraObsY.col(idx);
            m_calibrationObj.Do(meassurementSet, m_realCoords.row(idx), "camera");
        }
    }
}

// get meassurement from lidar
void SensorCalibrationSynch::DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg)
{
    int msgSize = msg->points.size();
    if (msgSize == 0) return;

    std::cout << "collected meassurements from lidar: " << m_lidarCount << std::endl;
       
    geometry_msgs::PointStamped coordsMsgFrame = geometry_msgs::PointStamped();
    geometry_msgs::PointStamped coordsFixedFrame = geometry_msgs::PointStamped();
    for (int i = 0; i < msgSize; i++)
    {
        if (std::isnan(msg->points[i].x) || std::isnan(msg->points[i].y))
            continue;

        // transformation to common frame
        coordsMsgFrame.header = msg->points[i].header;
        coordsMsgFrame.point.x = msg->points[i].x;
        coordsMsgFrame.point.y = msg->points[i].y;
        coordsMsgFrame.point.z = 0;

        if (coordsMsgFrame.header.frame_id.compare(m_fixedFrame) != 0)
            coordsFixedFrame = TransformCoords(coordsMsgFrame);
        else
            coordsFixedFrame = coordsMsgFrame;

        // data association
        RowVector2d meassuredCoords(coordsFixedFrame.point.x, coordsFixedFrame.point.y);
        int idx = DataAssociation(meassuredCoords, m_lidarObsX, m_lidarObsY, m_lidarCount);
        if (idx < 0) continue;
            
        if (m_lidarCount(idx) >= m_numOfMeassurements)
        {
            MatrixX2d meassurementSet(m_numOfMeassurements,2);
            meassurementSet.col(0) = m_lidarObsX.col(idx);
            meassurementSet.col(1) = m_lidarObsY.col(idx);
            m_calibrationObj.Do(meassurementSet, m_realCoords.row(idx), "lidar");
        }
    }
}

geometry_msgs::PointStamped SensorCalibrationSynch::TransformCoords(geometry_msgs::PointStamped coordsChildFrame)
{
    geometry_msgs::PointStamped coordsParentFrame = geometry_msgs::PointStamped();
    try
    {
        m_listener.transformPoint(m_fixedFrame, coordsChildFrame, coordsParentFrame);
    }
    catch (tf::TransformException &e)
    {
        std::cout << e.what();
    }
    return coordsParentFrame;
}

int SensorCalibrationSynch::DataAssociation(const Ref<const RowVector2d> &meassuredCoords, Ref<MatrixXd> obsX,
                                            Ref<MatrixXd> obsY, Ref<RowVectorXi> obsCount)
{
    for (int i = 0; i < m_numOfCones; i++)
    {
        if (obsCount(i) < m_numOfMeassurements)
        {
            if (euclidDist(m_realCoords.row(i), meassuredCoords) < m_distTH)
                {   
                    obsX(obsCount(i),i) = meassuredCoords(0);
                    obsY(obsCount(i),i) = meassuredCoords(1);
                    obsCount(i)++;
                    return i;
                }
        }
    }
    return -1;
}

double SensorCalibrationSynch::euclidDist(const Ref<const RowVector2d> &v1, const Ref<const RowVector2d> &v2)
{
    RowVector2d diff(2);
    diff(0) = v1(0) - v2(0);
    diff(1) = v1(1) - v2(1);

    //std::cout << "\ndist: " << diff.norm() << std::endl;
    return diff.norm();
}