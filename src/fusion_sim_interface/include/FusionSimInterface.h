/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/Point2DArr.h>

class SimInterface
{
    public:
        SimInterface();
        ~SimInterface();

        void setPublishers(ros::Publisher cameraPub, ros::Publisher lidarPub)
        {
            this->m_cameraPublisher = cameraPub;
            this->m_lidarPublisher = lidarPub;
        };
        
        void DoLidar(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void DoCamera(const sensor_msgs::PointCloud2::ConstPtr &msg);
        
    private:
        ros::Publisher m_lidarPublisher;
        ros::Publisher m_cameraPublisher;

        ros::Subscriber m_lidarSubscriber;
        ros::Subscriber m_cameraSubscriber;
};