/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include <sgtdv_msgs/CarPose.h>
//#include  IMU msg

class PoseEstimate
{
public:
    PoseEstimate();
    ~PoseEstimate();

    void SetPublisher(ros::Publisher publisher);
    void DoSlamState(const sgtdv_msgs::CarPose::ConstPtr &msg);
    void DoIMU(/*imu msg*/);
private:
    ros::Publisher m_publisher;
    sgtdv_msgs::CarPose m_currentState;

    void SendCarPose();
};