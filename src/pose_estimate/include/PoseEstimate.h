#include <ros/ros.h>
#include <sgtdv_msgs/CarState.h>
//#include  IMU msg

class PoseEstimate
{
public:
    PoseEstimate();
    ~PoseEstimate();

    void SetPublisher(ros::Publisher publisher);
    void DoSlamState(const sgtdv_msgs::CarState::ConstPtr &msg);
    void DoIMU(/*imu msg*/);
private:
    ros::Publisher m_publisher;
    sgtdv_msgs::CarState m_currentState;

    void SendCarState();
};