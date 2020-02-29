#include <ros/ros.h>
#include "../include/PoseEstimate.h"
#include <sgtdv_msgs/CarState.h>
//#include  IMU msg

int main(int argc, char** argv)
{
    PoseEstimate poseEstimate;

    ros::init(argc, argv, "pose_estimate");
    ros::NodeHandle handle;

    ros::Publisher publisher = handle.advertise<sgtdv_msgs::CarState>("pose_estimate", 1);

    poseEstimate.SetPublisher(publisher);

    ros::Subscriber cameraSub = handle.subscribe("slam_pose", 1, &PoseEstimate::DoSlamState, &poseEstimate);
    //ros::Subscriber lidarSub = handle.subscribe("imu", 1, &PoseEstimate::DoIMU, &poseEstimate);

    ros::spin();

    return 0;
}
