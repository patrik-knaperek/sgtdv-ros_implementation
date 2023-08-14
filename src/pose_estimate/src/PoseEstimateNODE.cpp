/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include <ros/ros.h>
#include "../include/PoseEstimate.h"
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
//#include  IMU msg

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_estimate");
    ros::NodeHandle handle;

    ros::Publisher posePublisher = handle.advertise<sgtdv_msgs::CarPose>("pose_estimate", 1);
    ros::Publisher velocityPublisher = handle.advertise<sgtdv_msgs::CarVel>("velocity_estimate", 1);

    PoseEstimate poseEstimate(posePublisher, velocityPublisher);

    // ros::Subscriber slamSub = handle.subscribe("slam_pose", 1, &PoseEstimate::DoSlamState, &poseEstimate);
    //ros::Subscriber imuSub = handle.subscribe("imu", 1, &PoseEstimate::DoIMU, &poseEstimate);
    // ros::Subscriber cameraSub = handle.subscribe("camera_pose", 1, &PoseEstimate::DoCameraPose, &poseEstimate);
    ros::Subscriber odomSub = handle.subscribe("odometry/filtered", 1, &PoseEstimate::DoOdometry, &poseEstimate);

    ros::spin();

    return 0;
}
