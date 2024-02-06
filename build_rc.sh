#!/bin/bash

catkin build master camera_cone_detection lidar_cone_detection robot_localization path_planning path_tracking fusion mapper velodyne pose_estimate ptp_trajectory visual_odometry -DCMAKE_BUILD_TYPE=Release
source ${SGT_ROOT}/ros_implementation/devel/setup.bash
