#!/bin/bash

source ${FSD_ROOT}/devel/setup.bash
catkin build master path_planning path_tracking calibration fusion mapper visualizator fusion_sim_interface path_tracking_sim_interface slam_sim_interface -DCMAKE_BUILD_TYPE=Release
source ${SGT_ROOT}/devel/setup.bash