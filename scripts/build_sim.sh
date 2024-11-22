#!/bin/bash

source ${FSD_ROOT}/devel/setup.bash

catkin build \
    master \
    path_planning \
    path_tracking \
    fusion \
    mapper \
    data_visualization \
    debug_visualization \
    cone_detection_si \
    control_si \
    slam_si \
    -DCMAKE_BUILD_TYPE=Release


source ${SGT_ROOT}/devel/setup.bash