# **SLAMSimInterface package**

___

© **SGT Driverless**

**Authors:** Patrik Knaperek

**Objective:** Interface between AMZ FSSIM workspace and SGT-DV workspace for SLAM algorithm subtitute.

___

**[Requires AMZ FSD skeleton & FSSIM installed](https://gitlab.com/sgt-driverless/simulation/fsd_skeleton/-/blob/sgt-noetic-devel/SGT-DV_install_man.md)**

### Related packages
* `path_planning` (SGT-DV)
* `slam_slam` (AMZ)

### Topic conversions
* `/estimation/slam/map [fsd_common_msgs::Map] → /slam_map [sgtdv_msgs::ConeArr]`
* `/estimation/slam/state [fsd_common_msgs::CarState] → /slam_pose [sgtdv_msgs::CarPose], velocity_estimate [sgtdv_msgs::CarVel]`

## Compilation
```
$ cd <path_to_SGT_workspace>/ros_implementation
$ catkin clean
$ source <path_to_fsd_skeleton>/fsd_skeleton/devel/setup.bash
$ catkin build slam_sim_interface
```
## Setup


## Launch
Run SLAMSimInterface node separately:
```
$ source <path_to_SGT_workspace>/ros_implementation/devel/setup.bash
$ rosrun slam_tracking_sim_interface slam_tracking_sim_interface
```
Launch FSSIM:

```
$ source <path_to_fsd_skeleton>/fsd_skeleton/devel/setup.bash
$ roslaunch fssim_interface fssim.launch
```

For launching along with pathPlanning node, check [this README](../../path_planning/README.md).