# **PathTrackingSimInterface package**

___

© **SGT Driverless**

**Authors:** Patrik Knaperek

**Objective:** Interface between AMZ FSSIM workspace and SGT-DV workspace for independent path tracking testing.

___

**[Requires AMZ FSD skeleton & FSSIM installed](https://gitlab.com/sgt-driverless/simulation/fsd_skeleton/-/blob/sgt-noetic-devel/SGT-DV_install_man.md)**

### Related packages
* `path_tracking` (SGT-DV)
* `control_pure_pursuit` (AMZ)

### Topic conversions
* `/control/pure_pursuit/center_line [geometry_msgs::PolygonStamped] → pathplanning_trajectory [sgtdv_msgs::Point2DArr]`
* `/estimation/slam/state [fsd_common_msgs::CarState] → pose_estimate [sgtdv_msgs::CarPose], velocity_estimate [sgtdv_msgs::CarVel]`
* `pathtracking_commands [sgtdv_msgs::Control] → sgt/control_command [fsd_common_msgs::ControlCommand]`

## Compilation
```
$ cd <path_to_SGT_workspace>/ros_implementation
$ catkin clean
$ source <path_to_fsd_skeleton>/fsd_skeleton/devel/setup.bash
$ catkin build path_tracking_sim_interface
```
## Setup
In `fsd_skeleton/src/fssim_interface/config/config.yaml` file set parameter `fsd/cmd` to `"sgt/control_command"`.

## Launch
Run pathTrackingSimInterface node separately:
```
$ source <path_to_SGT_workspace>/ros_implementation/devel/setup.bash
$ rosrun path_tracking_sim_interface path_tracking_sim_interface
```
Launch FSSIM:

```
$ source <path_to_fsd_skeleton>/fsd_skeleton/devel/setup.bash
$ roslaunch fssim_interface fssim.launch
```
In new terminal:
```
$ source <path_to_fsd_skeleton>/fsd_skeleton/devel/setup.bash
$ roslaunch control_meta trackdrive.launch
```

For launching along with pathTracking node, check [this README](../../path_tracking/README.md).