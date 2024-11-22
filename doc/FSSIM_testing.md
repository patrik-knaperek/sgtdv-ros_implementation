# FSSIM testing
___

© **SGT Driverless**

Guidelines for testing SGT-DV ROS implementation with FSSIM.
___

## Introduction

Particular nodes, groups of nodes or whole AS can be tested with FS Simulator by AMZ-Driverless connected through packages from `simulation_interface` submodule.

<p align="center">
  <img src="./DV_architecture-ROS_fssim_setup.svg" width="800">
</p>
<figcaption align = "center">ROS nodes and topics - FSSIM setup</figcaption>

__To work with FSSIM__:
1. Follow [FSD skeleton and FSSIM installation steps](https://gitlab.com/sgt-driverless/simulation/fsd_skeleton/-/blob/master/README.md?ref_type=heads).

2. In new terminal run

    ```sh
    $ cd ${SGT_ROOT}
    $ FSD_source
    $ ./scripts/build_sim.sh
    ```
### Significant FSSIM/FSD nodes description
* `gazebo` : Runs simulation engines with custom plugins for vehicle model - computing vehicle state - and sensors' models - generating cone detections. The simulation depends on vehicle model parameters, sensor parameters and track configuration file.
* `fssim_interface` : Provides topic interface between simulation (Gazebo) and autonomous system.
* `control_pure_pursuit_node` : Based on vehicle state and map both obtained from Gazebo, generates a center line trajectory and computes control commands allowing the vehicle to pass the track.

### Configuration options

* **AMZ FSD skeleton**
  * set the parameter `/fsd/cmd` in the `fssim_interface/config/config.yaml` file according to current control source
    * `"/sgt/control_command"` if using SGT-DV `path_tracking_node`
    * `"/control/pure_pursuit/control_command"` if using AMZ FSD `control_pure_pursuit_node`
  * check [FSD skeleton README](https://gitlab.com/sgt-driverless/simulation/fsd_skeleton/-/blob/master/README.md?ref_type=heads) for further FSSIM configuration options.

* **SGT-DV ROS implementation**
  * Cone Detection Fusion : [fusion_sim.yaml](../src/fusion/params/fusion_sim.yaml) - frame IDs of camera and lidar detections must match FSSIM config.
  * Mapper : [mapper.yaml](../src/mapper/config/mapper.yaml)
  * Path Planning : [path_planning_sim.yaml](../src/path_planning/params/path_planning_sim.yaml) - set vehicle speed during the first lap (`slow`) and next laps (`fast`)
  * Path Tracking : [path_tracking_sim.yaml](../src/path_tracking/params/path_tracking_sim.yaml) - car parameters must match FSSIM config. Set speed and steering controller parameters.

In the next sections, various testing setups are described.

## Trackdrive test (complete AS)
Test and visualize the whole SGT Driverless autonomous system with FSSIM providing the physical layer simulation (sensor measurements and vehicle state feedback) on Trackdrive mission.

### Interface description

* **Inputs:**
  * `/fssim/camera/cones` : camera cone detections (auto-generated from SDF track file)
  * `/fssim/lidar/cones` : LiDAR cone detections (auto-generated from SDF track file)
  * `/estimation/slam/state` : vehicle pose
  * `/estimation/velocity_estimation/velocity_estimation` : vehicle velocity

* **Outputs:**
  * `sgt/control_command` : control the simulated vehicle

* **AMZ-FSD nodes:**
  * `gazebo`
  * `fssim_interface`

* **SGT-DV nodes:**
  * `fusion`
  * `slam` (`mapper`)
  * `path_planning`
  * `path_tracking`
  * `data_visualization`
  * `cone_detection_si`
  * `control_si`

### Launch
1. In the 1st terminal, run:
   ```sh
   $ FSD_source
   $ roslaunch fssim_interface fssim.launch
   ```
2. Wait for "Sending RES GO" log message. Then, in the 2nd terminal run:
    ```sh
    $ SGT_source
    $ roslaunch master trackdrive_sim.launch
    ```
3. Start the vehicle by calling service (in the 3rd terminal):
    ```sh
    $ SGT_source
    $ rosservice call /path_tracking/start "{}"
    ```
## Perception & SLAM test
Test the cone detection fusion and SLAM algorithms on simulated detections, letting the FSD control node to drive the vehicle.

### Interface description
* **Inputs:**
  * `/fssim/camera/cones` : camera cone detections (auto-generated from SDF track file)
  * `/fssim/lidar/cones` : LiDAR cone detections (auto-generated from SDF track file)
  *  `/estimation/slam/state` : vehicle pose
  * `/estimation/velocity_estimation/velocity_estimation` : vehicle velocity
  
* **Outputs:**
  * None

* **AMZ-FSD nodes:**
  * `gazebo` : simulation engine, responsible for cone detections generations and vehicle state computation
  * `fssim_interface` : interface between FSD autonomous system and Gazebo simulation
  * `control_pure_pursuit_node`

* **SGT-DV nodes:**
  * `fusion`
  * `slam` (`mapper`)
  * `data_visualization`
  * `cone_detection_si`

### Launch
1. In the 1st terminal, run:
   ```sh
   $ FSD_source
   $ roslaunch fssim_interface fssim.launch
   ```
2. Wait for "Sending RES GO" log message. Then, in the 2nd terminal run:
    ```sh
    $ SGT_source
    $ roslaunch master perception_sim.launch
    ```
3. Start the vehicle by running (in the 3rd terminal):
    ```sh
    $ FSD_source
    $ roslaunch control_meta trackdrive.launch
    ```
## Navigation test
Test Path Planning and Path Tracking on simulated SLAM data - avoiding possible bugs in the Perception and SLAM part of the system.

### Interface description

* **Inputs:**
  * `/estimation/slam/map` : cone positions
  * `/estimation/slam/state` : vehicle pose
  * `/estimation/velocity_estimation/velocity_estimation` : vehicle velocity

* **Outputs:**
  * `sgt/control_command` : control the simulated vehicle

* **AMZ-FSD nodes:**
  * `gazebo`
  * `fssim_interface`

* **SGT-DV nodes:**
  * `path_planning`
  * `path_tracking`
  * `data_visualization`
  * `slam_si`
  * `control_si`

### Launch
1. In the 1st terminal, run:
   ```sh
   $ FSD_source
   $ roslaunch fssim_interface fssim.launch
   ```
2. Wait for "Sending RES GO" log message. Then, in the 2nd terminal run:
    ```sh
    $ SGT_source
    $ roslaunch master navigation_sim.launch
    ```
3. Start the vehicle by calling service (in the 3rd terminal):
    ```sh
    $ SGT_source
    $ rosservice call /path_tracking/start "{}"

## Single node tests
[Fusion](../src/fusion/README.md), [Mapper](../src/mapper/README.md), [Path Planning](../src/path_planning/) and [Path Tracking](../src/path_tracking/README.md) nodes can be tested with FSSIM independently as well. Check their READMEs for more info.