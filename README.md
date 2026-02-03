# hcr: Gazebo + RViz demo workspace

This repo is a ROS1 Noetic catkin workspace (or a `src/` subtree) that brings up:

- A Pioneer 3-AT style skid-steer robot in Gazebo, with an RGB-D depth camera.
- `map_manager` for voxel mapping and dynamic obstacle detection (publishes PointCloud2 and MarkerArray).
- `move_base` configured to use the `map_manager` inflated PointCloud2 as an obstacle source.
- A simple `target_follower` node that repeatedly sends goals to `move_base` based on the detected dynamic target.

## Quick start

1) Put this repo inside your catkin workspace `src/`:

- `~/catkin_ws/src/hcr`

2) Install dependencies (typical set):

- ROS Noetic desktop-full
- `gazebo_ros_pkgs`, `gazebo_plugins`
- `pcl_ros`, `pcl_conversions`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `image_transport`, `message_filters`, `cv_bridge`

3) Build:

- `cd ~/catkin_ws && catkin_make`
- `source devel/setup.bash`

4) Run the full simulation:

- `roslaunch p3at_gazebo bringup_all_sim.launch`

This launches Gazebo, map_manager, move_base, target follower, and RViz with a working config:

- `p3at_gazebo/rviz/p3at_hcr.rviz`

## Key topics

- Depth image: `/camera/depth`
- Odometry: `/odom`
- Inflated obstacle cloud (for navigation): `/dynamic_map/inflated_voxel_map`
- Dynamic target position: `/dynamic_map/dynamic_pos`
- Dynamic boxes (MarkerArray): `/dynamic_map/box_visualization_marker`


---

# Integration Notes: Using `map_manager_pub` in a P3AT Ground Robot Project

This document describes how we use the upstream repository `Shawn207/map_manager_pub` as a component inside our ROS1 Noetic simulation and navigation stack.

Repo reference (upstream): `https://github.com/Shawn207/map_manager_pub`


## 1. Upstream capability and what we take from it

### 1.1 What the upstream repo provides
According to the upstream README, `map_manager_pub` is a real-time 3D dynamic mapping system that builds a 3D occupancy representation from an RGB-D camera, detects and tracks dynamic obstacles, and publishes visualization and trajectory prediction outputs for navigation and collision avoidance.

The upstream README also states the expected inputs and the main published outputs:
- Inputs: a localization topic (odometry or pose) and a depth camera topic.
- Outputs: an inflated occupancy map point cloud, dynamic obstacle bounding boxes, and a predicted trajectory marker.


### 1.2 What parts of the upstream repo are utilized in our system

We utilize the following parts of the upstream repo:

1) **ROS package entry points**
- `map_manager/launch/dynamic_map.launch`  
  Starts the node and loads parameters.
- `map_manager/cfg/dynamic_map_param.yaml`  
  Configures camera intrinsics, depth topic, and localization topic.

2) **The node executable**
- `map_manager/src/dynamic_map_node.cpp`  
  Node entry point that instantiates the dynamic map object and spins.

3) **Core mapping, raycasting, and dynamic object logic**
- `map_manager/include/map_manager/`  
  Core implementation used at runtime for:
  - depth to 3D processing
  - occupancy updates
  - map inflation for visualization and planning
  - dynamic obstacle detection and tracking
  - trajectory prediction markers

4) **RViz config**
- `p3at_gazebo/rviz/p3at_hcr.rviz`  
  Used as a starting point for visualization in RViz.

In our local build, we also observe extra published debug and helper topics beyond the upstream README, including UV depth debug streams and a representative dynamic target position topic that we use for following behavior.


### 1.3 Upstream repo parts not utilized in the current pipeline
These exist in the upstream repo but are not required for the current Gazebo + move_base + target follower demo:

- `map_manager/map/*.bt`  
  Example OctoMap map files. We do not load these in the current simulation pipeline.

- `map_manager/srv/*.srv`  
  Custom services compile and are available, but we do not call them in the basic demo workflow.

- C++ API usage beyond starting the node  
  The upstream README shows a C++ API example. In our current system, we use the package as a running node rather than as an embedded library in another node.


## 2. Our project intent

### 2.1 Platform and constraints
- ROS1 Noetic on Ubuntu
- Ground robot platform: Pioneer 3-AT style skid-steer vehicle
- Sensors: RGB-D camera
- Compute: onboard Jetson Orin Nano available for later deployment

### 2.2 What the project is trying to do
We want the robot to:
1) Navigate safely in a dynamic environment (collision avoidance).
2) Track a moving object and keep following it while avoiding obstacles.
3) Validate the full pipeline first in simulation using Gazebo and RViz, then move to real hardware on the Jetson platform.


## 3. System connections and data flow

### 3.1 Packages in our workspace
Our catkin workspace includes:
- `p3at_gazebo`  
  Gazebo simulation bringup for the robot and RGB-D sensor.
- `map_manager` (from `map_manager_pub`)  
  Dynamic mapping, dynamic obstacle detection, and trajectory marker publication.
- `p3at_navigation`  
  `move_base` configuration for global and local planning, and collision avoidance.
- `target_follower`  
  High-level behavior node that turns a tracked target position into repeated goals for `move_base`.


### 3.2 Topic and control flow
High-level data flow looks like this:

```
Gazebo (p3at_gazebo)
  publishes:  /camera/depth    /odom    /tf
  subscribes: /cmd_vel
        |
        v
Dynamic mapping (map_manager dynamic_map_node)
  subscribes: /camera/depth + /odom (or /pose)
  publishes:  /dynamic_map/inflated_voxel_map
             /dynamic_map/box_visualization_marker
             /dynamic_map/traj_marker
             (plus additional debug topics)
        |
        v
Navigation (p3at_navigation move_base)
  consumes:   /tf, /odom, goals
  costmap input: /dynamic_map/inflated_voxel_map as obstacle source
  publishes:  plans, costmaps
  outputs:    /cmd_vel
        ^
        |
Target following (target_follower)
  subscribes: dynamic target position topic from map_manager (local build observation)
  sends:      moving goals to move_base
```

Interpretation:
- `map_manager` converts depth plus localization into a 3D obstacle representation.
- `move_base` performs collision avoidance using costmaps that read obstacles from the `map_manager` point cloud output.
- `target_follower` keeps updating the navigation goal based on the tracked moving object.
- RViz is used to visualize the robot, point clouds, markers, costmaps, and plans.


## 4. Expected visualization in RViz

RViz can visualize all major outputs, for example:
- Robot model and TF tree
- Point cloud obstacles from `map_manager` (inflated voxel map)
- Marker arrays for dynamic obstacle bounding boxes and predicted trajectories
- `move_base` costmaps and paths

In our setup, RViz can be started either with a saved config (`map.rviz`) or manually by adding the displays for the topics above.


## 5. Practical launch sequence (simulation stage)

Typical run order is:

1) Start simulation
- `roslaunch p3at_gazebo bringup_sim.launch`

2) Start dynamic mapping
- `roslaunch map_manager dynamic_map.launch`

3) Start navigation
- `roslaunch p3at_navigation move_base.launch`

4) Start target following
- `roslaunch target_follower follow_target.launch`

5) Start RViz
- `rosrun rviz rviz -d <path_to_config>`


## 6. Configuration points that matter

For `map_manager`:
- `depth_image_topic` must match the Gazebo depth image topic.
- `localization_mode` must match your localization type:
  - `1` for odometry (`nav_msgs/Odometry`)
  - `0` for pose (`geometry_msgs/PoseStamped`)
- Camera intrinsics must match the simulated or real camera.
- Map size, resolution, and filtering parameters should be tuned later for Jetson runtime constraints.

For `move_base`:
- The local costmap should include `/dynamic_map/inflated_voxel_map` as an observation source so obstacle avoidance reacts to the dynamic map output.
- Robot footprint or radius must match the P3AT geometry for safe planning.

For `target_follower`:
- Follow distance, update rate, and target timeout control stability and behavior when the target is lost.
