# hcr: Gazebo + RViz demo workspace

这是一个 ROS1 Noetic 的 catkin 工作空间（或者可放在你 workspace 的 `src/` 目录下作为子树）。

它会一键拉起：

- Gazebo 中的 Pioneer 3-AT 风格差速/滑移转向小车（带 RGB-D 深度相机）。
- `map_manager` 用深度图做 voxel mapping 和动态障碍检测（发布 PointCloud2 和 MarkerArray）。
- `move_base` 使用 `map_manager` 的膨胀 PointCloud2 作为障碍源，生成 local/global costmap。
- `target_follower` 根据目标点不断给 `move_base` 发送导航 goal，让车跟随目标。

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

4) 运行一键仿真：

```bash
roslaunch p3at_gazebo bringup_all_sim.launch
```

会同时启动 Gazebo, map_manager, move_base, target follower 以及 RViz。

默认 RViz 配置：

- `p3at_gazebo/rviz/p3at_hcr.rviz`

## Key topics

- Depth image: `/camera/depth`
- Odometry: `/odom`
- Inflated obstacle cloud (for navigation): `/dynamic_map/inflated_voxel_map`
- Dynamic target position (real, from map_manager): `/dynamic_map/dynamic_pos`
- Target used by follower (muxed): `/dynamic_map/target_pos`
- Dynamic boxes (MarkerArray): `/dynamic_map/box_visualization_marker`

补充：
- move_base global costmap: `/move_base/global_costmap/costmap`
- move_base local costmap: `/move_base/local_costmap/costmap`

---

## 你在 Gazebo 里看到的红色 box 会自动移动吗

默认不会。

Gazebo world 里包含一个名为 `target_box` 的红色方块，用来当“可追踪目标”的视觉/深度输入。
如果你只运行 `bringup_all_sim.launch`，它是静止的。

如果你希望它在 Gazebo 里真实运动（方便触发动态检测），启动下面这个节点：

```bash
roslaunch p3at_gazebo move_target_box.launch
```

它通过 `/gazebo/set_model_state` 让 `target_box` 绕圈移动。
可调参数在 `p3at_gazebo/scripts/move_target_box.py`，常用的有：

- `model_name` (默认 `target_box`)
- `center_x`, `center_y`
- `radius`
- `omega` (角速度 rad/s)

---

## 为什么车会追目标，即使红色 box 没动

这是因为默认包含了一个目标点 mux: `target_pos_mux`。

- 如果 `map_manager` 在 `/dynamic_map/dynamic_pos` 上持续发布了真实目标点，mux 会转发它。
- 如果没有真实目标点（比如场景太静态，或者检测没触发），mux 会在 `map` 坐标系发布一个“虚拟绕圈目标点”。

`target_follower` 默认订阅 `/dynamic_map/target_pos`，因此你会看到车一直在动。

如果你想严格只追真实动态检测目标，有两种做法：

1) 直接让 follower 订阅真实话题

```bash
roslaunch target_follower follow_target.launch target_topic:=/dynamic_map/dynamic_pos
```

2) 在 `p3at_gazebo/launch/bringup_all_sim.launch` 里注释掉这行

```xml
<include file="$(find target_follower)/launch/target_pos_mux.launch" />
```

---

## 在 RViz 里看不到点云或 costmap 怎么办

正常情况下，RViz 里你应该能看到：

- RobotModel, TF
- `PointCloud2 Inflated` (默认订阅 `/dynamic_map/inflated_voxel_map`)
- `Global Costmap` 和 `Local Costmap`
- 动态 box marker

如果你只看到了车，通常是下面几类问题。

### 1) 先确认 RViz 用的是本仓库的配置

推荐直接用命令打开：

```bash
rosrun rviz rviz -d $(rospack find p3at_gazebo)/rviz/p3at_hcr.rviz
```

这个 rviz 配置里，PointCloud2 的颜色模式已经设为 AxisColor。
注意：`map_manager` 发布的 PointCloud2 通常没有 `intensity` 字段，如果你在 RViz 里选了 Intensity，点云可能会直接看不到。

### 2) 确认话题真的在发布

```bash
rostopic hz /camera/depth
rostopic hz /dynamic_map/inflated_voxel_map
rostopic hz /move_base/local_costmap/costmap
```

- `/camera/depth` 没频率，说明相机插件没起来。
- `/dynamic_map/inflated_voxel_map` 没频率，说明 map_manager 没产出。
- `/move_base/*costmap/costmap` 没频率，说明 move_base 或 costmap 配置没加载成功。

### 3) 确认 TF 没断

```bash
rosrun tf tf_echo map base_link
```

如果这里一直报找不到 transform，RViz 里的点云/costmap 也会显示失败。

---

## 关于“点云和 costmap 是不是有障碍才会出现”

- PointCloud2 话题一般会持续发布，但是如果场景几乎没有有效深度点（或者全被过滤掉），视觉上可能像“空”。
- costmap 通常也会持续发布，但只有当它的 `obstacle_layer` 观测源收到有效点云，才会出现明显的障碍和膨胀层。

如果你要快速确认 costmap 有没有在接收障碍输入：

```bash
rostopic echo -n 1 /move_base/local_costmap/costmap_updates
```

---


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
