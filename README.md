# Air-Ground Fire Simulation Workspace

一个基于 ROS Noetic、Gazebo、TARE、地面车自主探索和无人机局部避障的空地协同仿真工作空间，目前还未添加fire相关任务。当前系统的核心目标是：让 UGV 在未知/半未知环境中进行地面探索，同时 UAV 实时跟随 UGV，并利用自身 3D LiDAR 做局部避障。后续将继续添加空地协同定位与目标检测、定位（协同建图？）目前在forest环境中测试效果不错其他环境待进一步测试优化。。。

本仓库整合并改造了以下模块：

- `aerial_navigation_development_environment`：无人机仿真、局部规划、路径跟随、RGB-D，添加了3D LiDAR 传感器模型，修改对应话题为/uav/...
- `autonomous_exploration_development_environment`：无人车仿真、地形分析、地面局部规划、传感器扫描生成,修改对应话题为/ugv/...
- `tare_planner`：TARE 地面探索规划器,主要用于无人车探索任务，对应话题修改为/ugv/...
- `system_bringup`：本工作空间的集成启动包，负责同时启动 Gazebo、UAV、UGV、TARE 和 UAV 跟随节点。

## Features

- Gazebo 中同时生成 UAV 和 UGV。
- UGV 使用 TARE 进行探索规划。
- UAV 使用 3D LiDAR 点云进行局部避障和路径跟随。
- UAV 实时读取 UGV 位姿，跟随在 UGV 运动方向后方约 2 m。
- 提供用于空地协同仿真的统一启动入口：`system_bringup/launch/robot.launch`。

## System Overview

核心数据流如下：

```text
UGV simulator / planner
  -> /ugv/state_estimation
  -> TARE exploration and ground local planner

follow_node
  <- /ugv/state_estimation
  <- /uav/state_estimation
  -> /uav/way_point
  -> /uav/speed
  -> /airground/ugv_history_path

UAV local planner
  <- /uav/way_point
  <- /uav/speed
  <- /uav/lidar/points
  -> /uav/path

UAV path follower / simulator
  <- /uav/path
  -> /uav/attitude_control
  -> /uav/state_estimation
```

`follow_node` 只负责生成 UAV 的跟随目标点和速度比例；真正决定 UAV 能不能继续飞的是 `localPlanner_uav`。如果局部规划器认为没有安全路径，`pathFollower_uav` 会主动停车保护。

## Repository Layout

```text
.
├── src
│   ├── system_bringup
│   │   ├── launch
│   │   │   ├── robot.launch          # 一键启动空地协同系统
│   │   │   └── follow.launch         # UAV 跟随 UGV 的参数
│   │   └── src/follow_node.cpp       # UAV 跟随节点
│   ├── aerial_navigation_development_environment
│   │   └── src
│   │       ├── local_planner_uav      # UAV 局部避障和路径跟随
│   │       └── vehicle_simulator_uav  # UAV Gazebo 仿真和传感器模型
│   ├── autonomous_exploration_development_environment
│   │   └── src
│   │       ├── local_planner          # UGV 局部规划
│   │       ├── vehicle_simulator      # UGV Gazebo 仿真
│   │       ├── terrain_analysis       # 地形分析
│   │       └── sensor_scan_generation # 点云/扫描生成
│   └── tare_planner
│       └── src/tare_planner           # TARE 探索规划器
├── build
└── devel
```

## Requirements

测试环境：

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- CMake / catkin
- PCL、OpenCV、Eigen
- RViz
- Gazebo ROS plugins
- Velodyne Gazebo plugins
- Google glog
- TARE 使用的 OR-Tools 库

常用依赖安装示例：

```bash
sudo apt update
sudo apt install \
  ros-noetic-desktop-full \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-velodyne-gazebo-plugins \
  ros-noetic-pcl-ros \
  ros-noetic-tf \
  ros-noetic-tf2-ros \
  ros-noetic-rviz \
  ros-noetic-joy \
  ros-noetic-xacro \
  libgoogle-glog-dev \
  libusb-dev \
  libpcl-dev \
  libopencv-dev
```

如果系统中缺少某个 ROS 包，请按报错中的包名补装对应的 `ros-noetic-*` 包。

## Build

在工作空间根目录编译：

```bash
cd ~/airground_fire_sim_ws
catkin_make
source devel/setup.bash
```

## Quick Start

启动完整空地协同仿真：

```bash
cd ~/airground_fire_sim_ws
source devel/setup.bash
roslaunch system_bringup robot.launch
```

默认会启动：

- Gazebo 空环境或指定 world。
- UAV 机体、3D LiDAR、RGB-D camera。
- UGV 机体、LiDAR、camera。
- UAV 局部规划器和路径跟随器。
- UGV 地形分析、局部规划器。
- TARE 探索规划器。
- UAV 跟随节点 `follow_node`。
- RViz 可视化窗口。

## Changing Environments

`robot.launch` 中主要有两个环境参数：

- `world_name`：Gazebo world 文件路径。
- `explore_world`：TARE 使用的配置名称。

示例：

```bash
roslaunch system_bringup robot.launch \
  world_name:=$(rospack find vehicle_simulator_uav)/world/forest.world \
  explore_world:=forest
```

可用环境通常包括：

- `forest`
- `campus`
- `garage`
- `indoor`
- `tunnel`

切换环境时，建议让 `world_name` 和 `explore_world` 保持一致，否则 Gazebo 场景和 TARE 参数可能不匹配。

## UAV Follow Behavior

UAV 跟随参数位于：

```text
src/system_bringup/launch/follow.launch
```

关键参数：

```xml
<param name="follow_mode" value="direct" />
<param name="follow_height" value="1.5" />
<param name="direct_follow_offset" value="2.0" />
<param name="direct_follow_offset_mode" value="motion" />
<param name="publish_speed" value="true" />
<param name="follow_speed_max_ratio" value="0.7" />
```

含义：

- `follow_mode=direct`：实时根据 UGV 当前位姿计算 UAV 跟随目标。
- `direct_follow_offset=2.0`：目标点位于 UGV 后方约 2 m。
- `direct_follow_offset_mode=motion`：以后方运动方向为参考，而不是固定世界坐标 x 方向。
- `follow_height=1.5`：UAV 目标高度。
- `publish_speed=true`：跟随节点会向 `/uav/speed` 发布速度比例。

如果希望 UAV 飞得更高，可以增大 `follow_height`；如果希望 UAV 离 UGV 更远，可以增大 `direct_follow_offset`。

## UAV Local Planner Tuning

UAV 局部规划参数位于：

```text
src/aerial_navigation_development_environment/src/local_planner_uav/launch/local_planner_outdoor.launch
```

当前对 3D LiDAR 跟随场景比较关键的参数：

```xml
<param name="cloudPointFormat" type="string" value="lidar" />
<param name="pointPerPathThre" type="int" value="3" />
<param name="maxRange" type="double" value="8.0" />
<param name="keepSurrCloud" type="bool" value="false" />
<param name="lowerBoundZ" type="double" value="-1.0" />
<param name="upperBoundZ" type="double" value="1.5" />
```

这些参数用于过滤参与避障判断的点云。对于飞行高度约 `1.5 m` 的 UAV，地面点在 UAV 局部坐标里大约是 `z=-1.5`。如果 `lowerBoundZ` 设置得过低，比如 `-4.8`，地面点会被当成障碍物，导致所有候选路径被堵死，UAV 会停在原地。

排查局部规划问题时重点看日志：

```text
UAV local planner has no valid path. planner_cloud=... clear_paths=0 selected_group=-1
UAV stops because local planner has no valid path.
```

如果出现 `clear_paths=0`，说明局部规划器没有找到任何可行路径。优先检查：

- `lowerBoundZ` / `upperBoundZ` 是否把地面点纳入了障碍物。
- `keepSurrCloud` 是否保留了过多旧点云。
- `pointPerPathThre` 是否过小，导致少量噪声点就堵死路径。
- `/uav/lidar/points` 是否正常发布。
- Gazebo 中 UAV LiDAR eUAV 运动。

## Useful Topics

```text
/ugv/state_estimation          UGV 位姿估计
/uav/state_estimation          UAV 位姿估计
/uav/lidar/points              UAV 3D LiDAR 点云
/uav/way_point                 UAV 跟随目标点
/uav/speed                     UAV 跟随速度比例
/uav/path                      UAV 局部规划路径
/uav/attitude_control          UAV 姿态/速度控制命令
/airground/ugv_history_path    UGV 历史轨迹可视化
```

常用检查命令：

```bash
rostopic hz /uav/lidar/points
rostopic echo /uav/way_point
rostopic echo /uav/speed
rostopic echo /uav/path
rostopic echo /ugv/state_estimation
rostopic echo /uav/state_estimation
```


## Troubleshooting

### UAV runs for a while and then stops

先判断是谁让 UAV 停车：

```text
UAV stops because forward speed command is zero.
```

表示 `/uav/speed` 为 0，检查 `follow_node` 速度逻辑。

```text
UAV stops because it is within stopDis of the goal.
```

表示 UAV 已接近目标点，检查 `stopDis`、目标点和跟随距离。

```text
UAV stops because local planner has no valid path.
```

表示局部规划器没有安全路径。继续查 `localPlanner_uav` 日志中的 `clear_paths`。如果 `clear_paths=0`，优先调 UAV 点云过滤参数。

### Gazebo spawn service timeout

有时会看到：

```text
Spawn service failed. Exiting.
```

但如果随后又看到：

```text
Velodyne laser plugin ready
```

并且 `/uav/lidar/points` 正常发布，说明 LiDAR 插件实际上已经加载。此时重点检查传感器话题和模型状态，不要只看 spawn 节点退出码。

### TF_REPEATED_DATA warnings

如果看到：

```text
TF_REPEATED_DATA ignoring data with redundant timestamp
```

说明某个节点正在用重复时间戳发布 TF/里程计。检查对应 vehicle simulator 的时间戳更新逻辑，确保仿真时间单调递增。


## Upstream Projects and Credits

本工作空间基于并整合了以下开源/研究项目：

- Aerial Navigation Development Environment
- Autonomous Exploration Development Environment
- TARE Planner
- Velodyne Gazebo simulator/plugins

