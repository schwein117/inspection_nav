# inspection_nav

基于 ROS1 Noetic 的巡检导航包，提供路线录入、路线发送和路径可视化三个节点。

## 环境要求

- Ubuntu 20.04
- ROS1 Noetic

## 功能说明

- goal_recorder_node：录入巡检路线目标点
- goal_sender_node：读取路线并按顺序发送给 move_base
- route_path_publisher_node：把路线点连接成 nav_msgs/Path 并发布到 /inspection_nav/patrol_path

## 编译

在工作空间根目录执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/schwein/inspection_ws
catkin_make
source devel/setup.bash
```

## 路线文件

路线文件位于：

```bash
/inspection_ws/src/inspection_nav/config/pointN.yaml
```

其中 N 为路线编号，默认是 1。

目标点格式如下：

```yaml
goals:
  - index: 1
    name: patrol_point_1
    position: {x: 1.0, y: 2.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

## 使用方法

### 1. 录入路线

录入第 N 条路线：

```bash
roslaunch inspection_nav goal_recorder.launch route_id:=N
```

默认不带参数时录入第 1 条路线。

录入方式有两种：

1. 在 RViz 中使用 Publish Point 发布到 /clicked_point。
2. 在录入终端按回车，记录当前 /odom 位姿。

### 2. 导航路线

启动 move_base 后，发送第 N 条路线：

```bash
roslaunch inspection_nav goal_sender.launch route_id:=N
```

默认不带参数时发送第 1 条路线。

常用参数：

```bash
roslaunch inspection_nav goal_sender.launch route_id:=N   goal_frame:=map   goal_timeout_sec:=0.0   continue_on_failure:=false   enable_gimbal_action:=true
```

说明：

- `goal_timeout_sec:=0.0` 表示不超时。
- `enable_gimbal_action` 在 launch 中默认是 `true`，可手动关闭。
- `gimbal_service_name` 默认是 `/gimbal/execute_action`。
- `gimbal_action_name` 默认是 `inspect`。

### 3. 路径可视化

启动路径发布节点：

```bash
roslaunch inspection_nav route_path_publisher.launch route_id:=N
```

默认不带参数时显示第 1 条路线。

如果希望由发送节点同步触发路径显示，先启动路径发布节点，再启动发送节点并打开可视化：

```bash
roslaunch inspection_nav route_path_publisher.launch
roslaunch inspection_nav goal_sender.launch route_id:=N enable_path_visualization:=true
```

路径会发布到 /inspection_nav/patrol_path，可在 RViz 中add topic可添加 Path 话题显示。

## 云台联动

goal_sender 会在每个目标点到达后调用云台服务 /gimbal/execute_action，服务类型为 inspection_nav/GimbalAction。

如果服务返回 continue_nav=true，程序会继续下一个目标点；否则会等待服务允许继续。

## 目录结构

```text
inspection_nav/
├── config/
├── include/inspection_nav/
├── launch/
├── src/
└── README.md
```

## 常见问题

1. 如果路线文件为空或不存在，goal_sender 会读取失败。
2. 如果 /clicked_point 没有写入，先确认 RViz 已连接到正确的 ROS master。
3. 如果云台联动无反应，先确认 /gimbal/execute_action 服务已启动。
