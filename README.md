# inspection_nav

一个基于 ROS1 Noetic 的固定目标巡检导航包，包含三个独立可执行程序：

1. goal_recorder_node：目标点录入程序
2. goal_sender_node：目标点发送程序（基于 move_base）
3. route_path_publisher_node：路径可视化发布程序（发布 nav_msgs/Path）

## 多路线功能

程序支持录入和导航多条巡检路线。路线编号通过 roslaunch 参数指定：

1. roslaunch inspection_nav goal_recorder.launch route_id:=1
表示录入第 1 条路线，写入 config/point1.yaml（文件不存在会自动创建）。

2. roslaunch inspection_nav goal_sender.launch route_id:=1
表示导航第 1 条路线，从 config/point1.yaml 读取目标点。

3. roslaunch inspection_nav goal_recorder.launch route_id:=2
表示录入第 2 条路线，写入 config/point2.yaml。

4. roslaunch inspection_nav goal_recorder.launch
无参数时默认使用第 1 条路线（config/point1.yaml）。

5. roslaunch inspection_nav goal_sender.launch
无参数时默认读取第 1 条路线（config/point1.yaml）。

终端会明确显示当前是“第几条路线”的录入或导航。

## 路径可视化功能（独立程序）

该功能由独立节点 route_path_publisher_node 实现：

1. 启动时可通过位置参数 N 直接选择显示第 N 条路线。
2. 也可在运行中接收 send 节点下发的“路线号 + 开关”指令。
3. 按目标点顺序把各点直线连接为一条 nav_msgs/Path 并发布到 /inspection_nav/patrol_path。

### 最终使用方法

1. 仅显示第 1 条路径：
roslaunch inspection_nav route_path_publisher.launch route_id:=1

2. 仅显示第 2 条路径：
roslaunch inspection_nav route_path_publisher.launch route_id:=2

3. 不带参数启动（默认显示第 1 条路径）：
roslaunch inspection_nav route_path_publisher.launch

4. 与发送节点同步显示第 N 条路径：
先启动路径发布节点（可不带参数）：
roslaunch inspection_nav route_path_publisher.launch
然后启动发送节点并开启可视化：
roslaunch inspection_nav goal_sender.launch route_id:=N enable_path_visualization:=true

5. 若关闭可视化开关，则发送时不触发路径显示：
roslaunch inspection_nav goal_sender.launch route_id:=N enable_path_visualization:=false

RViz 显示方式：

1. Add -> By topic -> 选择 /inspection_nav/patrol_path（类型 Path）
2. 在 Path 显示项里把 Color 设置为紫色（例如 RGB 128,0,128）

说明：Path 消息本身不携带颜色，颜色由 RViz 显示配置控制。

## 目录结构

- include/inspection_nav/inspection_nav_common.hpp：公共结构体、颜色常量、路径函数与文件读写声明
- include/inspection_nav/goal_recorder.hpp：录入程序类声明
- include/inspection_nav/goal_sender.hpp：发送程序类声明
- src/goal_file_io.cpp：YAML 目标点读写实现
- src/goal_recorder.cpp：录入逻辑实现
- src/goal_sender.cpp：发送逻辑实现
- src/route_path_publisher.cpp：路径发布逻辑实现
- src/record.cpp：录入程序入口
- src/send.cpp：发送程序入口
- config/point1.yaml、config/point2.yaml ...：不同路线的目标点配置

## 目标点文件格式

goals:
  - index: 1
    name: patrol_point_1
    position: {x: 1.0, y: 2.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}

## 编译

在工作空间根目录执行：

source /opt/ros/noetic/setup.bash
cd /home/schwein/inspection_ws
catkin_make
source devel/setup.bash

## 使用

### 1) 录入第 N 条路线

roslaunch inspection_nav goal_recorder.launch route_id:=N

录入方式：

1. 在 RViz 中向 /clicked_point 发布点，自动写入当前路线文件。
2. 在录入程序终端按一次回车，写入当前 /odom 位姿。

录入成功会输出绿色高亮提示。

### 2) 导航第 N 条路线

先启动导航栈和 move_base，再执行：

roslaunch inspection_nav goal_sender.launch route_id:=N

如需开启云台联动：

roslaunch inspection_nav goal_sender.launch route_id:=N enable_gimbal_action:=true

程序将按顺序读取当前路线文件中的目标点并依次发送。
每个目标到达成功会输出绿色高亮提示。

## 可选参数

仍可通过 roslaunch 参数覆盖默认配置：

1. config_file:=/path/to/custom.yaml
2. goal_frame:=map
3. goal_timeout_sec:=0.0（0 表示不超时）
4. continue_on_failure:=false（失败后是否继续）
5. enable_path_visualization:=false（发送节点是否开启路径可视化触发）
6. enable_gimbal_action:=false（是否开启云台联动，默认关闭）

## 云台联动接口（新增）

当机器人到达每一个目标点时，发送节点会调用云台控制服务，等待云台返回继续执行指令后再前往下一个目标点。该功能默认关闭，需通过 roslaunch 参数开启。

### 服务名称

- 默认：/gimbal/execute_action
- 可通过参数设置：gimbal_service_name:=/your/service

### 服务类型

inspection_nav/GimbalAction

请求字段：

- std_msgs/Header header（frame_id 为导航坐标系，stamp 为发送时间）
- int32 route_id（当前路线编号）
- int32 goal_index（当前目标点序号，从 1 开始）
- int32 goal_total（本路线目标点总数）
- geometry_msgs/Pose target_pose（当前目标点位姿）
- string action_name（云台动作名，默认 inspect）

响应字段：

- bool success（云台是否成功执行）
- bool continue_nav（是否允许继续导航，true 表示继续）
- string message（执行结果说明）

### 行为约定

1. 发送节点在每个目标点到达后调用一次服务。
2. 若服务不可用或调用失败，发送节点会持续重试并等待。
3. 若响应 continue_nav 为 true，则进入下一个目标点；否则持续等待并重试。

### 示例（云台侧伪代码）

```cpp
bool onGimbalAction(inspection_nav::GimbalAction::Request& req,
                    inspection_nav::GimbalAction::Response& res) {
  // TODO: 按 req.action_name 执行云台动作
  res.success = true;
  res.continue_nav = true;
  res.message = "ok";
  return true;
}
```

### 可选参数

1. enable_gimbal_action:=false
2. gimbal_service_name:=/gimbal/execute_action
3. gimbal_action_name:=inspect
