# inspection_nav 云台 UDP 对接说明

本文面向云台开发人员，说明巡检程序需要的对接内容与返回要求。

## 对接目标

- 巡检程序在巡检点位到达后，通过 UDP 通知云台执行动作。
- 巡检程序需要云台返回“是否允许继续导航”的结果。

## 触发条件

- 仅当点位 gimbal_inspect=1 且参数 enable_gimbal_action=true 时发送。
- 未收到允许继续的响应时，巡检程序保持原地等待并持续重试。

## 传输方式

- UDP 单播，ASCII 文本。
- 目标地址：gimbal_udp_ip:gimbal_udp_port（默认 127.0.0.1:20001）。
- 云台程序必须回复到请求的源 IP/端口。

## 巡检程序发送的请求

```
INSPECT,<route_id>,<goal_index>,<goal_total>
```

- 请求仅包含路线与序号信息，不包含坐标。

字段说明：

- INSPECT: 固定命令字，表示巡检点位到达。
- route_id: 路线编号（int）。
- goal_index: 当前点位序号，从 1 开始（int）。
- goal_total: 当前路线点位总数（int）。

## 云台程序需要返回的响应

- CONTINUE / OK / 1 / TRUE：允许巡检程序继续发送下一个点位。
- 其他任意内容：巡检程序继续等待，并按重试间隔再次发送请求。

## 超时与重试

- gimbal_udp_timeout_sec：单次等待响应超时。
- gimbal_udp_retry_interval_sec：超时或未允许时的重试间隔。

## 示例

请求示例：

```
INSPECT,1,3,6
```

含义说明：

- route_id=1: 当前是第 1 条路线。
- goal_index=3: 当前到达第 3 个点位。
- goal_total=6: 该路线一共有 6 个点位。

允许继续示例：

```
CONTINUE
```

继续等待示例：

```
BUSY
```
