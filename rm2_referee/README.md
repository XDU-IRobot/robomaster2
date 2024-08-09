# rm2_referee

当前版本为[V1.6.3, 2024-5-31](https://rm-static.djicdn.com/tem/71710/RoboMaster%20%E8%A3%81%E5%88%A4%E7%B3%BB%E7%BB%9F%E4%B8%B2%E5%8F%A3%E5%8D%8F%E8%AE%AE%E9%99%84%E5%BD%95%20V1.6.3%EF%BC%8820240527%EF%BC%89.pdf)

## 使用

设置`referee_node/config/settings.yaml`里的配置项，然后编译、启动节点：

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch referee_node referee_node.launch.py
```

裁判系统通过串口发送的数据会被封装成消息发布到对应话题上，反之可以通过请求`/rm2_referee/tx`服务向裁判系统发送数据。

## 话题列表

### 常规链路

| 话题名                                    | 消息类型                                       | 说明                 |
| ----------------------------------------- | ---------------------------------------------- | -------------------- |
| /rm2_referee/air_support_data             | rm2_referee_msgs/msg/AirSupportData            | 空中机器人状态       |
| /rm2_referee/buff                         | rm2_referee_msgs/msg/Buff                      | 机器人增益信息       |
| /rm2_referee/dart_client_cmd              | rm2_referee_msgs/msg/DartClientCmd             | 飞镖信息 1           |
| /rm2_referee/dart_info                    | rm2_referee_msgs/msg/DartInfo                  | 飞镖信息 2           |
| /rm2_referee/event_data                   | rm2_referee_msgs/msg/EventData                 | 比赛信息 1           |
| /rm2_referee/ext_supply_projectile_action | rm2_referee_msgs/msg/ExtSupplyProjectileAction | 弹丸相关信息         |
| /rm2_referee/game_result                  | rm2_referee_msgs/msg/GameResult                | 比赛结果             |
| /rm2_referee/game_robot_hp                | rm2_referee_msgs/msg/GameRobotHP               | 双方机器人血量       |
| /rm2_referee/game_status                  | rm2_referee_msgs/msg/GameStatus                | 比赛信息 2           |
| /rm2_referee/ground_robot_position        | rm2_referee_msgs/msg/GroundRobotPosition       | 我方地面机器人位置   |
| /rm2_referee/hurt_data                    | rm2_referee_msgs/msg/HurtData                  | 扣血信息             |
| /rm2_referee/power_heat_data              | rm2_referee_msgs/msg/PowerHeatData             | 功率和枪口热量信息   |
| /rm2_referee/projectile_allowance         | rm2_referee_msgs/msg/ProjectileAllowance       | 允许发弹量和剩余金币 |
| /rm2_referee/radar_info                   | rm2_referee_msgs/msg/RadarInfo                 | 雷达双倍易伤相关信息 |
| /rm2_referee/radar_mark_data              | rm2_referee_msgs/msg/RadarMarkData             | 对方机器人被标记进度 |
| /rm2_referee/referee_warning              | rm2_referee_msgs/msg/RefereeWarning            | 判罚信息             |
| /rm2_referee/rfid_status                  | rm2_referee_msgs/msg/RFIDStatus                | RFID 检测信息        |
| /rm2_referee/robot_pos                    | rm2_referee_msgs/msg/RobotPos                  | 本机器人的位置和朝向 |
| /rm2_referee/robot_status                 | rm2_referee_msgs/msg/RobotStatus               | 本机器人状态         |
| /rm2_referee/sentry_info                  | rm2_referee_msgs/msg/SentryInfo                | 哨兵兑换信息         |
| /rm2_referee/shoot_data                   | rm2_referee_msgs/msg/ShootData                 | 弹丸信息             |
| /rm2_referee/map_command                  | rm2_referee_msgs/msg/MapCommand                | 选手端小地图交互数据 |

### 图传链路

| 话题名                         | 消息类型                             | 说明                         |
| ------------------------------ | ------------------------------------ | ---------------------------- |
| /rm2_referee/custom_robot_data | rm2_referee_msgs/msg/CustomRobotData | 自定义控制器与机器人交互数据 |
| /rm2_referee/remote_control    | rm2_referee_msgs/msg/RemoteControl   | 图传链路键鼠遥控数据         |

## 服务列表

| 服务名          | 服务类型                | 说明                   |
| --------------- | ----------------------- | ---------------------- |
| /rm2_referee/tx | rm2_referee_msgs/srv/Tx | 向裁判系统串口发送数据 |
