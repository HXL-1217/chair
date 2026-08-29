# 原始数据 rosbag 母包录制方案（不启动 EKF/SLAM/Nav2）

> 目标：录制一份干净的**原始母数据包**（双雷达 + IMU + 轮式里程计 + cmd_vel + 静态 TF），录制时**不启动** EKF / SLAM Toolbox / Nav2。
> 之后用同一份数据离线分叉做对比实验：EKF 融合 vs 纯 odom、scan_1 vs scan_1+scan_2、SLAM Toolbox vs Cartographer。
> ⚠️ 不要用 double_mapping.launch.py 边建图边录——那会引入 EKF/SLAM 节点和动态 TF，母数据就不"原始"了。

## 一、启动（只起传感器 + 底盘 + 静态 TF）

```bash
# 终端1：raw_record.launch.py = 双雷达(/scan_1,/scan_2) + IMU(/imu) + dsw_chair(/odom) + 3条静态TF
#        刻意不含 EKF/slam_toolbox/dual_laser_merger/Nav2/voice/rviz
ros2 launch jt_chair raw_record.launch.py

# 终端2：遥控（二选一）。录包期间不要开 start_web.sh / 语音节点——voice_nav_bridge 也发 /cmd_vel，会抢底盘
python3 ~/slam_ws/handle/cmd_vel_publisher_node.py   # 键盘（README 标记当前在用）
python3 ~/slam_ws/handle/joy.py                      # 手柄
```

不想用 launch 时的手动等效命令：

```bash
ros2 launch lslidar_driver lsn10p_double_launch.py    # 双雷达
ros2 launch fdilink_ahrs ahrs_driver.launch.py        # IMU
ros2 run dsw_chair dsw_chair                          # 底盘 /odom
ros2 run tf2_ros static_transform_publisher --x 0.29 --y -0.255 --z 0.2 --yaw 0 --frame-id base_link --child-frame-id laser_1
ros2 run tf2_ros static_transform_publisher --x -0.29 --y 0.255 --z 0.2 --yaw 3.124 --frame-id base_link --child-frame-id laser_2
ros2 run tf2_ros static_transform_publisher --x 0.185 --y 0 --z 0.2 --frame-id base_link --child-frame-id imu_link
```

## 二、录前体检（正式录前必做）

```bash
ros2 node list                 # 不应出现 ekf_filter_node / slam_toolbox / nav2 / dual_laser_merger / voice / rviz
ls /dev/ttyACM0 /dev/ttyACM1 /dev/wheeltec_FDI_IMU_GNSS /dev/ttyUSB0   # 四个设备都在
df -h /                        # 剩余空间够本次路线时长
ros2 topic hz /scan_1          # ~10Hz（/scan_2 同）
ros2 topic hz /imu             # 稳定非 0（fdilink 常见 ~100Hz，以实测为准）
ros2 topic hz /odom            # 20Hz（代码 50ms 定时器）
ros2 topic echo /scan_1 --once | grep frame_id   # 必须 laser_1（防 ttyACM0/1 重启后互换）
ros2 topic echo /scan_2 --once | grep frame_id   # 必须 laser_2
ros2 topic echo /odom --once   # frame_id=odom, child=base_link，pose/twist 非全 0
ros2 topic echo /imu --once    # orientation / angular_velocity 非全 0，时间戳为当前时间
ros2 run tf2_ros tf2_echo base_link laser_2      # 三条静态 TF 逐个确认能查到
```

- scan 中 88°~200° 角窗内全是 inf 是**正常**（angle_disable 配置裁掉的自遮蔽区）
- 雷达接反检查：轮椅正对一面近墙，/scan_1 近距回波应在 0° 附近，/scan_2 应在 180° 附近

## 三、录制命令

```bash
mkdir -p ~/slam_ws/rosbags && cd ~/slam_ws/rosbags
ros2 bag record -o wheelchair_raw_test_01 /scan_1 /scan_2 /imu /odom /cmd_vel /tf_static /tf
```

- `/tf`、`/tf_static` **按名字直接录即可，不是隐藏话题**，不需要 `--include-hidden-topics`
- `/tf` 内容为 dsw_chair 广播的 **odom→base_link**（launch 参数 `publish_tf` **默认关闭**；想要 odom TF 进 bag 就 `publish_tf:=true`）。
  开启后母包可直接做"纯 odom"离线回放；但做 **EKF 离线回放时必须用 `--topics` 排除 /tf**（见第六节），否则与 EKF 广播的 TF 打架
- 磁盘紧张时加 `-b 2048`（每 2GB 自动切分文件）
- 停止录制：Ctrl+C

## 四、话题说明与验收（ros2 bag info）

| 话题 | 类型 | 作用 | 验收标准 |
|---|---|---|---|
| `/scan_1` | sensor_msgs/LaserScan | 1 号雷达（laser_1） | ~600 条/分钟 @10Hz |
| `/scan_2` | sensor_msgs/LaserScan | 2 号雷达（laser_2） | ~600 条/分钟，与 scan_1 相近 |
| `/imu` | sensor_msgs/Imu | IMU（imu_link） | 数千条/分钟 |
| `/odom` | nav_msgs/Odometry | 轮式里程计（dsw_chair） | ~1200 条/分钟 @20Hz |
| `/cmd_vel` | geometry_msgs/Twist | 遥控指令（对照/回放用） | 手柄在动就有持续输出 |
| `/tf_static` | tf2_msgs/TFMessage | 3 条静态外参 | **≥1 条即合格**（锁存话题只有 1~2 条，不是"大量"） |
| `/tf` | tf2_msgs/TFMessage | odom→base_link（dsw_chair，publish_tf） | 默认 0 条；`publish_tf:=true` 时 ~1200 条/分钟 @20Hz |

- ⚠️ 若 `/tf_static` 为 0 条（transient_local QoS 与录制订阅不匹配）：改用
  `ros2 bag record --qos-profile-overrides-path ~/slam_ws/rosbag_qos_overrides.yaml -o <名字> ...` 重录
- 体积外推：`du -sh <bag目录>` ÷ 时长 = MB/min，据此规划正式路线所需空间
- 任何话题 0 条或骤降 → 该包作废重录

## 五、命名规范与实验记录

```
wheelchair_raw_test_01      # 1~2 分钟试录（先录这个！）
wheelchair_raw_loop_01      # 闭环综合：走廊+转弯+房间+回起点
wheelchair_raw_corridor_01  # 长走廊往返
wheelchair_raw_door_01      # 窄门/房间进出
```

- 母包只读保存，不覆盖；对比实验一律在副本/离线回放上做
- 每包录完立即 `ros2 bag info` 验收 + 填 [rosbags/EXPERIMENT_LOG.md](rosbags/EXPERIMENT_LOG.md)（日期、地点、路线、速度、人流）

## 六、离线回放建图流程（母包的分叉用法之一）

bag 里只有原始数据、动态 TF 需回放时重算。回放要点：
1. 回放 rosbag（`--clock`，**必须从开头播**）
2. 发布静态 TF（数值取自 [src/jt_chair/launch/double_nav2.launch.py:281-298](src/jt_chair/launch/double_nav2.launch.py#L281-L298)）：laser_1 `(0.29,-0.255,0.2)`、laser_2 `(-0.29,0.255,0.2,3.124)`、imu_link `(0.185,0,0.2)`
3. 重跑 EKF（`src/jt_chair/config/ekf.yaml`，`use_sim_time: true`）→ 恢复 odom→base_link TF
4. 重跑 `dual_laser_merger`（`use_sim_time: true`）→ 恢复 `/scan_merged`
5. 启动 slam_toolbox 离线模式（`src/jt_chair/config/mapper_params_offline.yaml`，`scan_topic: /scan_merged`）
6. 结束后调用 `/slam_toolbox/save_map` 保存地图到 `~/slam_ws/maps/`

手动分步执行：

```bash
# 终端1: 回放 bag（EKF 流程必须用 --topics 排除 /tf —— bag 里录的是轮式 odom 的 TF，
#        会和 EKF 广播的 odom→base_link 打架）
ros2 bag play ~/slam_ws/rosbags/wheelchair_raw_loop_01 --clock --topics /scan_1 /scan_2 /imu /odom /cmd_vel

# 终端2: 静态 TF
ros2 run tf2_ros static_transform_publisher 0.29 -0.255 0.2 0 0 0 base_link laser_1 &
ros2 run tf2_ros static_transform_publisher -0.29 0.255 0.2 3.124 0 0 base_link laser_2 &
ros2 run tf2_ros static_transform_publisher 0.185 0 0.2 0 0 0 base_link imu_link &

# 终端3: EKF
ros2 run robot_localization ekf_node --ros-args \
    --params-file ~/slam_ws/src/jt_chair/config/ekf.yaml \
    -r /imu/data:=/imu -p use_sim_time:=true

# 终端4: 双雷达融合
ros2 run dual_laser_merger dual_laser_merger_node --ros-args \
    -p laser_1_topic:=/scan_1 -p laser_2_topic:=/scan_2 \
    -p merged_scan_topic:=/scan_merged -p target_frame:=base_link \
    -p publish_rate:=20 -p angle_increment:=0.00698 -p scan_time:=0.1 \
    -p range_min:=0.20 -p range_max:=25.0 -p use_inf:=False \
    -p use_sim_time:=true

# 终端5: 离线建图
ros2 run slam_toolbox sync_slam_toolbox_node --ros-args \
    --params-file ~/slam_ws/src/jt_chair/config/mapper_params_offline.yaml \
    -p use_sim_time:=true

# 回放结束后保存地图
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: '~/slam_ws/maps/offline_map'}"
```

只做 EKF 融合对照（不建图）：跑 终端1/2/3，对比 `/odometry/filtered` 与 `/odom` 即可。

**纯 odom 对照（不跑 EKF）**：前提是录制时开了 `publish_tf:=true`（bag 的 /tf 含轮式 odom→base_link），整包播放即可，无需 EKF：

```bash
# 终端1: 整包播放（含 /tf，不加 --topics）
ros2 bag play ~/slam_ws/rosbags/wheelchair_raw_loop_01 --clock
# 终端2/3/4: 静态 TF、双雷达融合、离线建图，与上面 EKF 流程的命令完全相同
```

对比实验做法：同一段 bag 分别走「EKF 流程」和「纯 odom 流程」各建一次图，对比轨迹与地图质量。

## 七、注意事项

- ⚠️ [ekf.yaml:42](src/jt_chair/config/ekf.yaml#L42) 中 `imu0_relative: true`（以第一帧 Yaw 为基准）→ **回放必须从 bag 开头播**，中途起播航向基准会偏移
- 回放所有节点需设 `use_sim_time: true`，并与 `ros2 bag play --clock` 配合
- 录制过程中可用 `ros2 bag info <bag目录>` 确认各话题消息数在增长
- 雷达串口是 /dev/ttyACM0、/dev/ttyACM1，**重启后可能互换**：每次上电录包前用第二节的方法核对 frame_id
