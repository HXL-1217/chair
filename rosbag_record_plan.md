# 离线建图 rosbag 录制方案（纯原始数据）

> 目标：在 slam_toolbox 在线建图过程中，只录制必要的**原始数据**话题，之后可用 `mapper_params_offline.yaml` 离线重建地图。

## 一、录制命令

```bash
ros2 bag record -o ~/slam_ws/rosbags/mapping_20260816 /odom /imu /scan_1 /scan_2
```

- 这 4 个话题都不是隐藏话题，**不需要** `--include-hidden-topics`（只有 `/tf`、`/tf_static` 是隐藏话题）
- 磁盘紧张时加 `-b 2048`（每 2GB 自动切分文件）
- 停止录制：Ctrl+C

## 二、话题说明

| 话题 | 类型 | 作用 |
|---|---|---|
| `/odom` | nav_msgs/Odometry | 轮式里程计原始数据（dsw_chair 发布） |
| `/imu` | sensor_msgs/Imu | IMU 原始数据（fdilink_ahrs 发布） |
| `/scan_1` | sensor_msgs/LaserScan | 前雷达激光帧原始数据 |
| `/scan_2` | sensor_msgs/LaserScan | 后雷达激光帧原始数据 |

**不录**：`/merged_cloud`（3D 点云，数据量巨大）、`/map`、`/tf`、`/cmd_vel`、`/diagnostics`、`/rosout` 等。

## 三、离线回放建图流程

bag 里只有原始数据、**没有 TF**，回放时必须重跑处理节点。**一键脚本已就绪**：

```bash
./start_offline_mapping.sh [bag目录] [回放倍速] [地图保存路径前缀]
# 例: ./start_offline_mapping.sh ~/slam_ws/rosbags/mapping_20260816
```

脚本自动完成（详见 [start_offline_mapping.sh](start_offline_mapping.sh)）：
1. 回放 rosbag（`--clock`，必须从开头播）
2. 发布静态 TF（数值取自 [src/jt_chair/launch/double_nav2.launch.py:281-298](src/jt_chair/launch/double_nav2.launch.py#L281-L298)）：laser_1 `(0.29,-0.255,0.2)`、laser_2 `(-0.29,0.255,0.2,3.124)`、imu_link `(0.185,0,0.2)`
3. 重跑 EKF（`src/jt_chair/config/ekf.yaml`，`use_sim_time: true`）→ 恢复 odom→base_link TF
4. 重跑 `dual_laser_merger`（`use_sim_time: true`）→ 恢复 `/scan_merged`
5. 启动 slam_toolbox 离线模式（`src/jt_chair/config/mapper_params_offline.yaml`，`scan_topic: /scan_merged`）
6. 回放结束后自动调用 `/slam_toolbox/save_map` 保存地图到 `~/slam_ws/maps/`

手动分步执行（不想用脚本时）：

```bash
# 终端1: 回放 bag
ros2 bag play --clock ~/slam_ws/rosbags/mapping_20260816

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

## 四、注意事项

- ⚠️ [ekf.yaml:42](src/jt_chair/config/ekf.yaml#L42) 中 `imu0_relative: true`（以第一帧 Yaw 为基准）→ **回放必须从 bag 开头播**，中途起播航向基准会偏移
- 回放所有节点需设 `use_sim_time: true`，并与 `ros2 bag play --clock` 配合
- 录制过程中可用 `ros2 bag info ~/slam_ws/rosbags/mapping_20260816` 确认 4 个话题消息数在增长
- 若想省去回放时手动发静态 TF 的步骤，可加录 `/tf_static`（只录一次、仅几 KB）：命令末尾追加 `/tf_static --include-hidden-topics`
