# JT Chair 智能轮椅导航系统 — 使用手册

> **平台：** Orange Pi 5 (RK3588) | **系统：** Ubuntu 22.04 + ROS 2 Humble  
> **版本：** 巡迹过门 v1.0 / 过门循迹 v2.0

---

## 目录

1. [系统概述](#1-系统概述)
2. [硬件清单与连接](#2-硬件清单与连接)
3. [目录结构](#3-目录结构)
4. [环境准备](#4-环境准备)
5. [快速开始](#5-快速开始)
6. [建图流程](#6-建图流程)
7. [重定位流程](#7-重定位流程)
8. [导航流程](#8-导航流程)
9. [语音控制](#9-语音控制)
10. [路径点标定](#10-路径点标定)
11. [手柄遥控](#11-手柄遥控)
12. [配置文件详解](#12-配置文件详解)
13. [ROS2 接口参考](#13-ros2-接口参考)
14. [常见问题与故障排查](#14-常见问题与故障排查)
15. [附录](#15-附录)

---

## 1. 系统概述

本项目是 **JT Chair 智能轮椅** 的 ROS 2 自主导航系统。轮椅配备双激光雷达、IMU 惯性导航单元和语音识别模块，能够实现 SLAM 建图、全局重定位、自主导航、语音指令控制、自动过门和马桶对接泊车等功能。

### 硬件平台

- **主控：** Orange Pi 5 (Rockchip RK3588, 4×A76 + 4×A55)
- **激光雷达：** 2× 镭神 LSN10P（360° 机械式，前右 + 后左 对角安装）
- **IMU：** FDIlink AHRS (Wheeltec 定制版，带磁力计航向角)
- **电机驱动：** 2× ZLAC8015 伺服驱动器（Modbus RTU 总线）
- **运动模型：** 双舵轮全向底盘 (Dual Steered Wheelchair)

### 软件栈

| 层 | 组件 | 说明 |
|----|------|------|
| 定位 | slam_toolbox (localization 模式) | 基于预建位姿图地图的扫描匹配定位，利用回环检测实现全局重定位 |
| 融合 | robot_localization (EKF) | 融合轮式里程计线速度 + IMU 航向角/角速度，输出平滑 odom→base_link 变换 |
| 建图 | slam_toolbox (mapping 模式) | 同步建图，支持交互式标记与回环检测 |
| 导航 | Nav2 (MPPI + SmacPlanner2D) | 模型预测路径积分控制器 + 混合 A* 全局规划器 |
| 语音 | voice_nav_bridge (自研) | 串口接收语音模块指令，驱动 Nav2 导航与自定义过门/泊车逻辑 |
| 感知 | dual_laser_merger | 双雷达点云实时融合 |

### 核心功能

- **SLAM 建图** — 通过手柄遥控轮椅遍历环境，自动构建位姿图地图
- **全局重定位** — 加载已有地图后，在任意位置自动锁定（利用回环检测 + 2D Pose Estimate）
- **自主导航** — 在地图中设定目标点，轮椅自主规划路径并行驶到目的地
- **语音指令控制** — 通过语音模块发送串口命令，无需屏幕即可导航到各房间
- **自动过门** — 识别门口位置，使用 PID 巡线控制精确穿过狭窄门框
- **马桶对接泊车** — 倒车模式自动倒退到马桶指定位置并锁定
- **手柄遥控** — USB/蓝牙手柄操控轮椅移动

---

## 2. 硬件清单与连接

### 2.1 设备串口映射表

| 设备 | 串口路径 | 通信协议 | 波特率 | ROS 话题/用途 |
|------|----------|----------|--------|---------------|
| 激光雷达 1（前右） | `/dev/ttyACM1` | UART (LSN10P 协议) | 驱动默认 (230400) | `/scan_1` |
| 激光雷达 2（后左） | `/dev/ttyACM0` | UART (LSN10P 协议) | 驱动默认 (230400) | `/scan_2` |
| IMU (AHRS) | `/dev/wheeltec_FDI_IMU_GNSS` | 原始串口 (FDI 协议) | 921600 | `/imu` |
| 电机驱动器 (ZLAC8015) | `/dev/ttyUSB0` | Modbus RTU | 115200 | `/odom`, `/cmd_vel` |
| 语音模块 | `/dev/ttyS4` | 原始串口 (单字节命令) | 115200 | 语音导航指令 |
| USB 手柄 | 自动检测 | USB HID | — | `/cmd_vel` (遥控) |

### 2.2 激光雷达安装位置

```
        前方 (+x)
          ↑
    ┌─────────────┐
    │   激光雷达1  │  ← 前右侧 (0.29, -0.255, 0.2)m  yaw=0°
    │   (前右)    │
    │             │
    │    base_link│
    │             │
    │   激光雷达2  │  ← 后左侧 (-0.29, 0.255, 0.2)m  yaw≈179°
    │   (后左)    │
    └─────────────┘
```

每台激光雷达裁剪掉面向机器人本体方向的扫描角度（激光雷达1剪掉 88°~200°，激光雷达2剪掉 90°~200°），避免扫描到轮椅自身结构。

### 2.3 TF 坐标系树

```
map
  └── odom                    ← slam_toolbox 发布 (map→odom)
        └── base_link         ← EKF 发布 (odom→base_link)
              ├── laser_1     ← static_transform_publisher
              ├── laser_2     ← static_transform_publisher
              └── imu_link    ← static_transform_publisher
```

### 2.4 数据流架构

```
[语音模块] ---(/dev/ttyS4)---> voice_nav_bridge ---(NavigateToPose Action)---> Nav2
                                                                                │
[LSN10P 雷达1] --/scan_1--> dual_laser_merger --/scan_merged--> slam_toolbox   │
[LSN10P 雷达2] --/scan_2---/                                  (localization)    │
                                                                  │             │
                                                                  v             v
[IMU] --/imu--> EKF (robot_localization) --/odom (filtered)--> TF 树       /cmd_vel
                  ↑                                         map→odom→base   │
[电机] --/odom(raw)----------------------------------------/                │
                  ↑                                                        │
                  └────────────── /cmd_vel ────────────────────────────────┘
```

---

## 3. 目录结构

### 3.1 工作空间根目录

```
~/slam_ws/
├── README.md                    ← 本手册
├── start_mapping.sh             ← 建图启动脚本
├── start_localization.sh        ← 重定位启动脚本
├── start_nav2_pro.sh            ← 导航启动脚本
├── get_points.py                ← RViz 点击坐标捕获工具
├── qr_scanner.py                ← 二维码扫描工具 (测试使用)
├── handle/
│   └── cmd_vel_publisher_node.py  ← 手柄遥控节点
├── src/                         ← ROS 2 功能包源代码
│   ├── jt_chair/                ← 核心功能包 (语音导航/launch/配置/地图)
│   ├── Wheelchair/              ← 轮椅底盘驱动 (包名: dsw_chair)
│   ├── fdilink_ahrs_ROS2/       ← IMU 驱动 (包名: fdilink_ahrs)
│   ├── LSLIDAR_X_ROS2-20240228/ ← 镭神激光雷达驱动 (包名: lslidar_driver)
│   ├── dual_laser_merger/       ← 双雷达融合节点
│   ├── slam_toolbox/            ← SLAM Toolbox (建图/定位)
│   └── serial_ros2/             ← 串口通信库
├── build/                       ← 编译中间文件
├── install/                     ← 编译输出 (需 source 此目录)
└── log/                         ← 编译日志
```

### 3.2 核心包说明

| 包名 | 类型 | 功能 |
|------|------|------|
| **jt_chair** | ament_python | 核心编排包，包含所有 launch 文件、YAML 配置、预建地图、语音导航桥接节点 |
| **dsw_chair** | ament_cmake | 轮椅底盘驱动：轮式里程计 (odom_publisher)、DualSteeredWheelchair 运动学、ZLAC8015 Modbus 控制、驾驶模式管理 |
| **fdilink_ahrs** | ament_cmake | 九轴 IMU 驱动，解析 FDI 协议帧，发布 `/imu`、欧拉角、磁力计、GPS 话题 |
| **lslidar_driver** | ament_cmake | 镭神 LSN10/LSM10 系列激光雷达驱动 (LifecycleNode)，支持单/双雷达配置 |
| **dual_laser_merger** | ament_cmake | 将两路 LaserScan 变换到同一坐标系后合并为单路 `/scan_merged` |
| **slam_toolbox** | ament_cmake | 2D SLAM 库：同步/异步建图、定位模式、终身建图、回环检测 |
| **serial_ros2** | ament_cmake | 跨平台串口通信 C++ 库，由 wjwwood/serial 移植 |

### 3.3 jt_chair 包内部结构

```
src/jt_chair/
├── jt_chair/
│   ├── __init__.py
│   └── voice_nav_bridge.py     ← 语音导航桥接节点 (891 行)
├── launch/                     
│   ├── double_mapping.launch.py       ← 建图启动文件
│   ├── double_localization.launch.py  ← 定位启动文件
│   ├── double_nav2.launch.py          ← 导航启动文件
├── config/                     
│   ├── double_localization.yaml             ← 定位配置文件
│   ├── mapper_params_online_sync.yaml       ← 建图配置文件
│   ├── ekf.yaml                             ← 融合算法配置文件
│   ├── nav2_params_dw.yaml                  ← 导航配置文件
│   └── waypoints_config.json                ← 定位坐标文件
├── map/                        ← 预建地图文件 (.data + .posegraph)
├── rviz/
│   └── slam_toolbox.rviz       ← RViz 显示配置
├── setup.py
└── package.xml
```

---

## 4. 环境准备

### 4.1 系统依赖

- **操作系统：** Ubuntu 22.04 (ARM64)
- **ROS 2 发行版：** Humble Hawksbill (`/opt/ros/humble/`)
- **Python：** 3.10+
- **必备系统包：**
  ```bash
  sudo apt install python3-pip python3-serial python3-pygame
  pip3 install pyserial pyzbar opencv-python  # 语音桥接依赖
  ```

### 4.2 编译

```bash
cd ~/slam_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

> **说明：** `--symlink-install` 使 Python 文件的修改无需重新编译即可生效（修改 `.py` 后直接重新启动 launch 即可）。



## 5. 快速开始

### 5.1 三种运行模式

| 模式 | 启动脚本 | Launch 文件 | 核心节点 |
|------|---------|------------|---------|
| **建图** | `./start_mapping.sh` | `double_mapping.launch.py` | sync_slam_toolbox_node (mapping) |
| **重定位** | `./start_localization.sh` | `double_localization.launch.py` | localization_slam_toolbox_node |
| **导航** | `./start_nav2_pro.sh` | `double_nav2.launch.py` | slam_toolbox + Nav2 全栈 |


### 5.3 传感器组合

| Launch 文件 | 激光雷达 | IMU | EKF | 雷达融合 | Nav2 |
|------------|---------|-----|-----|---------|------|
| `double_mapping.launch.py` | 双雷达 | ✓ | ✓ | ✓ | ✗ |
| `double_localization.launch.py` | 双雷达 | ✓ | ✓ | ✓ | ✗ |
| `double_nav2.launch.py` | 双雷达 | ✓ | ✓ | ✓ | ✓ |

> **当前主要使用双雷达系列 launch 文件**（`double_*`），单雷达系列为历史兼容保留。

---

## 6. 建图流程

### 6.1 操作步骤

1. **将轮椅推到环境起点**（建议空旷区域，方便回环检测）

2. **启动建图：**
   ```bash
   cd ~/slam_ws
   ./start_mapping.sh
   ```

3. **（另一终端）打开 RViz 可视化（可选）：**
   ```bash
  
   rviz2 
   ```

4. **使用手柄遥控轮椅**，以低速 (≤0.3 m/s) 遍历整个环境。
   - 建议走闭合路径（回到起点）以触发回环检测
   - 经过门框、狭窄走廊时放慢速度，保证扫描点云充足
   - 运行手柄文件：cmd_vel_publisher_node.py

5. **保存地图：**
   - slam_toolbox地图保存指令：
ros2 service call /slam_toolbox/serialize_map  slam_toolbox/srv/SerializePoseGraph "{filename: '/home/orangepi/slam_ws/src/jt_chair/map/your_map'}"
   只需要修改‘your_map’名称，文件路径已经设置好



6. **修改 launch 文件中的地图路径**（见下方 [6.2 地图路径配置](#62-地图路径配置)）

7. **按 Ctrl+C 退出建图**

### 6.2 地图路径配置

地图保存后，需要在定位/导航 launch 文件中指定新地图路径。

**修改 `double_localization.launch.py`（第 193 行）：**
```python
map_file = "/home/orangepi/slam_ws/src/jt_chair/map/ysg1"  # 改为你的地图名，只改‘ysg1’名称即可，文件不需要加后缀
```

**修改 `double_nav2.launch.py`（对应位置）：**
```python
map_file = "/home/orangepi/slam_ws/src/jt_chair/map/ysg1"  # 改为你的地图名，只改‘ysg1’名称即可，文件不需要加后缀
```

> ⚠️ **注意：** 地图文件是一对：`<name>.data` + `<name>.posegraph`，两个文件必须同时存在。`map_file` 参数无需写 `.data` 后缀。

### 6.3 已有地图清单

| 地图名 | .data 大小 | .posegraph 大小 | 最后修改 | 说明 |
|--------|----------|----------------|---------|------|
| **ysg1** | 1 MB | 11 MB | 2026-07-03 | 当前活跃地图（主场1） |
| ysg | 2.5 MB | 13 MB | 2026-07-01 | 备用主场地图 |
| tz_1 | 3.3 MB | 14 MB | 2026-05-12 | 套间1 |
| tz_2 | 2.3 MB | 13 MB | 2026-05-17 | 套间2 |
| tz_3 | 2.3 MB | 13 MB | 2026-05-21 | 套间3 |
| zh_0 ~ zh_2 | 0.5~0.8 MB | 11 MB | 2026-05-27~06-03 | 展会地图系列 |
| test_1 | 1.4 MB | 12 MB | 2026-06-24 | 测试地图 |
| new1 | 2.1 MB | 13 MB | 2026-02-02 | 单雷达模式地图 |
| 105_2 | 3.2 MB | 14 MB | 2026-01-18 | 单雷达模式地图 |

### 6.4 建图参数要点

配置文件：`src/jt_chair/config/mapper_params_online_sync.yaml`

| 参数 | 值 | 说明 |
|------|-----|------|
| `mode` | `mapping` | 同步建图模式 |
| `scan_topic` | `/scan_merged` | 使用双雷达融合后的扫描 |
| `resolution` | 0.05 m | 地图分辨率 (5cm) |
| `scan_buffer_size` | 20 | 扫描缓存（针对 RK3588 大内存优化） |
| `do_loop_closing` | `true` | 开启回环检测 |
| `loop_match_minimum_chain_size` | 10 | 最少10个连续节点才触发回环（防止走廊误匹配） |
| `loop_search_maximum_distance` | 5.0 m | 回环搜索最大距离 |
| `distance_variance_penalty` | 2.0 | 距离方差惩罚（建图时更严格，防止漂移） |

---

## 7. 重定位流程

### 7.1 重定位原理

本系统使用 **slam_toolbox 的 localization 模式**。它的定位机制是：

1. 加载预先建好的位姿图地图（`.data` + `.posegraph`）
2. 持续将当前激光扫描与地图中的关键帧进行扫描匹配
3. 通过 `map→odom` 的 TF 变换发布机器人在世界坐标系中的位置
4. 利用 **回环检测** (`do_loop_closing: true`) 实现全局重定位——
   即使机器人被"绑架"（搬到一个完全陌生的位置），
   只要在 `loop_search_space_dimension: 10.0m` 范围内有匹配的关键帧，就能自动恢复到正确位姿

### 7.2 操作步骤

1. **启动重定位：**
   ```bash
   cd ~/slam_ws
   ./start_localization.sh
   ```

2. **（另一终端）打开 RViz：**
   ```bash
   source ~/slam_ws/install/setup.bash
   rviz2 
   ```

3. **设置初始位姿（首次定位）：**
   - 在 RViz 中点击顶部工具栏的 **"2D Pose Estimate"**
   - 在地图上机器人实际所在位置点击并拖拽设定方向
   - 机器人随后缓慢移动即可自动锁定到正确位姿

   > **说明：** 设置 `/initialpose` 后，slam_toolbox 会在该位置附近搜索匹配的关键帧。锁定通常只需前进/后退 1-2 米即可完成。

4. **验证定位是否成功：**
   - RViz 中激光扫描点云与地图墙壁对齐，说明定位成功
   - 若明显错位，重新设置 2D Pose Estimate

5. **按 Ctrl+C 退出**

### 7.3 关键参数

配置文件：`src/jt_chair/config/double_localization.yaml`

| 参数 | 值 | 说明 |
|------|-----|------|
| `mode` | `localization` | 纯定位，不更新地图 |
| `do_loop_closing` | `true` | **全局重定位开关**（最关键） |
| `loop_search_space_dimension` | 10.0 m | 被绑架后搜索范围，越大恢复能力越强 |
| `loop_search_maximum_distance` | 5.0 m | 回环匹配最大距离 |
| `loop_match_minimum_chain_size` | 10 | 最少匹配节点数（防走廊误匹配） |
| `correlation_search_space_dimension` | 0.3 m | 局部搜索范围（正常跟踪时） |
| `distance_variance_penalty` | 1.5 | 距离方差惩罚（配合轮式里程计，防长走廊漂移） |
| `angle_variance_penalty` | 1.5 | 角度方差惩罚 |
| `minimum_travel_distance` | 0.15 m | 最小移动距离阈值（防抖动） |
| `minimum_travel_heading` | 0.05 rad | 最小旋转阈值（防微动误更新） |
| `transform_publish_period` | 0.01 s (100Hz) | TF 发布频率 |

---

## 8. 导航流程

### 8.1 启动导航

```bash
cd ~/slam_ws
./start_nav2_pro.sh
```

此脚本后台启动 `double_nav2.launch.py`，等待 10 秒初始化后自动将 `controller_server` 和 `planner_server` 绑定到大核。

### 8.2 设置导航目标

设定导航目标：

```bash
# 另一终端
source ~/slam_ws/install/setup.bash
rviz2 
```
- 点击顶部工具栏 **"2D Goal Pose"**
- 在地图上目标位置点击并拖拽设定方向
- 轮椅开始自主规划并导航



### 8.3 导航参数

#### MPPI 控制器（核心控制器）

```
控制器频率: 20 Hz
运动模型:   DiffDrive (差速模型)
轨迹优化:   batch_size=1000, time_steps=40, model_dt=0.1s
温度参数:   0.6（轨迹平滑度）
速度限制:   vx_max=0.5 m/s, vx_min=-0.1 m/s, wz_max=1.2 rad/s
目标精度:   xy=0.1 m, yaw=0.1 rad
```

**8 个活跃的 Critic 评分项：**

| Critic | 权重 | 作用 |
|--------|------|------|
| ObstaclesCritic | 35.0 | 障碍物避障（碰撞代价=1e6, 安全边距=0.10m） |
| PathAlignCritic | 15.0 | 路径对齐 |
| PathFollowCritic | 12.0 | 路径跟踪 |
| PreferForwardCritic | 10.0 | 倾向前进运动（阈值 0.5m） |
| ConstraintCritic | 5.0 | 运动学约束 |
| GoalCritic | 5.0 | 目标接近 |
| GoalAngleCritic | 3.0 | 目标朝向 |
| PathAngleCritic | 2.0 | 路径朝向跟踪 |

#### SmacPlanner2D 全局规划器

```
类型:       Hybrid-A* (nav2_smac_planner/SmacPlanner2D)
最大迭代:   100,000
最大规划时间: 2.0 秒
容许误差:   0.5 m
平滑器:     启用 (SimpleSmoother, 1000 次迭代)
```

#### 代价地图

| 属性 | 局部代价地图 | 全局代价地图 |
|------|------------|------------|
| 坐标系 | `odom` | `map` |
| 类型 | 滚动窗口 | 固定 |
| 尺寸 | 4m × 4m | 整张地图 |
| 分辨率 | 0.05 m | 0.05 m |
| 更新频率 | 10 Hz | — |
| 发布频率 | 5 Hz | — |
| 膨胀半径 | 0.5 m | 0.5 m |
| 膨胀衰减 | 4.0 | 4.0 |
| 占用层来源 | `/scan_merged` | `/scan_merged` |

**机器人 Footprint（外轮廓）：**
```
[[0.41, 0.29], [0.41, -0.29], [-0.32, -0.29], [-0.32, 0.29]]
```
约 0.82m (长) × 0.58m (宽)，含 0.02m 膨胀余量。

#### 速度平滑器

```
频率:    20 Hz
模式:    OPEN_LOOP（开环平滑）
最大速度: [0.5, 0.0, 1.20]     ← vx, vy, vyaw
最小速度: [-0.15, 0.0, -1.20]
最大加速度: [1.0, 0.0, 2.0]
最大减速度: [-1.0, 0.0, -2.0]
```

### 8.4 恢复行为

导航遇到阻塞时，Nav2 行为树会触发以下恢复行为（按优先级）：

| 行为 | 说明 | 参数 |
|------|------|------|
| **Spin** | 原地旋转寻找可行路径 | 最大 1.0 rad/s, 最小 0.4 rad/s |
| **Backup** | 后退一段距离再重试 | — |
| **Drive On Heading** | 沿当前朝向直行冲出死角 | — |
| **Wait** | 原地等待（如等障碍物移开） | — |

---

## 9. 语音控制

### 9.1 硬件连接

语音识别模块通过 UART 连接 Orange Pi 的 `/dev/ttyS4`（波特率 115200）。模块发送**单字节十六进制命令**到串口，`voice_nav_bridge` 节点持续读取并解析。

### 9.2 完整命令码表

#### 导航命令

| 十六进制 | 十进制 | 功能 | 目标 | 说明 |
|----------|--------|------|------|------|
| `0x30` | 48 | **驶出解锁** | — | 退出 LOCKED 状态，前进 0.5m 后恢复导航 |
| `0x31` | 49 | **去厕所** | 厕所 (key 49) | 直接导航，无过门流程 |
| `0x32` | 50 | **去客厅** | 客厅 (key 50) | 若在其他房间则先过门 |
| `0x33` | 51 | **去厨房** | 厨房 (key 51) | 若在其他房间则先过门 |
| `0x34` | 52 | **去主卧** | 主卧 (key 52) | 若在其他房间则先过门 |
| `0x35` | 53 | **去厕所/倒车** | 厕所 (key 53) | 到达后自动倒车泊入马桶位置 |
| `0x36` | 54 | **去客卧** | 客卧 (key 54) | 若在其他房间则先过门 |

#### 标定命令

| 十六进制 | 十进制 | 功能 | 说明 |
|----------|--------|------|------|
| `0x40` | 64 | **标定马桶目标** | 记录当前位置为马桶正前方位姿 |
| `0x41` | 65 | **标定厕所区域** | 记录厕所位置、门口、区域多边形（需先通过 QR 码验证） |
| `0x42` | 66 | **标定客厅** | 记录当前位置为客厅 |
| `0x43` | 67 | **标定厨房** | 记录当前位置为厨房 |
| `0x44` | 68 | **标定主卧** | 记录当前位置为主卧 |
| `0x45` | 69 | **标定客卧** | 记录当前位置为客卧 |

#### 模式/速度命令

| 十六进制 | 功能 | 说明 |
|----------|------|------|
| `0x11` | 驾驶模式 0 | — |
| `0x12` | 驾驶模式 1 | — |
| `0x13` | 驾驶模式 2 | — |
| `0x21` | 速度等级 0（最低速） | — |
| `0x22` | 速度等级 1（中速） | — |
| `0x23` | 速度等级 2（最高速） | — |

#### 反馈命令

| 十六进制 | 功能 | 说明 |
|----------|------|------|
| `0x01` | 反馈同步脉冲 | 倒车过程中语音模块发送的心跳同步信号 |

### 9.3 状态机

voice_nav_bridge 内部维护一个有限状态机，管理导航全流程：

```
                    ┌─────────────┐
        0x30        │   LOCKED    │  ← 泊车后锁定
      ┌───────────→ │  (已泊车)   │
      │             └──────┬──────┘
      │                    │ 0x35 (倒车泊入)
      │                    v
      │             ┌─────────────┐
      │             │   DOCKING   │  ← PID 倒车控制
      │             └──────┬──────┘
      │                    │
┌─────┴──────┐      ┌─────┴──────┐
│    IDLE    │ ←──→ │ NAVIGATING │  ← 直接导航到目标
│  (空闲)    │      └─────┬──────┘
└─────┬──────┘            │
      │                   v
      │            ┌──────────────────┐
      │            │ NAVIGATING_TO_   │  ← 首次导航到源房间门口
      │            │ SOURCE_DOOR      │
      │            └──────┬───────────┘
      │                   │ 到达门口内侧
      │                   v
      │            ┌──────────────────┐
      │            │ DOOR_EXITING     │  ← PID 巡线出房门
      │            └──────┬───────────┘
      │                   │ 到达门外侧
      │                   v
      │            ┌──────────────────┐
      │            │ NAVIGATING_TO_   │  ← Nav2 导航到目标房间门口
      │            │ DOOR             │
      │            └──────┬───────────┘
      │                   │ 到达目标门口外侧
      │                   v
      │            ┌──────────────────┐
      │            │ DOOR_PASSING     │  ← PID 巡线进房门
      │            └──────┬───────────┘
      │                   │ 到达门内侧
      │                   v
      │            ┌──────────────────┐
      │            │ NAVIGATING_TO_   │  ← Nav2 导航到房间内目标
      │            │ ROOM             │
      │            └──────┬───────────┘
      │                   │
      └───────────────────┘
```

### 9.4 过门流程详解

当机器人需要从房间 A 导航到房间 B 时，系统自动执行以下跨房间流程：

1. **源房间门口导航：** Nav2 导航到房间 A 门口内侧点 (`door.inside`)
2. **PID 巡线出房门：** 
   - 从 `door.inside` 沿直线巡线到 `door.outside`
   - 计算相对于门口线段的横向误差，用 PD 控制器修正航向
   - 控制律：`angular_z = Kp_cross × cross_track + Kp_heading × heading_error + Kd_yaw × derivative`
3. **公区导航：** Nav2 从门外侧导航到目标房间门口外侧点
4. **PID 巡线进房门：**
   - 从 `door.outside` 沿直线巡线到 `door.inside`
   - 同出房门控制逻辑
5. **目标房间导航：** Nav2 从门内侧导航到最终房间目标点

### 9.5 房间检测

系统使用**射线投射算法** (Ray Casting) 判断机器人当前所在的房间：
- 每个房间定义了一个多边形区域（`waypoints_config.json` 中的 `polygon`）
- 机器人获取当前定位位姿后，检测该点落在哪个房间的多边形内
- 如果在目标房间内则直接导航，否则触发过门流程

### 9.6 马桶 QR 码验证

标定厕所区域 (0x41) 前，系统要求 QR 码验证：
1. 机器人通过 USB 摄像头打开 `/dev/video0`
2. 6 秒内尝试 5 次扫描二维码
3. 必须扫描到内容为 `"1"` 的二维码才能通过验证
4. 验证失败则拒绝标定，并向语音模块发送错误反馈


---

## 10. 路径点标定

# === 查看 当前机器人坐标 指令 ===
ros2 run tf2_ros tf2_echo map base_link
示例：
At time 1778556906.886650899
- Translation: [-1.687, 4.518, 0.000]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, -0.007, 1.000]
- Rotation: in RPY (radian) [0.000, 0.000, -0.013]
- Rotation: in RPY (degree) [0.000, 0.000, -0.760]
- Matrix:
  1.000  0.013  0.000 -1.687
 -0.013  1.000 -0.000  4.518
 -0.000  0.000  1.000  0.000
  0.000  0.000  0.000  1.000
  #机器人坐标为：[-1.687, 4.518,-0.013]


### 10.1 标定概述

路径点标定是将真实物理位置与地图坐标系绑定的过程。建图完成后，需要标定每个房间的目标点、门口位置和区域多边形，语音导航才能正常工作。

**标定时机：** 
- 建图完成后首次使用语音导航前
- 家具布局变更后
- 导航到某房间位置不准确时重新标定

**标定精度建议：** ±0.1m（过大的误差会导致通过门口时失败）

### 10.2 标定步骤（逐房间）

以下以标定客厅（命令 0x42）为例：

1. **启动重定位模式**（确保定位准确）：
   ```bash
   cd ~/slam_ws && ./start_localization.sh
   ```

2. **用手柄遥控轮椅到客厅目标位置**（如沙发前方开阔处）：
   ```bash
   # 另一终端
   source ~/slam_ws/install/setup.bash
   python3 ~/slam_ws/handle/cmd_vel_publisher_node.py
   ```

3. **发送标定命令：**
   - 通过语音模块发送 `0x42`
   - 或通过串口直接发送：`echo -ne '\x42' > /dev/ttyS4`

4. **系统自动将当前位置记录为客厅坐标**，保存到 `waypoints_config.json`

5. **重复以上步骤标定其他房间**

### 10.3 门口和区域标注

门口和区域的标注需要在 `waypoints_config.json` 中手动配置（或通过代码修改）。每个有门的房间需要定义：

```json
{
  "50": {                                          // 房间 key（对应 0x32 客厅）
    "pose": [-0.926, -3.481, 1.618],               // 房间内目标点 [x, y, yaw]
    "door": {
      "outside": [0.0, 2.0],                       // 门口外侧坐标（公区侧）
      "inside": [-0.5, 2.2]                        // 门口内侧坐标（房间侧）
    },
    "polygon": [                                    // 房间区域多边形顶点
      [-1.959, -0.898],
      [0.43, -0.87],
      [0.25, -4.208],
      [-2.139, -4.208]
    ]
  }
}
```

**门口线段说明：**
- `outside` → 门外侧点（走廊/公区侧），是 PID 巡线的终点（出房时）
- `inside` → 门内侧点（房间侧），是 PID 巡线的终点（进房时）
- 线段方向决定了机器人的巡线方向

**区域多边形说明：**
- 用于 room detection，判断机器人当前是否在房间内
- 顶点按顺时针或逆时针排列（任意顺序均可，射线投射算法兼容）
- 建议比房间实际边界稍大 0.5m，确保定位误差下仍能正确检测

### 10.4 马桶工作点标定

马桶标定需要标注两个位置：

| 标定命令 | 标定内容 | 说明 |
|----------|---------|------|
| `0x41` | 厕所区域 | 记录厕所门口 + 房间多边形 + 房间目标点（需 QR 验证） |
| `0x40` | 马桶目标 | 记录马桶正前方位姿，是倒车泊入的起点 |

**马桶倒车流程：**
1. 发送 `0x35`（厕所倒车命令）
2. 机器人从厕所房间目标点导航到马桶目标前方的指定点
3. 触发倒车模式，PID 控制倒退到 `toilet_dock_target` 位置
4. 泊车完成后进入 `LOCKED` 状态，禁止导航

### 10.5 waypoints_config.json 格式

```json
{
  "toilet_dock_target": [-1.715, 0.221, -0.05],
  "location_map": {
    "49": {
      "pose": [1.166, 0.055, 3.117],
      "door": {
        "outside": [-0.352, -1.962],
        "inside": [-0.249, 0.210]
      },
      "polygon": [
        [-1.902, 1.121],
        [2.724, 0.941],
        [2.62, -0.823],
        [-1.949, -0.633]
      ]
    },
    "50": {
      "pose": [-0.926, -3.481, 1.618],
      "polygon": [
        [-1.959, -0.898],
        [0.43, -0.87],
        [0.25, -4.208],
        [-2.139, -4.208]
      ]
    }
    // ... 其他房间
  }
}
```
"pose": [1.166, 0.055, 3.117]，        该坐标可以通过指令或者语音进行记录， 
"door": {                              door坐标需要遥控到位置后用指令记录
        "outside": [-0.352, -1.962],
        "inside": [-0.249, 0.210]
      },
      "polygon": [                    
        [-1.902, 1.121],
        [2.724, 0.941],
        [2.62, -0.823],
        [-1.949, -0.633]         
      ]
 polygon轮廓坐标运行get_points.py代码后，点击rviz界面的publish point点击地图，终端会打印坐标。

| 字段 | 类型 | 说明 |
|------|------|------|
| `toilet_dock_target` | `[x, y, yaw]` | 马桶泊车最终位姿 |
| `location_map.<key>.pose` | `[x, y, yaw]` | 房间导航目标点 |
| `location_map.<key>.door` | `{outside: [x,y], inside: [x,y]}` | 门口坐标（可选，无门的房间可省略） |
| `location_map.<key>.polygon` | `[[x,y], ...]` | 房间区域多边形（可选，无门的房间可省略） |

### 10.6 辅助工具：get_points.py

用于在 RViz 中获取房间轮廓坐标的工具：

```bash
source ~/slam_ws/install/setup.bash
python3 ~/slam_ws/get_points.py
```

- 在 RViz 中使用 **"Publish Point"** 工具点击地图任意位置
- 终端输出该点在地图坐标系中的 `(x, y)` 坐标
- 用于精确测量门口坐标、房间角点等

---

## 11. 手柄遥控

### 11.1 启动方式

```bash

# 启动手柄控制节点
python3 ~/slam_ws/handle/cmd_vel_publisher_node.py
```

> **说明：** 本系统**仅使用** `handle/cmd_vel_publisher_node.py` 进行手柄遥控。Wheelchair 包内置的 `teleop_joystick.py` 和 `rs485_joystick_node.py` 是旧方案，不再使用。

### 11.2 手柄按键映射

| 摇杆/按键 | 操作 | 输出 | 量程 |
|-----------|------|------|------|
| **左摇杆 上下** | 前进 / 后退 | `linear.x` | ±0.5 m/s |
| **右摇杆 左右** | 原地旋转 | `angular.z` | 取决于摇杆 |



### 11.4 技术说明

- 节点基于 `pygame` 读取手柄输入，设置了 `SDL_VIDEODRIVER=dummy` 以在**无显示器**的 Orange Pi 上正常运行
- 自动检测连接的 USB / 蓝牙手柄（取第一个检测到的设备）
- 摇杆死区：±0.04（小于此范围的输入被视为 0）
- 发布频率：10 Hz (`create_timer(0.1)`)
- 发布 topic：`/cmd_vel` (`geometry_msgs/Twist`)

### 11.5 注意事项

- 手柄须在节点启动前已连接
- 若启动时报 "未检测到手柄"，请检查 USB 连接或蓝牙配对状态
- 手柄的 `/cmd_vel` 与 Nav2 的 `/cmd_vel` 共用一个 topic——同时操作时以最后收到的消息为准

---

## 12. 配置文件详解

### 12.1 double_localization.yaml — 双雷达定位参数

**路径：** `src/jt_chair/config/double_localization.yaml`  
**使用：** `double_localization.launch.py`, `double_nav2.launch.py`

| 参数 | 值 | 类别 | 说明 |
|------|-----|------|------|
| `mode` | `localization` | 模式 | 纯定位，不更新地图 |
| `scan_topic` | `/scan_merged` | 输入 | 使用双雷达融合后的扫描 |
| `odom_frame` | `odom` | TF | 里程计坐标系 |
| `map_frame` | `map` | TF | 地图坐标系 |
| `base_frame` | `base_link` | TF | 机器人本体坐标系 |
| `resolution` | 0.05 | 地图 | 5cm 分辨率 |
| `transform_publish_period` | 0.01 | TF | 100Hz TF 发布 |
| `map_update_interval` | 1.0 | 更新 | 每秒更新一次位姿图 |
| `restamp_tf` | true | 同步 | 重打 TF 时间戳（关键） |
| `min_laser_range` | 0.15 m | 感知 | 最小有效距离 |
| `max_laser_range` | 15.0 m | 感知 | 最大有效距离 |
| `minimum_time_interval` | 0.05 s | 更新 | 20Hz 最低处理频率 |
| `transform_timeout` | 0.1 s | 同步 | TF 超时阈值 |
| `minimum_travel_distance` | 0.15 m | 防抖 | 最小移动触发更新 |
| `minimum_travel_heading` | 0.05 rad | 防抖 | 最小旋转触发更新 |
| `scan_buffer_size` | 20 | 缓存 | 扫描缓存量 |
| `do_loop_closing` | true | 回环 | **全局重定位开关** |
| `loop_match_minimum_chain_size` | 10 | 回环 | 最少节点匹配数 |
| `correlation_search_space_dimension` | 0.3 m | 局部 | 局部搜索范围 |
| `loop_search_space_dimension` | 10.0 m | 全局 | 绑架后搜索范围 |
| `loop_search_maximum_distance` | 5.0 m | 全局 | 回环匹配最大距离 |
| `distance_variance_penalty` | 1.5 | 约束 | 里程计距离约束强度 |
| `angle_variance_penalty` | 1.5 | 约束 | 里程计角度约束强度 |
| `minimum_angle_penalty` | 0.8 | 约束 | 最小角度约束 |
| `minimum_distance_penalty` | 0.7 | 约束 | 最小距离约束 |

### 12.2 mapper_params_online_sync.yaml — 建图参数

**路径：** `src/jt_chair/config/mapper_params_online_sync.yaml`  
**使用：** `double_mapping.launch.py`

与定位参数高度一致，关键差异：

| 参数 | 定位值 | 建图值 | 说明 |
|------|--------|--------|------|
| `mode` | `localization` | `mapping` | 建图模式允许写入新关键帧 |
| `distance_variance_penalty` | 1.5 | 2.0 | 建图时更信任里程计，防止错误约束破坏地图 |
| `use_map_saver` | — | `true` | 自动保存地图 |

### 12.3 ekf.yaml — EKF 融合配置

**路径：** `src/jt_chair/config/ekf.yaml`  
**使用：** 所有双雷达 launch 文件

```
频率:      20 Hz
模式:      2D (忽略 Z, Roll, Pitch)
```

**传感器输入及融合配置：**

| 输入 | Topic | 融合信号 | 不融合信号 | 原因 |
|------|-------|---------|-----------|------|
| odom0 | `/odom` | Vx, Vy（线速度）, Vyaw（角速度） | X, Y（绝对位置） | 轮子打滑导致位置不可靠 |
| imu0 | `/imu` | Yaw（磁力计航向）, Vyaw（陀螺仪角速度） | Ax, Ay, Az（加速度） | 消费级 IMU 加速度漂移严重 |

- `imu0_relative: true` — IMU 首帧航向作为零参考，防止启动时航向突变
- 队列深度：两个传感器均为 10
- 不发布 TF（`publish_tf: false` 或留空）

### 12.4 nav2_params_dw.yaml — Nav2 导航参数

**路径：** `src/jt_chair/config/nav2_params_dw.yaml`  
**使用：** `double_nav2.launch.py`

详见 [8.3 导航参数](#83-导航参数)，下面补充完整配置项列表：

**MPPI 完整轨迹生成参数：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `motion_model` | `DiffDrive` | 差速模型 |
| `controller_frequency` | 20 | Hz |
| `time_steps` | 40 | 预测步数 |
| `model_dt` | 0.1 | 每步时间 (s)，共预测 4 秒 |
| `batch_size` | 1000 | 每轮采样轨迹数 |
| `iteration_count` | 1 | 优化迭代次数（计算力有限） |
| `temperature` | 0.6 | 轨迹选择温度（越低越保守） |
| `vx_max` | 0.5 m/s | |
| `vx_min` | -0.1 m/s | 基本禁止倒车 |
| `wz_max` | 1.2 rad/s | |
| `lookahead_dist` | 1.5 m | 前瞻距离 |
| `prune_distance` | 1.5 m | 轨迹剪枝距离 |
| `transform_tolerance` | 0.1 s | |

**ObstaclesCritic 详细参数：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `cost_power` | 1 | 代价乘幂 |
| `repulsion_weight` | 3.0 | 排斥力权重 |
| `critical_weight` | 35.0 | 临界障碍权重 |
| `consider_footprint` | true | 使用机器人 footprint |
| `collision_cost` | 1000000 | 碰撞代价（极大，确保绝不碰撞） |
| `collision_margin` | 0.10 m | 附加安全边距 |
| `near_goal_distance` | 0.5 m | |
| `inflation_layer_name` | `inflation_layer` | |
| `inflation_radius` | 0.5 m | |

### 12.5 waypoints_config.json — 路标点存储

**路径：** `src/jt_chair/config/waypoints_config.json`

详见 [10.5 节](#105-waypoints_configjson-格式)。

---

## 13. ROS2 接口参考

### 13.1 核心 Topic

| Topic | 消息类型 | 方向 | 频率 | 发布者 |
|-------|---------|------|------|--------|
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 20 Hz | Nav2 / 手柄节点 |
| `/odom` | `nav_msgs/Odometry` | 发布 | 20 Hz | `odom_publisher` (dsw_chair) |
| `/imu` | `sensor_msgs/Imu` | 发布 | ~100 Hz | `ahrs_driver_node` (fdilink_ahrs) |
| `/scan_1` | `sensor_msgs/LaserScan` | 发布 | 10 Hz | `lslidar_driver_node1` |
| `/scan_2` | `sensor_msgs/LaserScan` | 发布 | 10 Hz | `lslidar_driver_node2` |
| `/scan_merged` | `sensor_msgs/LaserScan` | 发布 | 20 Hz | `dual_laser_merger` |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 订阅 | 手动 | RViz "2D Pose Estimate" |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 订阅 | 手动 | RViz "2D Goal Pose" |
| `/clicked_point` | `geometry_msgs/PointStamped` | 订阅 | 手动 | RViz "Publish Point" |
| `/drive_profile` | 自定义 | 订阅 | 事件 | 语音模块 / voice_model_publisher |
| `/euler_angles` | `geometry_msgs/Vector3` | 发布 | ~100 Hz | fdilink_ahrs |
| `/magnetic` | `geometry_msgs/Vector3` | 发布 | ~100 Hz | fdilink_ahrs |

### 13.2 核心 Action

| Action | 类型 | 用途 |
|--------|------|------|
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | 语音桥接调用此 Action 发送导航目标 |

### 13.3 关键节点

| 节点名 | 可执行文件 | 包 | 功能 |
|--------|-----------|-----|------|
| `odom_publisher` | `dsw_chair` | dsw_chair | 轮式里程计发布 |
| `voice_nav_bridge` | `voice_nav_bridge` | jt_chair | 语音导航桥接 |
| `cmd_vel_publisher_node` | — (Python) | — | 手柄遥控 |
| `dual_laser_merger` | `dual_laser_merger_node` | dual_laser_merger | 双雷达融合 |
| `slam_toolbox` | `localization_slam_toolbox_node` / `sync_slam_toolbox_node` | slam_toolbox | 定位/建图 |
| `ekf_filter_node` | `ekf_node` | robot_localization | EKF 融合 |
| `ahrs_driver_node` | `ahrs_driver_node` | fdilink_ahrs | IMU 驱动 |
| `lslidar_driver_node1` | `lslidar_driver_node` | lslidar_driver | 激光雷达 1 驱动 |
| `lslidar_driver_node2` | `lslidar_driver_node` | lslidar_driver | 激光雷达 2 驱动 |
| `controller_server` | `controller_server` | nav2_controller | MPPI 控制器 |
| `planner_server` | `planner_server` | nav2_planner | SmacPlanner2D 规划器 |
| `bt_navigator` | `bt_navigator` | nav2_bt_navigator | 行为树导航器 |

### 13.4 TF 帧

| 帧 | 父帧 | 发布者 | 说明 |
|-----|------|--------|------|
| `odom` | `map` | slam_toolbox | 定位修正后的世界坐标变换 |
| `base_link` | `odom` | EKF (robot_localization) | 融合后的机器人位姿 |
| `laser_1` | `base_link` | static_transform_publisher | 前右雷达 (0.29, -0.255, 0.2) |
| `laser_2` | `base_link` | static_transform_publisher | 后左雷达 (-0.29, 0.255, 0.2, yaw≈179°) |
| `imu_link` | `base_link` | static_transform_publisher | IMU 安装位置 (0.185, 0, 0) |

---

## 14. 常见问题与故障排查

### 14.1 定位相关

**Q: 定位漂移，激光点云和地图错位**
- 在 RViz 使用 "2D Pose Estimate" 重新设置初始位姿
- 检查 `double_localization.yaml` 中 `do_loop_closing: true` 是否开启
- 检查 EKF 是否正常运行（`ros2 node list | grep ekf`）
- 确认 `/odom` 和 `/imu` 话题均有数据发布

**Q: 机器人被搬动后无法重新定位**
- 参数 `loop_search_space_dimension: 10.0` 定义了搜索范围
- 若搬动距离超过 10m，先用 2D Pose Estimate 缩小搜索范围
- 增大 `loop_search_space_dimension` 可提升恢复范围但会增加误匹配风险

**Q: 长走廊中定位逐渐漂移**
- 增大 `distance_variance_penalty` 和 `angle_variance_penalty`（当前为 1.5）
- 确保 EKF 正常工作——走廊中 IMU 航向角是重要的约束来源
- 建图时在长走廊中多走几个来回，增加关键帧密度

### 14.2 导航相关

**Q: 门通过失败，机器人在门口卡住或走偏**
- 检查 `waypoints_config.json` 中该房间的 `door.outside` 和 `door.inside` 坐标是否准确
- 门口线段方向应与实际通行方向一致
- 过门 PID 参数在 `voice_nav_bridge.py` 中调节（`Kp_cross`, `Kp_heading`, `Kd_yaw`）
- 确保门口宽度大于机器人 footprint 宽度（0.58m）+ 安全余量

**Q: 导航路径规划失败**
- 检查目标点是否在代价地图中被标记为障碍物
- 局部代价地图尺寸（4m×4m）内是否无可行路径
- 尝试用 RViz 的 "2D Goal Pose" 发送不同位置的目标

**Q: 机器人接近目标但不停止，来回摇摆**
- 目标精度 `xy_goal_tolerance: 0.1 m` 较严格
- 检查是否有局部障碍物阻挡目标附近区域
- 增大 `xy_goal_tolerance` 或 `yaw_goal_tolerance`

### 14.3 硬件相关

**Q: 串口权限错误 "Permission denied"**
```bash
sudo usermod -a -G dialout $USER
# 注销并重新登录
```

**Q: 激光雷达无数据**
- 检查 `/dev/ttyACM0` 和 `/dev/ttyACM1` 是否存在
- 检查雷达电源和 USB 连接
- 实测 topic：`ros2 topic echo /scan_1 --once`

**Q: 电机无响应**
- 检查 `/dev/ttyUSB0` 是否存在
- 检查 ZLAC8015 驱动器是否上电
- 检查 Modbus 从站地址：右轮 `(1<<8)|0`，左轮 `(1<<8)|1`
- 实测里程计：`ros2 topic echo /odom --once`

**Q: IMU 无数据**
- 检查 `/dev/wheeltec_FDI_IMU_GNSS` 是否存在
- 检查 udev 符号链接是否正确
- 实测 IMU：`ros2 topic echo /imu --once`

### 14.4 手柄相关

**Q: 手柄无法检测**
```python
# 报错: "未检测到手柄"
```
- 手柄须在启动节点前已连接
- 检查 USB 连接或蓝牙配对状态
- 确认 `SDL_VIDEODRIVER=dummy` 环境变量已设置（节点代码内置）

**Q: 手柄操作方向不对**
- 确认当前模式（A/B/X/Y），模式会影响允许的运动轴
- 检查 `/cmd_vel` topic 的输出：`ros2 topic echo /cmd_vel`

### 14.5 性能相关

**Q: 系统响应慢，导航轨迹卡顿**
- 检查 CPU 核心绑定是否生效：
  ```bash
  taskset -cp $(pgrep controller_server)
  ```
- 确保 `OMP_NUM_THREADS=4`（启动脚本已设置）
- 检查 CPU 温度（RK3588 过热会降频）

**Q: RViz 显示卡顿**
- 降低 RViz 中的 Topic 订阅频率
- 关闭不必要的显示面板（如 TF 全帧显示）
- 在网络延迟较大的 WiFi 环境下减少订阅带宽

### 14.6 系统相关

**Q: 地图加载失败**
- 地图文件必须是 `.data` + `.posegraph` 成对存在
- launch 文件中的 `map_file` 路径不要写 `.data` 后缀
- 地图文件路径使用绝对路径

**Q: 如何从旧地图切换新地图**
2. 修改 `double_nav2.launch.py` 和 `double_localization.launch.py` 中的 `map_file` 变量（两处都要改）
3. 重新启动

**Q: 如何查看日志排查问题**
```bash
# 查看某节点的输出
ros2 node info /controller_server

# 查看 topic 实时数据
ros2 topic echo /odom --once
ros2 topic hz /scan_merged

# 查看 TF 树
ros2 run tf2_tools view_frames
```

---

## 15. 附录


### 15.2 Launch 文件传感器组合对照表

| Launch 文件 | LSN10P 雷达 | IMU | EKF | 雷达融合 | Nav2 | 语音桥 | 建图/定位 |
|------------|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| `double_mapping.launch.py` | 2 | ✓ | ✓ | ✓ | ✗ | ✗ | 建图 |
| `double_localization.launch.py` | 2 | ✓ | ✓ | ✓ | ✗ | ✓ | 定位 |
| `double_nav2.launch.py` | 2 | ✓ | ✓ | ✓ | ✓ | ✓ | 定位 |
| `localization_nav2.launch.py` | 1 | ✗ | ✗ | ✗ | ✓ | ✗ | 定位 |
| `slam_toolbox_localization.launch.py` | 1 | ✗ | ✗ | ✗ | ✗ | ✗ | 定位 |
| `slam_toolbox_mapping.launch.py` | 1 | ✗ | ✗ | ✗ | ✗ | ✗ | 建图 |

### 15.3 常用命令行速查

```bash
# === 编译 ===
cd ~/slam_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# === 单包编译 ===
colcon build --symlink-install --packages-select jt_chair

# === Source 环境 ===
source ~/slam_ws/install/setup.bash

# === 启动命令 ===
./start_mapping.sh        # 建图
./start_localization.sh   # 重定位
./start_nav2_pro.sh       # 导航

# === 手柄遥控 ===
python3 ~/slam_ws/handle/cmd_vel_publisher_node.py

# === RViz ===
rviz2 

# === 查看节点/话题 ===
ros2 node list
ros2 topic list
ros2 topic echo /odom --once
ros2 topic hz /scan_merged

# === 查看 TF 树 ===
ros2 run tf2_tools view_frames

# === 单发导航目标 (手动测试) ===
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' --once
```
# === 查看 当前机器人坐标 指令 ===
ros2 run tf2_ros tf2_echo map base_link
示例：
At time 1778556906.886650899
- Translation: [-1.687, 4.518, 0.000]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, -0.007, 1.000]
- Rotation: in RPY (radian) [0.000, 0.000, -0.013]
- Rotation: in RPY (degree) [0.000, 0.000, -0.760]
- Matrix:
  1.000  0.013  0.000 -1.687
 -0.013  1.000 -0.000  4.518
 -0.000  0.000  1.000  0.000
  0.000  0.000  0.000  1.000
  #机器人坐标为：[-1.687, 4.518,-0.013]

### 15.4 地图文件路径修改指南

当需要切换地图时，需修改两个文件中的 `map_file` 变量：

**1. `src/jt_chair/launch/double_localization.launch.py` 第 193 行：**
```python
map_file = "/home/orangepi/slam_ws/src/jt_chair/map/你的地图名"
```

**2. `src/jt_chair/launch/double_nav2.launch.py` 中对应行。**

> 地图名称不需要 `.data` 后缀，slam_toolbox 会自动查找 `.data` 和 `.posegraph` 文件对。

---

> 📝 **文档版本：** v1.0  
> 📅 **最后更新：** 2026-07-06  
> 👤 **维护者：** orangepi
