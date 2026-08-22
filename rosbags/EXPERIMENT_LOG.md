# 轮椅原始 rosbag 实验记录

> 每录一包填一行。母包只读保存、不覆盖；任何话题 0 条或骤降的包标记作废，不作正式数据。

| bag 名 | 日期时间 | 地点 | 路线描述 | 时长 | 速度(档/实测) | 人流 | 磁盘(du -sh) | 备注(异常/掉线/接反) |
|---|---|---|---|---|---|---|---|---|
| wheelchair_raw_test_01 | 2026-08-22 17:06:55~17:10:41 | | 试录：原地+短距直线+小转弯 | 3min46s | | | 32 MiB | 验收通过：scan 10Hz×2 / imu 100Hz / odom 20Hz / cmd_vel 10Hz，tf_static=3，无 /tf 漏网；≈8.5 MB/min |

## 正式路线模板（录完补填）

- **wheelchair_raw_loop_01**（闭环综合：走廊+转弯+房间+回起点）
- **wheelchair_raw_corridor_01**（长走廊往返）
- **wheelchair_raw_door_01**（窄门/房间进出）

## 待分叉实验（同一份母包，全部离线做）

- /odom+/imu → EKF → 融合定位 vs 纯 /odom 对照
- scan_1 单雷达 vs scan_1+scan_2 双雷达融合
- SLAM Toolbox vs Cartographer
