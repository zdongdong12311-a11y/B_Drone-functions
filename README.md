# Highly Modular Autonomous Drone

基于 **Livox Mid-360 激光雷达** 的模块化无人机自主避障、建图与导航系统。支持多航点自主飞行、视觉识别抓取投放、多层安全保护机制。

## 硬件配置

| 组件 | 型号 | 备注 |
|------|------|------|
| 机载电脑 | Orange Pi 5 Max | RK3588, 8GB+ RAM |
| 飞控 | CUAV X7+ | PX4 1.13.3 |
| 激光雷达 | Livox Mid-360 | 非重复扫描，360° 视场 |
| 摄像头 (可选) | USB 1080p | MJPG, 用于视觉识别 |
| 舵机爪 (可选) | 铝合金铁爪 + ESP8266 | 串口控制，抓取/投放 |
| 舵机 (可选) | mg90/sg90 + pico | 串口控制，投放 |

## 软件环境

| 组件 | 版本 |
|------|------|
| OS | Ubuntu 20.04 (aarch64) |
| ROS | Noetic |
| PX4 | 1.13.3 |
| Python | 3.8+ |
| OpenCV | 4.x (视觉识别模块) |
| YOLO | V5/V8 .etc|



## 系统架构

```
Livox Mid-360 LiDAR
    │
    ├─→ Livox-SDK2 → livox_ros_driver2 → 原始点云
    │
    ├─→ FAST-LIO2 → /Odometry
    │       │
    │       ├─→ lidar_to_mavros → /mavros/vision_pose/pose → PX4 EKF2 (视觉位置融合)
    │       │
    │       └─→ /cloud_registered_body (去畸变点云)
    │               │
    │               └─→ pointcloud_to_laserscan → /scan (2D 激光切片)
    │                       │
    │                       ├─→ Cartographer 2D SLAM → /map (建图与定位)
    │                       │
    │                       └─→ move_base (GlobalPlanner + DWA) → /cmd_vel
    │                               │
    │                               └─→ navigation.py → 航点任务 + 高度控制
    │
    └─→ MAVROS ↔ PX4 (飞控通信)
```

## 首飞自检
```
rostopic hz /scan                                     # 切片数据在流
rostopic hz /mavros/local_position/odom               # DWA 的速度源
rosrun tf tf_echo map base_link                       # TF 链完整 (map->odom->base_link)
rosrun tf tf_echo map odom                            # 偏移量, 起飞后 log 也会打印
rostopic echo /move_base/status -n1                   # 状态流正常
```
## 坐标变换 (TF) 树

![TF Tree](./tf_tree.png)


| 变换 | 发布者 | 说明 |
|------|--------|------|
| `map` → `odom` | Cartographer | SLAM 实时估计 |
| `odom` → `base_link` | FAST-LIO2 | 激光雷达里程计 (原 camera_init → body_ 修改而来) |

> **重要**: 不要额外静态发布 `map→odom`，Cartographer 负责这些动态 TF，重复发布会导致冲突。

另一种树：
```
map ──┬─→ camera_init ──→ body ──→ base_link ──┬─→ base_link_frd
      └─→ map_ned                              └─→ livox_frame
```

## 目录结构

```
drone/
├── README.md                    ← 本文档
├── start.sh                     ← 一键启动脚本 (含健康检查)
├── navigation.py                ← 自主导航脚本 (纯导航)
├── opencv_nav_micro.py          ← 视觉+爪控制+导航 (完整任务)
├── point.txt                    ← 航点文件 (支持 # 注释)
│
├── 1.mid360-drone/              ← Step 1: LiDAR 驱动 + 状态估计 + PX4 桥接
│   ├── README.md                ← 详细安装指南
│   └── lidar_to_mavros/         ← ROS 包: FAST-LIO2 → PX4 vision_pose 桥接
│       ├── launch/lidar_to_mavros.launch
│       ├── src/lidar_to_mavros.cpp
│       ├── CMakeLists.txt
│       └── package.xml
│
├── 2.3D_to_2D/                  ← Step 2: 3D 点云转 2D 激光扫描
│   ├── README.md
│   └── point_to_scan.launch
│
├── 3.track_nav/                 ← Step 3: SLAM + 导航
│   ├── README.md
│   ├── cartographer/            ← Cartographer 2D SLAM 配置
│   │   ├── launch/livox.launch
│   │   └── lua/livox.lua
│   └── navigation/              ← ROS Navigation Stack 配置
│       └── move_base/
│           ├── launch/nav_3dto2d.launch
│           ├── config/nav.rviz
│           └── yaml/            ← 6 个参数文件
│
└── Modular_fuctions/            ← 模块化功能集
    ├── opecv_RGB_舵机控制铝合金铁爪/   ← OpenCV 颜色识别 + 舵机爪
    │   ├── opencv_nav_micro.py
    │   ├── R.png / G.png / B.png
    │   └── README.md
    └── rknn-yolov8-master/      ← RK3588 NPU 加速 YOLOv8
        ├── src/                 ← C++ 源码 (三线程流水线)
        ├── launch/
        ├── weights/
        └── README.md
```

## 前置安装

每个步骤的详细安装指南见对应目录的 README:

1. **[1.mid360-drone/README.md](./1.mid360-drone/README.md)** — Mid-360 配网与驱动
   - Livox-SDK2 安装
   - livox_ros_driver2 编译
   - FAST-LIO2 安装 (需修改源码适配 driver2)
   - lidar_to_mavros 编译
   - PX4 EKF2 参数调优

2. **[2.3D_to_2D/README.md](./2.3D_to_2D/README.md)** — 点云转激光扫描
   - pointcloud_to_laserscan 安装

3. **[3.track_nav/README.md](./3.track_nav/README.md)** — SLAM + 导航
   - Cartographer 2D SLAM 安装与配置
   - ROS Navigation Stack (move_base) 安装与配置

4. **[Modular_fuctions/](./Modular_fuctions/)** — 扩展功能
   - OpenCV 颜色识别 + 舵机爪控制
   - RK3588 NPU 加速 YOLOv8 目标检测

## 快速启动

完成前置安装后:

```bash
# 1. 确保 ROS 环境已 source或写入bashrc
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash --extend
source ~/fast_lio2_ws/devel/setup.bash --extend
source ~/trans_ws/devel/setup.bash --extend
source ~/3D_to_2D_ws/devel/setup.bash --extend
source ~/my_carto/install_isolated/setup.bash --extend
source ~/ros_nav_ws/devel/setup.bash --extend

source ~/.bashrc
# 2. 一键启动所有 ROS 节点 (含健康检查)
chmod +x start.sh
./start.sh

# 3. 在新终端运行自主导航 
python3 navigation.py              # 纯航点导航


退出 `start.sh` (Ctrl+C) 时会自动清理所有后台节点。

### ROS 参数配置

导航脚本支持通过 ROS 参数服务器动态配置 (无需改代码):

```bash
# 通过命令行参数覆盖默认值
python3 navigation.py _takeoff_height:=1.0 _waypoint_timeout:=60.0

# 通过 launch 文件配置
<node pkg="navigation_controller" type="navigation.py" name="nav" output="screen">
    <param name="takeoff_height" value="1.0" />
    <param name="kp_z" value="2.0" />
    <param name="waypoint_xy_tol" value="0.2" />
    <param name="low_battery_threshold" value="15.0" />
</node>
```

## 航点文件格式

编辑 `point.txt`，每行一个航点:

```
约定：航点文件每行 x y z 悬停秒数,XY 是 map 系坐标，z 是相对起飞点的高度
# 格式: x y z hover_time
# x          - 目标点 X 坐标 (米, map 坐标系)
# y          - 目标点 Y 坐标 (米, map 坐标系)
# z          - 目标高度 (米, 相对起飞点)
# hover_time - 到达后悬停时间 (秒)

0.0  0.0  0.6  2.0
1.0  0.0  0.8  3.0
1.0  1.0  1.0  3.0
0.0  1.0  0.8  3.0
0.0  0.0  0.6  2.0
```

## 架构说明与已知限制

### 3D → 2D 转换

`pointcloud_to_laserscan` 从去畸变点云 (`/cloud_registered_body`) 中提取高度范围 **-0.05m ~ +0.1m** 的水平切片，生成 2D 激光扫描。

### FAST-LIO2 坐标系说明

由于已修改 FAST-LIO2 源码：
- 父坐标系: `odom`
- 子坐标系: `base_link`

(官方默认源码中为 `camera_init` 和 `body`，修改后统一了 ROS 导航的标准 TF 树，防止与 Cartographer 的坐标系冲突。)

> 如果你使用社区 fork (如 guzhaoyuan 的版本)，可能有 `lidar_odometry_frame_id` 等参数。这种情况下需统一 TF 名称，避免与 Cartographer 的 `map` 帧冲突。

### PX4 EKF2 参数

| 参数 | 建议 | 说明 |
|------|------|------|
| `EKF2_EV_CTRL` | 启用水平/垂直位置和偏航融合 | 开启视觉位置融合 |
| `EKF2_AID_MASK` | 启用水平/垂直位置和偏航融合 | 开启视觉位置融合 |(旧版)
| `EKF2_HGT_MODE` | Vision | 高度源使用视觉 |
| `EKF2_GPS_CTRL` | 关闭 GPS 融合 | 室内/无 GPS 场景 |
| `EKF2_EV_DELAY` | 实测 | Mid-360 + FAST-LIO2 延迟需现场测 |
| `EKF2_EV_POS_X/Y/Z` | 实际安装外参 | LiDAR 相对飞控中心的偏移 |
| `EKF2_EVP_NOISE` | 配合调优 | 视觉位置噪声 |
| `EKF2_EVA_NOISE` | 配合调优 | 视觉姿态噪声 |

## 自主导航脚本详解

### `navigation_simple.py` — 纯航点导航

执行流程:
1. 连接 MAVROS，等待飞控就绪
2. 切换 OFFBOARD 模式并解锁
3. 自动起飞到指定高度
4. 读取 `point.txt` 航点
5. 逐航点导航 (XY 由 move_base 规划，Z 由 PD 控制)
6. 到达每个航点后悬停指定时长
7. 全部完成后安全降落

## 常见问题

| 问题 | 原因 | 解决 |
| Cartographer 报 `Could not find livox.lua` | launch 文件路径不对 | 使用本项目 `3.track_nav/cartographer/launch/livox.launch` |
| move_base 运行但 `/cmd_vel` 无输出 | TF 树不完整 | `rosrun tf tf_echo map base_link` |
| 建图漂移严重 | 扫描匹配参数不当 | 调整 `livox.lua` 中的 `translation_weight` 和 `min_score` |
| 起飞后漂移 | OFFBOARD 前设定点未锁位 | 已在代码中修复: 锁死 lock_x/lock_y/lock_yaw |
| 爪子串口连接失败 | 权限不足或设备不存在 | `sudo chmod 666 /dev/ttyUSB0` |
| FAST-LIO2 编译找不到 driver2 | CMAKE_PREFIX_PATH 未设置 | `export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/livox_ws/devel` |


