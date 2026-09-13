# Step 1: LiDAR 驱动 + 状态估计 + PX4 桥接

本步骤负责把 Livox Mid-360 点云送入 FAST-LIO2，再把 FAST-LIO2 的里程计结果通过 MAVROS 注入 PX4 EKF2。

| 项目 | 内容 |
|------|------|
| Linux 版本 | Ubuntu 20.04 |
| ROS 版本 | Noetic |
| 机载电脑 | Orange Pi 5 Max |
| 激光雷达 | Livox Mid-360 |
| 飞控 | CUAV X7+ (PX4 1.13.3) |

---

## 一、运行 Mid-360

### 1. 硬件连接

将 Mid-360 的一分三航空线网口接入机载电脑网口，然后给 Mid-360 上电。

### 2. 配网

Mid-360 默认 IP 通常为 `192.168.1.1xx`（`xx` 为 SN 码后两位），机载电脑需配置同网段静态 IP。

```bash
# 查看网卡
ls /sys/class/net/

# 配置静态 IP，按实际网卡名替换 enP3p49s0
sudo ip addr add 192.168.1.50/24 dev enP3p49s0
sudo ip link set enP3p49s0 up

# 测试连接，按实际雷达 IP 替换 xx
ping 192.168.1.1xx

重要提醒：你当前的配置是“临时的”
你使用的 sudo ip addr add ... 和 sudo ip link set ... 命令是临时生效的。
这意味着：如果你重启了电脑，或者插拔了网线、重启了网络服务，这个 IP 配置就会丢失，你需要重新敲这两行命令。
建议（永久配置方法）：
如果你在使用 Ubuntu 桌面版，建议直接在右上角的 网络设置 (Settings -> Network) 中，找到你的有线网卡（enP3p49s0），点击齿轮图标：
选择 IPv4 标签页。
将方法改为 手动 (Manual)。
地址 (Address) 填入：192.168.1.50
子网掩码 (Netmask) 填入：255.255.255.0
网关 (Gateway) 可以留空，或者填 192.168.1.1。
点击应用并重新连接网络。这样重启后依然有效。
```

### 3. 安装 Livox-SDK2

```bash
mkdir -p ~/livox_ws/3rd_party
cd ~/livox_ws/3rd_party
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake ..
make -j
sudo make install
```

编辑 `~/livox_ws/3rd_party/Livox-SDK2/samples/livox_lidar_quick_start/mid360_config.json`，将 `host_ip` 改为机载电脑 IP，例如 `192.168.1.50`。

### 4. 安装 livox_ros_driver2

```bash
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
./build.sh ROS1
source ~/livox_ws/devel/setup.bash
```

编辑 `~/livox_ws/src/livox_ros_driver2/config/MID360_config.json`：

- `host_net_info` 下的 IP 改为机载电脑 IP，例如 `192.168.1.50`
- `lidar_configs` 下的 IP 改为 Mid-360 实际 IP，例如 `192.168.1.1xx`

测试：

```bash
roslaunch livox_ros_driver2 msg_MID360.launch
mid360s用
roslaunch livox_ros_driver2 msg_MID360s.launch
看点云图：
roslaunch livox_ros_driver2 rviz_MID360.launch
```

RViz 中应能看到 Mid-360 点云。

---

## 二、安装 FAST-LIO2

FAST_LIO2编译依赖livox_ros_driver2,请先编译livox_ros_driver2
```bash
mkdir -p ~/fast_lio2_ws/src
cd ~/fast_lio2_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ~/fast_lio2_ws
catkin_make
source ~/fast_lio2_ws/devel/setup.bash
```

官方 hku-mars FAST-LIO 默认依赖 `livox_ros_driver`。如果你使用 `livox_ros_driver2`，需把 FAST-LIO 中的相关依赖名和 include 路径从 `livox_ros_driver` 改为 `livox_ros_driver2`，主要涉及：

| 文件 | 修改项 |
|------|--------|
| `src/FAST_LIO/CMakeLists.txt` | `find_package` 依赖名 |
| `src/FAST_LIO/src/laserMapping.cpp` | Livox 消息 include |
| `src/FAST_LIO/src/preprocess.h` | Livox 消息 include |
| `src/FAST_LIO/src/preprocess.cpp` | Livox 消息 include |

如果编译时找不到 `livox_ros_driver2`：

```bash
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/livox_ws/devel
catkin_make
```

运行测试：

```bash
# Terminal 1
source ~/livox_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch

# Terminal 2
source ~/fast_lio2_ws/devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
```

确认 `/Odometry` 和 `/cloud_registered_body` 有数据。

### FAST-LIO2 TF 说明

修改 FAST-LIO2 源码：
及cfg里面的rviz文件

- 父坐标系：`odom`
- 子坐标系：`base_link`

也就是 FAST-LIO2 现发布 `odom -> base_link`。(官方默认版本为 `camera_init -> body`，修改后统一了 ROS 导航的标准 TF 树，避免与 Cartographer 等组件的坐标系冲突。)

本项目中，Cartographer 负责 `map -> odom` 的动态 TF，FAST-LIO2 负责 `odom -> base_link`。

---

## 三、编译 lidar_to_mavros 桥接节点

```bash
mkdir -p ~/trans_ws/src
cp -r /path/to/Highly_modular_autonomous_drone-main/1.mid360-drone/lidar_to_mavros ~/trans_ws/src/
cd ~/trans_ws
catkin_make
source ~/trans_ws/devel/setup.bash
```

节点功能：

- 订阅 `/Odometry`，读取 FAST-LIO2 位姿
- 发布 `/mavros/vision_pose/pose`，供 PX4 EKF2 融合
- 控制台每秒输出 LiDAR 与 PX4 位姿对比

当前桥接节点会保留 FAST-LIO2 odom 的原始时间戳；如果原始时间戳为空才使用当前 ROS 时间。这样后续用 `rostopic delay /mavros/vision_pose/pose` 调 `EKF2_EV_DELAY` 更可靠。

如果用的mid360s：
将lidar_to_mavros/src/launch 里面的lidar_to_mavros.launch里的msg_MID360.launch改为msg_MID360s.launch.
---

## 四、配置 shell 环境

将以下内容按你的实际工作空间路径加入 `~/.bashrc`：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash --extend
source ~/fast_lio2_ws/devel/setup.bash --extend
source ~/trans_ws/devel/setup.bash --extend
```

然后执行：

```bash
source ~/.bashrc
```

---

## 五、PX4 EKF2 参数

使用 QGroundControl 连接飞控，重点检查：

| 参数 | 建议 | 说明 |
|------|------|------|
| `EKF2_EV_CTRL` | 启用水平/垂直位置和偏航融合 | 开启视觉位置融合 |
| `EKF2_HGT_MODE` | Vision | 高度源使用视觉 |
| `EKF2_GPS_CTRL` | 关闭 GPS 融合 | 室内/无 GPS 场景 |
| `EKF2_EV_DELAY` | 实测 | Mid-360 + FAST-LIO2 延迟需现场测 |
| `EKF2_EV_POS_X/Y/Z` | 实际安装外参 | LiDAR/估计源相对飞控中心的偏移 |
| `EKF2_EVP_NOISE` | 配合调优 | 视觉位置噪声 |
| `EKF2_EVA_NOISE` | 配合调优 | 视觉姿态噪声 |

延迟测量：

```bash
rostopic delay /mavros/vision_pose/pose
rostopic delay /mavros/local_position/pose
```

---

## 六、初步测试

```bash
cd trans_ws
source ~/trans_ws/devel/setup.bash
roslaunch lidar_to_mavros lidar_to_mavros.launch
```

该 launch 会启动：

1. MAVROS `px4.launch`
2. Livox ROS Driver 2 `msg_MID360.launch`
3. FAST-LIO2 `mapping_mid360.launch`
4. `lidar_to_mavros` 桥接节点

首次飞行前请先手持测试：

1. 确认 `/Odometry`、`/mavros/vision_pose/pose`、`/mavros/local_position/pose` 都有数据
2. 拿着飞机缓慢移动一圈，回到起点后观察位置是否明显漂移
3. 若漂移明显，先不要解锁飞行，优先检查外参、时间延迟和 EKF2 参数
