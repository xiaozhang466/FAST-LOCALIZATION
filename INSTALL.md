# FAST-LOCALIZATION 项目安装指南

> 本项目集成了 FAST-LOCALIZATION 定位算法、多种雷达驱动、IMU 驱动、AgileX Ranger Mini 底盘驱动及 ROS 导航栈，适用于室外自主导航场景。

---

## 目录

- [1. 环境要求](#1-环境要求)
- [2. 克隆仓库](#2-克隆仓库)
- [3. 安装依赖](#3-安装依赖)
- [4. 编译项目](#4-编译项目)
- [5. 硬件配置](#5-硬件配置)
- [6. 地图配置](#6-地图配置)
- [7. 启动指令](#7-启动指令)
- [8. 项目结构说明](#8-项目结构说明)
- [9. 常见问题](#9-常见问题)

---

## 1. 环境要求

| 项目 | 要求 |
|------|------|
| **系统** | Ubuntu 20.04 |
| **ROS** | ROS Noetic（需安装 `ros-noetic-desktop-full`）|
| **编译器** | GCC 支持 C++14 |
| **CMake** | ≥ 2.8.3 |
| **Python** | Python 3 |

---

## 2. 克隆仓库

```bash
git clone https://github.com/xiaozhang466/FAST-LOCALIZATION.git fast_localization_ws
cd fast_localization_ws
```

---

## 3. 安装依赖

### 3.0 ROS Noetic 完整安装

> 如果新设备尚未安装 ROS，或仅安装了 `ros-base`，请先安装完整版：

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install -y ros-noetic-desktop-full
```

确认安装成功：

```bash
source /opt/ros/noetic/setup.bash
rosversion -d   # 应输出 noetic
```

### 3.1 系统依赖

```bash
sudo apt-get update
sudo apt-get install -y \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    libboost-all-dev \
    libapr1-dev \
    can-utils
```

### 3.2 ROS 依赖

```bash
sudo apt-get install -y \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-eigen-conversions \
    ros-noetic-diagnostic-updater \
    ros-noetic-nodelet \
    ros-noetic-pluginlib \
    ros-noetic-serial \
    ros-noetic-move-base \
    ros-noetic-map-server \
    ros-noetic-costmap-2d \
    ros-noetic-nav-core \
    ros-noetic-navfn \
    ros-noetic-dwa-local-planner \
    ros-noetic-message-generation \
    ros-noetic-message-runtime
```

或者使用 `rosdep` 一键安装：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 3.3 Python 依赖（地图生成脚本，可选）

```bash
pip3 install open3d numpy scipy Pillow
```

---

## 4. 编译项目

```bash
cd fast_localization_ws
catkin_make
source devel/setup.bash
```

> **提示：** 建议将 `source ~/fast_localization_ws/devel/setup.bash` 添加到 `~/.bashrc` 中，避免每次打开终端都要手动 source。

---

## 5. 硬件配置

### 5.1 底盘 CAN 总线配置（AgileX Ranger Mini）

每次开机或重新插拔 CAN 设备后需执行：

```bash
# 加载 CAN 内核模块
sudo modprobe gs_usb

# 启动 CAN 接口（Ranger Mini v2 默认 500kbps）
sudo ip link set can0 up type can bitrate 500000
```

验证 CAN 连接：

```bash
candump can0    # 应能看到底盘发送的数据帧
```

### 5.2 雷达网络配置

#### 镭神 LSLidar C16

| 参数 | 值 |
|------|------|
| 雷达默认 IP | `192.168.1.200` |
| 数据端口 | `2368` |
| **主机网口 IP** | 需设置为 `192.168.1.x`（如 `192.168.1.102`）|

设置主机 IP（以 `eth0` 为例，请替换为实际网口名称）：

```bash
sudo ifconfig eth0 192.168.1.102 netmask 255.255.255.0
```

验证连接：

```bash
ping 192.168.1.200
```

#### Livox MID-360 / AVIA

Livox 雷达通过 **广播码** 自动发现设备，无需手动配置 IP。需确保主机与雷达在同一网段。

### 5.3 IMU 串口配置

IMU 通过 USB 串口连接，确保设备已识别：

```bash
ls /dev/ttyUSB*
```

如遇权限问题：

```bash
sudo chmod 666 /dev/ttyUSB0
# 或将用户加入 dialout 组（永久生效）
sudo usermod -a -G dialout $USER
```

---

## 6. 地图配置

本项目使用两种地图：

| 地图类型 | 文件路径 | 用途 |
|----------|----------|------|
| **3D 点云地图** | `src/FAST-LOCALIZATION/map/pcd/` | FAST-LOCALIZATION 定位用 |
| **2D 栅格地图** | `src/global_map/nav_map.yaml` + `nav_map.pgm` | move_base 导航用 |

### 更换地图

1. **定位地图**：将新的 PCD 文件放入 `src/FAST-LOCALIZATION/map/pcd/` 目录，并更新对应的 `pose.json`
2. **导航地图**：替换 `src/global_map/` 下的 `nav_map.yaml` 和 `nav_map.pgm`

### 从轨迹生成导航地图（可选）

```bash
python3 src/generate_nav_map_from_trajectory.py
```

---

## 7. 启动指令

### 7.1 完整系统启动流程

以 **LSLidar C16 + Ranger Mini** 为例，按顺序在不同终端执行：

```bash
# ---- 终端 1：启动雷达驱动 ----
source devel/setup.bash
roslaunch lslidar_c16_decoder lslidar_c16.launch

# ---- 终端 2：启动 IMU ----
source devel/setup.bash
roslaunch imu_launch imu_msg1.launch

# ---- 终端 3：启动定位 ----
source devel/setup.bash
roslaunch fast_localization localization_lslidar_C16.launch

# ---- 终端 4：启动导航（含底盘驱动）----
source devel/setup.bash
roslaunch navigation nav_bringup.launch
```

### 7.2 各雷达定位启动

| 雷达类型 | 启动命令 |
|----------|----------|
| **LSLidar C16** | `roslaunch fast_localization localization_lslidar_C16.launch` |
| **Livox MID-360** | `roslaunch fast_localization localization_mid360.launch` |
| **Livox AVIA** | `roslaunch fast_localization localization_avia.launch` |
| **Ouster OS1-64** | `roslaunch fast_localization localization_ouster64.launch` |

### 7.3 各雷达驱动启动

| 雷达类型 | 启动命令 |
|----------|----------|
| **LSLidar C16** | `roslaunch lslidar_c16_decoder lslidar_c16.launch` |
| **LSLidar C16 双雷达** | `roslaunch lslidar_c16_decoder lslidar_c16_double.launch` |
| **Livox（PointCloud2）** | `roslaunch livox_ros_driver livox_lidar.launch` |
| **Livox（带 RViz）** | `roslaunch livox_ros_driver livox_lidar_rviz.launch` |
| **Livox（CustomMsg）** | `roslaunch livox_ros_driver livox_lidar_msg.launch` |

### 7.4 IMU 启动

```bash
# 默认 spec 协议
roslaunch imu_launch imu_msg1.launch

# 指定协议类型（可选：spec / 0x91 / 0x62）
roslaunch imu_launch imu_msg1.launch imu_package:=0x91
```

### 7.5 导航启动

```bash
# 完整导航（含底盘、move_base、map_server）
roslaunch navigation nav_bringup.launch

# 关闭避障
roslaunch navigation nav_bringup.launch enable_obstacle_avoidance:=false

# 不启动底盘（仅测试导航逻辑）
roslaunch navigation nav_bringup.launch enable_chassis:=false

# 不启动 RViz
roslaunch navigation nav_bringup.launch rviz:=false
```

### 7.6 底盘单独启动

```bash
# Ranger Mini v2
roslaunch ranger_base ranger_mini_v2.launch

# Ranger Mini v1
roslaunch ranger_base ranger_mini_v1.launch

# Ranger 全尺寸
roslaunch ranger_base ranger.launch
```

---

## 8. 项目结构说明

```
fast_localization_ws/
├── INSTALL.md                          # 本安装指南
├── src/
│   ├── FAST-LOCALIZATION/              # 核心定位算法
│   │   ├── config/                     # 各雷达参数配置（YAML）
│   │   ├── launch/                     # 定位启动文件
│   │   └── map/                        # 3D 点云地图
│   ├── navigation/                     # ROS 导航模块
│   │   ├── config/                     # move_base / costmap 参数
│   │   ├── launch/nav_bringup.launch   # 导航主启动文件
│   │   └── scripts/                    # 速度滤波脚本
│   ├── livox_ros_driver/               # Livox 雷达驱动（含 Livox-SDK）
│   ├── lslidar/                        # 镭神 C16 雷达驱动
│   ├── imu/                            # IMU 串口驱动
│   ├── ranger_ros/                     # AgileX Ranger 底盘 ROS 驱动
│   ├── ugv_sdk/                        # AgileX UGV 底层 SDK
│   ├── global_map/                     # 2D 导航地图文件
│   └── generate_nav_map_from_trajectory.py  # 地图生成工具
├── build/                              # 编译产物（自动生成）
└── devel/                              # 开发空间（自动生成）
```

### 关键话题对照表

| 话题 | 来源 | 用途 |
|------|------|------|
| `/lslidar_point_cloud` | LSLidar C16 驱动 | 点云输入 |
| `/livox/lidar` | Livox 驱动 | 点云输入 |
| `/livox/imu` | Livox 内置 IMU | IMU 数据 |
| `/IMU_data` | 外接 IMU 串口驱动 | IMU 数据 |
| `/cloud_registered` | FAST-LOCALIZATION | 配准后点云（导航避障用） |
| `/Odometry` | FAST-LOCALIZATION | 6DOF 定位结果 |
| `/cmd_vel` | move_base | 底盘运动控制 |
| `/ranger_odom` | Ranger 底盘驱动 | 里程计 |

---

## 9. 常见问题

### 9.1 编译时 Boost 警告

```
catkin_package() DEPENDS on 'boost' but neither 'boost_INCLUDE_DIRS' nor 'boost_LIBRARIES' is defined
```

**这只是警告，不影响编译和运行，可以忽略。**

### 9.2 VTK 相关警告

VTK 的警告信息不影响使用，可以忽略。

### 9.3 CAN 设备未找到

```
Cannot find device "can0"
```

请确认：
1. CAN-USB 适配器已插入
2. 已执行 `sudo modprobe gs_usb`
3. 使用 `ip link show` 查看是否有 `can0` 设备

### 9.4 雷达无法连接

- **LSLidar**：确认主机 IP 在 `192.168.1.x` 网段，且能 `ping 192.168.1.200`
- **Livox**：确认广播码正确，主机和雷达在同一网段

### 9.5 IMU 串口权限不足

```bash
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### 9.6 定位漂移或初始化失败

- 确认 `src/FAST-LOCALIZATION/map/pcd/` 中有正确的点云地图
- LSLidar C16 使用 `localization_mode: 2`（ScanContext 自动初始化）
- MID-360 使用 `localization_mode: 1`（需给定初始位姿），可在对应 YAML 中修改 `pose_init`
