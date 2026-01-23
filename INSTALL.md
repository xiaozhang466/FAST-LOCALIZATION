# FAST-LOCALIZATION 安装指南

## 克隆仓库

```bash
git clone https://github.com/xiaozhang466/FAST-LOCALIZATION.git fast_localization_ws
cd fast_localization_ws
```

## 编译说明

### 直接编译

```bash
catkin_make
source devel/setup.bash
```

**注意：** 由于 `livox_ros_driver` 采用了嵌套目录结构（`src/livox_ros_driver/livox_ros_driver/`），这是正常的，编译系统会正确处理。

## 依赖安装

如果缺少依赖，运行：

```bash
# 安装 ROS 依赖
sudo apt-get update
sudo apt-get install ros-noetic-pcl-ros ros-noetic-pcl-conversions

# 使用 rosdep 安装所有依赖
rosdep install --from-paths src --ignore-src -r -y
```

## 常见问题

### 1. boost 警告
```
catkin_package() DEPENDS on 'boost' but neither 'boost_INCLUDE_DIRS' nor 'boost_LIBRARIES' is defined
```
这只是警告，不影响编译，可以忽略。

### 2. VTK 相关警告
VTK 的警告可以忽略，不影响使用。

## 运行

```bash
# Livox Avia
roslaunch fast_localization localization_avia.launch

# Livox MID-360
roslaunch fast_localization localization_mid360.launch

# LSLidar C16
roslaunch fast_localization localization_lslidar_C16.launch

# Ouster64
roslaunch fast_localization localization_ouster64.launch
```
