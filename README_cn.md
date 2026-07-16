# OriginCar 智能小车项目

## 项目简介

本项目基于地瓜机器人开发板和 OriginCar 智能小车平台开发，
实现智能小车的环境感知、运动控制和二维码导航等功能。

## 主要功能

- 智能小车运动控制
- 摄像头图像采集
- 二维码识别与导航
- ROS2 节点通信
- 自动路径规划与任务执行

## 项目目录

项目主要代码位于：

```text
origincar/
```

## 硬件环境

- OriginCar 智能小车
- 地瓜机器人开发板
- 摄像头
- 电机和小车底盘

## 软件环境

- Ubuntu
- ROS2
- Python
- OpenCV

## 环境配置

本项目运行在 OriginCar 小车的 ROS2 工作空间中。

打开小车终端，进入工作空间：

```bash
cd /userdata/dev_ws
```

加载项目环境：

```bash
source install/setup.bash
```

## 项目启动

加载环境后，运行二维码导航启动脚本：

```bash
bash /userdata/dev_ws/src/origincar/origincar_system/scripts/start_qr_navigation.sh
```

请根据实际项目文件位置调整启动命令。

## 使用说明

1. 将项目代码放入 ROS2 工作空间。
2. 编译工作空间。
3. 加载项目环境。
4. 运行对应的启动脚本。
5. 检查小车、摄像头和网络连接是否正常。

## 团队信息

- 学校：绍兴大学
- 项目名称：OriginCar 智能小车系统
