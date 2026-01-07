# YOLOv8与ROS2的示例程序

## 概要

- 基于 YOLOv8 与 ROS2 的深度学习物体检测程序
- 在 Ubuntu 22.04 和 ROS Humble 环境下开发并验证

## 安装

- 完成 [opencv_ros2](../opencv_ros2/README.md) 的安装步骤

- 安装 YOLOv8 软件
  pip3 install ultralytics
  pip3 uninstall -y opencv-python
  注：opencv-python 会随 ultralytics 自动安装。为避免与 opencv-contrib-python 冲突，需手动卸载。

## 实行

5.7.2节：YOLO 物体检测
- 终端1：启动 USB 摄像头的 usb_cam 节点
  ros2 run usb_cam usb_cam_node_exe
- 终端2：运行物体检测程序
  ros2 run yolov8_ros2 object_detection
- 程序正常运行后，将弹出新窗口，显示带有彩色边界框的检测结果图像。

5.7.2节：检测物体的位置估计
- 终端1：启动 Intel RealSense D415 深度相机的 ROS 节点
  ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
- 终端2：运行带深度信息的物体检测程序
  ros2 run yolov8_ros2 object_detection_tf
- 程序正常运行后，将弹出新窗口，在深度图像上显示目标物体的边界框。
- /tf 话题将输出物体在相机坐标系下的三维位置。

5.7.2节：物体检测的动作服务器
- 终端1：启动 Intel RealSense D415 深度相机的 ROS 节点
  ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
- 终端2：启动动作服务器
  ros2 run yolov8_ros2 object_detection_action_server
- 终端3：调用 ROS 动作接口（以目标物体 'cup' 为例）
  - 查找目标物体
    ros2 action send_goal /vision/command airobot_interfaces/action/StringCommand "{command: find cup}"
  - 持续追踪目标物体
    ros2 action send_goal /vision/command airobot_interfaces/action/StringCommand "{command: track cup}"
  - 停止物体检测处理
    ros2 action send_goal /vision/command airobot_interfaces/action/StringCommand "{command: stop}"

## 帮助

- 本示例程序仅在 Ubuntu 系统上验证过。若使用 Windows 开发，可通过 VirtualBox、VMware 等虚拟机安装 Ubuntu 并运行程序。

## 作者

TAN Jeffrey Too Chuan

## 历史

- 2024-10-10: 更新至 Ubuntu 22.04 和 ROS Humble
- 2022-08-27: 完善许可证与文档

## 许可证

Copyright (c) 2022-2025, TAN Jeffrey Too Chuan  
All rights reserved.  
This project is licensed under the Apache License 2.0 found in the LICENSE file in the root directory of this project.

## 参考文献

- https://github.com/ultralytics/ultralytics