# OpenCV与ROS2的示例程序

## 概要

- 基于 OpenCV 与 ROS2 的机器人视觉应用程序
- 在 Ubuntu 22.04 和 ROS Humble 环境下开发并验证

## 安装

- 安装 OpenCV 相关包
  pip3 install opencv-contrib-python==4.5.5.64

- 安装 ROS2 与 OpenCV 的接口包
  sudo apt install ros-humble-vision-opencv

- 安装 USB 摄像头节点包
  sudo apt install ros-humble-usb-cam

- 安装 Intel RealSense RGB-D 摄像头的 ROS 封装包
  sudo apt install ros-humble-realsense2-camera

- 假设 ROS 工作空间为 ~/airobot_ws
  cd ~/airobot_ws/src

- 获取 JMU-ROBOTICS-VIVA 的 ros2_aruco 包
  git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco

- 获取包含本示例程序的仓库
  git clone https://github.com/AI-Robot-Book-Humble/chapter5

- 获取包含动作接口定义的仓库（来自第2章）
  git clone https://github.com/AI-Robot-Book-Humble/chapter2

- 构建软件包
  cd ~/airobot_ws/
  colcon build

- 设置环境变量（overlay）
  source install/setup.bash

## 实行

5.3.1节：OpenCV 图像处理
- 运行程序
  python3 ~/airobot_ws/src/chapter5/opencv_ros2/opencv_ros2/imgproc_opencv.py
- 查看结果

5.3.2节：ROS 中的 OpenCV 图像处理
- 终端1：启动 USB 摄像头的 usb_cam 节点
  ros2 run usb_cam usb_cam_node_exe
- 终端2：运行图像处理节点
  ros2 run opencv_ros2 imgproc_opencv_ros
- 终端3：查看结果
  - 启动 RQt
    rqt
  - 在 RQt 菜单中选择 Plugins → Visualization → Image View，点击两次以添加两个 Image View 插件
  - 分别将两个插件的 Topic 设置为 /image_raw 和 /result
- 终端4：查看节点与话题之间的连接关系
  - 启动 rqt_graph
    rqt_graph

5.3.3节：深度数据订阅
- 终端1：启动 Intel RealSense D415 深度相机的 ROS 节点
  ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
- 终端2：列出当前话题
  ros2 topic list
- 终端3：显示彩色图像与深度图像
  - 启动 RQt
    rqt
  - 使用 Image View 插件分别显示彩色图像和深度图像的话题

5.5.1节：Canny 边缘检测
- 终端1：启动 USB 摄像头
  ros2 run usb_cam usb_cam_node_exe
- 终端2：运行边缘检测程序
  ros2 run opencv_ros2 canny_edge_detection
- 终端3：查看结果
  - 启动 RQt
    rqt
  - 在 Image View 插件中选择话题 /edges_result 查看结果

5.5.2节：基于 Haar 特征的级联分类器进行人脸检测
- 终端1：启动 USB 摄像头
  ros2 run usb_cam usb_cam_node_exe
- 终端2：运行人脸检测程序
  ros2 run opencv_ros2 face_detection
- 终端3：查看结果
  - 启动 RQt
    rqt
  - 在 Image View 插件中选择话题 /face_detection_result 查看结果

5.6.1节：QR 码检测
- 终端1：启动 USB 摄像头
  ros2 run usb_cam usb_cam_node_exe
- 终端2：运行 QR 码检测程序
  ros2 run opencv_ros2 qrcode_detector
- 终端3：查看文字结果
  ros2 topic echo /qrcode_detector_data
- 终端4：查看图像结果
  - 启动 RQt
    rqt
  - 在 Image View 插件中选择话题 /qrcode_detector_result 查看结果

5.6.2节：使用 ArUco 标记进行位姿估计
- 终端1：生成示例标记图像
  ros2 run ros2_aruco aruco_generate_marker
- 终端2：启动 USB 摄像头
  ros2 run usb_cam usb_cam_node_exe
- 终端3：运行位姿估计程序
  ros2 run opencv_ros2 aruco_node_tf
- 终端4：查看 /tf 话题内容
  ros2 topic echo /tf
- 终端5：可视化标记与相机坐标系
  - 启动 RViz
    rviz2
  - 在 RViz 中将 Fixed Frame 改为 default_cam，点击 Add 添加 TF 显示

## 帮助

- 本示例程序仅在 Ubuntu 系统上验证过。若使用 Windows 开发，可通过 VirtualBox、VMware 等虚拟机安装 Ubuntu 并运行程序。

- 若同时安装了 Python 的 opencv-python 和 opencv-contrib-python 包，会导致冲突报错。  
  请执行以下命令卸载冲突包：
  pip3 uninstall opencv-python

- 运行 aruco_node_tf 时，图像中未显示坐标轴，或 /tf 话题中 translation 为 (0,0,0)、rotation 为 (0,0,0,1)。  
  原因：缺少摄像头标定文件。TF 变换依赖 camera_info，而 camera_info 来自标定结果。  
  请确保存在以下文件：
  ~/.ros/camera_info/default_cam.yaml

## 作者

坦 杰弗里 图 全　TAN Jeffrey Too Chuan

## 历史

- 2024-10-10: 更新至 Ubuntu 22.04 和 ROS Humble
- 2022-08-27: 完善许可证与文档

## 许可证

Copyright (c) 2022-2025, TAN Jeffrey Too Chuan  
All rights reserved.  
This project is licensed under the Apache License 2.0 found in the LICENSE file in the root directory of this project.

## 参考文献

- https://opencv.org/
- https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco