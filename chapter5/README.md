# 第５章 视觉

## 概要

《ROS 2与Python动手学AI机器人入门 修订第2版》（出村・萩原・升谷・坦 著，讲谈社）第５章的示例程序与补充信息等。

## 目录结构

- [opencv_ros2](opencv_ros2)： OpenCV 与 ROS2 的示例程序

- [yolov8_ros2](yolov8_ros2)： YOLOv8 与 ROS2 的示例程序

## 补充信息

- 请事先确认在您使用的 Ubuntu 环境中能够正常从摄像头获取输入。

- 启动摄像头时，若出现以下错误：
  Cannot open '/dev/video0': 13, Permission denied
  解决方法：
  sudo chmod 666 /dev/video0