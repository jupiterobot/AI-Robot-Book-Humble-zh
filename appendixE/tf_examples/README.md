# 用于说明TF的示例程序（修订第2版）

## 概要

- 使用Python操作ROS 2中TF（TF2）的示例程序。
- 在 Ubuntu 22.04 和 ROS Humble 环境下开发并验证。

## 安装

- 假设 ROS 工作空间为 `~/airobot_ws`：
  cd ~/airobot_ws/src

- 获取包含本包和 simple_arm 的仓库：
  git clone https://github.com/AI-Robot-Book-Humble/appendixE
  git clone https://github.com/AI-Robot-Book-Humble/chapter6

- 构建包：
  sudo apt -y install ros-humble-tf-transformations
  pip3 install transforms3d
  cd ~/airobot_ws
  colcon build --packages-select simple_arm_description tf_examples

## 运行

### TF 广播器与监听器的简单示例

- 以下所有终端均假设已执行过以下命令：
  source ~/airobot_ws/install/setup.bash

- 终端1：
  ros2 run tf_examples planet_broadcaster

- 终端2：
  - 删除 RViz 的旧配置：
    rm ~/.rviz2/default.rviz
  - 启动 RViz：
    rviz2

- 终端3：
  ros2 run tf_examples satellite_broadcaster

- 终端4：
  ros2 run tf_examples satellite_listener

- 终端5：
  ros2 run tf_examples planet_broadcaster planet2 1 8

- 终端6：
  - 查看话题：
    ros2 topic list
    ros2 topic echo /tf
    ros2 topic echo /tf_static
    ros2 topic echo /pose
  - 查看节点图：
    rqt_graph

- 终端4：
  - 按 Ctrl+C 终止正在运行的 `satellite_listener`。
  - 执行以下命令：
    ros2 run tf_examples satellite_listener planet2

- Launch 文件：
  - 以下命令可一次性启动上述所有内容：
    ros2 launch tf_examples solar_system.launch.py

### 与 URDF 和 Launch 文件结合使用・传感器数据的坐标变换

- 打开新终端，执行以下命令：
  source ~/airobot_ws/install/local_setup.bash
  ros2 launch tf_examples simple_arm.launch.py

## 帮助

- ROS 2 的 Python 接口中未提供与 C++ 中 `tf2_ros::MessageFilter` 等效的功能。
  - 因此，无法对传感器数据的时间戳与 TF 变换的时间戳进行自动对齐。
  - 在 simple_arm 示例中，人为地延迟了传感器数据的时间戳以规避该问题。

## 作者

升谷 保博

## 历史

- 2023-10-15: 支持 ROS Humble
- 2022-08-23: 完善许可证与文档

## 许可证

Copyright (c) 2022-2025 MASUTANI Yasuhiro  
All rights reserved.  
This project is licensed under the Apache License 2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献

- [Using stamped datatypes with tf2_ros::MessageFilter](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Using-Stamped-Datatypes-With-Tf2-Ros-MessageFilter.html)