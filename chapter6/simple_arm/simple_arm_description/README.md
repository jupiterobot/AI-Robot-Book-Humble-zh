# 简易的2自由度机器人臂模型（修订第2版）

## 概要

- 用于了解机器人臂的基本结构
- URDF 文件示例
- 在 RViz 中显示机器人模型的示例

## 安装

- 假设 ROS 工作空间为 ~/airobot_ws
  cd ~/airobot_ws/src

- 获取包含本包的仓库
  git clone https://github.com/AI-Robot-Book-Humble/chapter6

- 构建软件包
  sudo apt install ros-humble-joint-state-publisher-gui
  cd ~/airobot_ws
  colcon build --packages-select simple_arm_description

## 实行

- 在终端中执行以下命令
  source install/setup.bash
  ros2 launch simple_arm_description display.launch.py
- 操作弹出的 joint_state_publisher_gui 窗口中的滑块，即可控制关节角度。

## 帮助

## 作者

升谷 保博

## 历史

- 2023-10-13: 在 ROS Humble 上完成运行验证
- 2022-08-23: 完善许可证与文档

## 许可证

Copyright (c) 2022-2025, MASUTANI Yasuhiro  
All rights reserved.  
This project is licensed under the Apache License 2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献