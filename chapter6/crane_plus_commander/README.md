# 使用 CRANE+ V2 的 ROS2 节点群的简易控制节点（修订第2版）

## 概要

- 使用由 RT 公司（アールティ社）公开的 [CRANE+ V2 专用 ROS 2 节点包 crane_plus](https://github.com/rt-net/crane_plus) 的控制节点。
- 支持实机与仿真两种模式。
- 所有节点程序均使用 Python 编写。
- 包含两类控制方式：不使用 MoveIt 直接向各关节发送指令，以及使用 MoveIt 进行规划控制。
- 使用 [pymoveit2](https://github.com/AndrejOrsula/pymoveit2) 作为 MoveIt 2 的 Python 接口。
- 所使用的 crane_plus 并非 RT 公司的[原始仓库](https://github.com/rt-net/crane_plus)，而是从其[派生的版本](https://github.com/AI-Robot-Book-Humble/crane_plus)。
- 在 Ubuntu 22.04 和 ROS Humble 环境下开发并验证。

## 准备

- 若使用 CRANE+ V2 实机，请按照 [crane_plus_control 的 README](https://github.com/rt-net/crane_plus/blob/master/crane_plus_control/README.md) 进行设置：
  - 要点包括：
    - 1. USB 通信端口权限设置（例如 `sudo chmod 666 /dev/ttyUSB0`）
    - 2. 修改 USB 通信端口的 latency_timer
    - 3. 设置机器人各执行器的 Return Delay Time
  - 对于第 1 和第 2 项，可通过在 `/etc/udev/rules.d` 中添加规则文件，实现每次插入 USB 设备时自动配置  
    （详见 [ROBOTIS 官方文档](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/#copy-rules-file)）。
  - 若在 Linux 主机上使用本书提供的 Docker 镜像，请在主机侧完成 CRANE+ V2 的连接设置与运行验证。

## 安装

- 假设 ROS 工作空间为 ～/airobot_ws
  cd ～/airobot_ws/src

- 克隆 AI-Robot-Book-Humble 维护的 crane_plus 仓库（非 RT 官方原版）
  git clone https://github.com/AI-Robot-Book-Humble/crane_plus

- 按照 [crane_plus 的 README](https://github.com/AI-Robot-Book-Humble/crane_plus/blob/master/README.md) 进行依赖安装与构建
  rosdep install -r -y -i --from-paths .
  cd ～/airobot_ws
  colcon build
  source install/setup.bash

- 获取并构建 pymoveit2 包
  cd ～/airobot_ws/src
  git clone https://github.com/AndrejOrsula/pymoveit2
  rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .
  cd ～/airobot_ws
  colcon build --merge-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
  source install/setup.bash

- 获取包含本章示例程序的仓库
  cd ～/airobot_ws/src
  git clone https://github.com/AI-Robot-Book-Humble/chapter6

- 获取定义动作接口的仓库（来自第2章）
  git clone https://github.com/AI-Robot-Book-Humble/chapter2

- 获取包含动作测试客户端的仓库（附录B）
  git clone https://github.com/AI-Robot-Book-Humble/appendixB

- 构建相关软件包
  cd ～/airobot_ws
  colcon build --packages-select airobot_interfaces crane_plus_commander
  source install/setup.bash

## 运行（不使用 MoveIt）

- 终端1
  - 设置环境变量
    cd ～/airobot_ws
    source install/setup.bash

  - 使用实机时
    ros2 launch crane_plus_examples no_moveit_demo.launch.py
  - 使用 Ignition Gazebo 仿真替代实机时
    ros2 launch crane_plus_gazebo no_moveit_crane_plus_with_table.launch.py 

- 终端2
  - 设置环境变量
    cd ～/airobot_ws
    source install/setup.bash
  - 通过键盘控制关节角度
    ros2 run crane_plus_commander commander1

  - 通过键盘控制末端位置（基于正运动学）
    ros2 run crane_plus_commander commander2

  - 控制关节并实时显示关节状态
    ros2 run crane_plus_commander commander3

  - 使用同步式（等待结果返回）的动作客户端
    ros2 run crane_plus_commander commander4

  - 将末端移动到 tf 坐标系中指定的目标点
    ros2 run crane_plus_commander commander5

## 运行（使用 MoveIt）

- 终端1
  - 设置环境变量
    cd ～/airobot_ws
    source install/setup.bash

  - 使用实机时
    ros2 launch crane_plus_examples endtip_demo.launch.py 
  - 使用 Ignition Gazebo 仿真替代实机时
    ros2 launch crane_plus_gazebo endtip_crane_plus_with_table.launch.py 

- 终端2
  - 设置环境变量
    cd ～/airobot_ws
    source install/setup.bash
  - 通过键盘控制末端位置（使用 MoveIt 规划）
    ros2 run crane_plus_commander commander2_moveit

  - 将末端移动到 tf 坐标系中指定的目标点（使用 MoveIt）
    ros2 run crane_plus_commander commander5_moveit

  - 以动作服务器模式运行（使用 MoveIt）
    ros2 run crane_plus_commander commander6_moveit

- 终端3（用于测试动作服务器）
  - 运行测试客户端
    cd ～/airobot_ws
    source install/setup.bash
    ros2 run airobot_action test_client /manipulation/command

## 帮助

## 作者

升谷 保博

## 历史

- 2024-09-15: 引入 MoveIt 支持等更新
- 2023-10-15: 适配 ROS Humble
- 2022-08-23: 完善许可证与文档

## 许可证

Copyright (c) 2022-2025, MASUTANI Yasuhiro  
All rights reserved.  
This project is licensed under the Apache License 2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献

- https://github.com/rt-net/crane_plus
- https://github.com/AndrejOrsula/pymoveit2