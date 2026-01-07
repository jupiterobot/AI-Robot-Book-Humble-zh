# 动作的实用程序示例

## 概要

- 本书中统一使用 [StringCommand 动作类型](https://github.com/AI-Robot-Book-Humble/chapter2/blob/main/airobot_interfaces/action/StringCommand.action)。
- 提供一个动作服务器示例，可在处理当前目标（goal）的过程中接受取消请求或新的目标。
- 提供一个动作客户端示例，可在目标执行过程中发送取消请求或发送新目标。
- 在 Ubuntu 22.04 和 ROS Humble 环境下开发并验证。

## 安装

- 假设 ROS 工作空间为 `～/airobot_ws`：
  cd ～/airobot_ws/src

- 获取包含本包的仓库：
  cd ～/airobot_ws/src
  git clone https://github.com/AI-Robot-Book-Humble/appendixB

- 获取定义动作接口的包所在的仓库：
  git clone https://github.com/AI-Robot-Book-Humble/chapter2

- 构建包：
  cd ～/airobot_ws
  colcon build
  source install/setup.bash

## 运行

- 终端1（动作服务器）：
  cd ～/airobot_ws
  source install/setup.bash
  ros2 run airobot_action new_bringme_action_server_node

- 终端2（动作客户端）：
  cd ～/airobot_ws
  source install/setup.bash
  ros2 run airobot_action test_client

## 帮助

（暂无内容）

## 作者

升谷 保博

## 历史

- 2024-10-15: 公开发布

## 许可证

Copyright (c) 2025 MASUTANI Yasuhiro  
All rights reserved.  
This project is licensed under the Apache License 2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献

- [rclpy Actions](https://docs.ros2.org/foxy/api/rclpy/api/actions.html)
- [Understanding actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- [Writing an action server and client (Python)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
  - 未说明如何处理取消或在执行中接收新目标。
- [ROS1 的 SimpleActionServer 在 ROS2 中的改写方法](https://qiita.com/nasu_onigiri/items/783d7ee77556528e5a52)
- [minimal_action_server 包](https://github.com/ros2/examples/tree/humble/rclpy/actions/minimal_action_server)
  - 仅处理单个目标的服务器 [server_single_goal](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_single_goal.py)
  - 将目标加入队列的服务器 [server_queue_goals](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_queue_goals.py)
  - 并行处理多个目标的服务器 [server](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server.py)
  - 延迟并行处理多个目标的服务器 [server_defer](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_defer.py)
  - 不使用节点类的服务器 [server_not_composable](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_not_composable.py)
- [minimal_action_client 包](https://github.com/ros2/examples/tree/humble/rclpy/actions/minimal_action_client)
  - 最简客户端 [client](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client.py)
  - 发送目标后立即发送取消请求的客户端 [client_cancel](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client_cancel.py)
  - 异步并行处理示例 [client_asyncio](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client_asyncio.py)
  - 不使用节点类的客户端 [client_not_composable](https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client_not_composable.py)