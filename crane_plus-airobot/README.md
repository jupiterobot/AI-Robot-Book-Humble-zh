# crane_plus for AI Robot Book（讲谈社，日文版）

本书使用的末端执行器（endtip）模型已做修改。

- 带末端执行器的模型可视化：
  ros2 launch crane_plus_description endtip_display.launch.py

- 实体机器人操作：
  - 不使用 MoveIt：
    ros2 launch crane_plus_examples no_moveit_demo.launch.py
  - 使用带末端执行器的配置：
    ros2 launch crane_plus_examples endtip_demo.launch.py

- 仿真环境（Gazebo）：
  - 不使用 MoveIt：
    ros2 launch crane_plus_gazebo no_moveit_crane_plus_with_table.launch.py
  - 使用带末端执行器的配置：
    ros2 launch crane_plus_gazebo endtip_crane_plus_with_table.launch.py

以下为原始 README 内容。

----------------------------------

# crane_plus

[![industrial_ci](https://github.com/rt-net/crane_plus/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/crane_plus/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

CRANE+ V2 的 ROS 2 软件包套件。

<img src=https://www.rt-shop.jp/images/RT/CRANEplusV2.png width=400px/><img src=https://rt-net.github.io/images/crane-plus/pick_and_place.gif width=400px />

## 目录

- 支持的 ROS 2 发行版
- 系统要求
- 安装方法
  - 二进制安装
  - 从源码构建
- 快速开始
- 软件包说明
- 许可证
- 免责声明

## 支持的 ROS 2 发行版

- Foxy
- Humble

## 系统要求

- CRANE+ V2
  - 产品介绍：https://rt-net.jp/products/cranev2/
  - 购买链接：https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_1&products_id=3626&language=ja
- Linux 操作系统
  - Ubuntu 22.04
- ROS
  - Humble Hawksbill：https://docs.ros.org/en/humble/Installation.html

## 安装方法

### Docker 镜像

我们提供了包含预编译软件包的 Docker 镜像。  
详情请参阅 .docker/README.md。

### 二进制安装

TBD

### 从源码构建

# 设置 ROS 环境
$ source /opt/ros/humble/setup.bash

# 下载 crane_plus 仓库
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/rt-net/crane_plus.git

# 安装依赖项
$ rosdep install -r -y -i --from-paths .

# 编译并安装
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source ~/ros2_ws/install/setup.bash

## 快速开始

# 将 CRANE+ V2 连接到电脑后执行
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch crane_plus_examples demo.launch.py port_name:=/dev/ttyUSB0

# 在另一个终端中
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch crane_plus_examples example.launch.py example:='gripper_control'

# 按 [Ctrl-c] 终止程序。

更多详情请参阅 crane_plus_examples/README.md。

## 软件包说明

- crane_plus_control
  - README：./crane_plus_control/README.md
  - 用于控制 CRANE+ V2 的软件包
  - USB 通信端口的设置方法详见 README
- crane_plus_description
  - README：./crane_plus_description/README.md
  - 定义 CRANE+ V2 机器人模型（xacro 格式）的软件包
- crane_plus_examples
  - README：./crane_plus_examples/README.md
  - CRANE+ V2 的示例代码集合
- crane_plus_gazebo
  - README：./crane_plus_gazebo/README.md
  - CRANE+ V2 的 Gazebo 仿真软件包
- crane_plus_moveit_config
  - README：./crane_plus_moveit_config/README.md
  - CRANE+ V2 的 moveit2 配置文件

## 许可证

本仓库根据 Apache 2.0 许可证发布。  
详细条款请参阅 LICENSE 文件。

我们已获得 ROBOTIS 公司授权，可使用其 AX-12A 伺服电机相关的 CAD 模型。  
CRANE+ V2 中所使用的 ROBOTIS 部件相关的版权、商标及其他知识产权均归 ROBOTIS 公司所有。

We have obtained permission from ROBOTIS Co., Ltd. to use CAD models relating to servo motors AX-12A. The proprietary rights relating to any components or parts manufactured by ROBOTIS and used in this product, including but not limited to copyrights, trademarks, and other intellectual property rights, shall remain vested in ROBOTIS.

## 免责声明

本软件按“现状”（AS IS）提供，遵循 Apache 2.0 许可证，不提供任何形式的免费技术支持。

在使用本产品及本软件过程中所引发的任何损害，株式会社阿鲁蒂（RT Corporation）概不负责。若用户自行编写的程序未设置适当的安全限制，可能导致设备损坏，或机械臂与周围环境、操作人员发生接触甚至碰撞，从而引发严重事故。请用户务必在充分确保安全的前提下，自行承担全部责任进行使用。