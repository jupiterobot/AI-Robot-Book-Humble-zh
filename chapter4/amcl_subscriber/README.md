# amcl_subscriber
## 概要
第4章的示例程序  
订阅 /amcl_pose 和 /odom 话题，从中提取位姿并在终端中显示的程序


## 安装
第4章的软件包将统一进行安装与构建。
- 请参考 [第4章 安装](https://github.com/AI-Robot-Book-Humble/chapter4)。　


## 实行
将终端2分割为三部分，在上方终端中使用以下命令启动模拟器：
cd ~/airobot_ws
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

在中间的终端中执行以下命令：
cd ~/airobot_ws
source install/setup.bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map/house.yaml

在下方终端中执行以下命令以移动机器人并确认话题的值：
cd ~/airobot_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard


## 帮助
- 目前没有。
 
## 作者
出村 公成


## 历史
- 2022-08-29: 初版


## 许可证
Copyright (c) 2022-2025, Kosei Demura  
All rights reserved. This project is licensed under the Apache License 2.0 license found in the LICENSE file in the root directory of this project.


## 参考文献
- 目前没有。