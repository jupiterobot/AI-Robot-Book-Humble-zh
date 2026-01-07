# TurtleBot3 Happy Mini  
## 概要  
基于以下 turtlebot3 官方仓库进行复制，并新增了 Happy Mini 的模型（URDF 和 Mesh）。移动底盘的参数与 waffle_pi 相同。  
- [ROBOTIS-GIT turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3.git)

## 新增与修改内容  
- turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_happy_mini  
- turtlebot3_simulations/turtlebot3_gazebo/urdf/turtlebot3_happy_mini.urdf  
- turtlebot3_simulations/turtlebot3_gazebo/launch/spawn2_turtlebot3.launch.py  
- turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_house2.launch.py  

## 环境  
- ROS 2 Humble  

## 安装  
- 安装 Gazebo 与 ROS 2 集成所需的包：
  source ~/.bashrc
  sudo apt install ros-humble-gazebo-*
  sudo apt install ros-humble-gazebo-ros-pkgs

- 安装 Happy Mini 相关包：
  cd ~/airobot_ws/src
  git clone https://github.com/AI-Robot-Book-Humble/turtlebot3_happy_mini
  cd ~/colcon_ws
  colcon build

## 运行  
1. Empty World  
![happy mini empty world](https://github.com/demulab/happy_mini_turtlebot3_sim/blob/main/happy_mini_empty_world.png "happy mini empty world")
  export TURTLEBOT3_MODEL=happy_mini
  ros2 launch turtlebot3_gazebo empty_world.launch.py

2. TurtleBot3 World  
![happy mini turtlebot3 world](https://github.com/demulab/happy_mini_turtlebot3_sim/blob/main/happy_mini_turtlebot3_world.png "happy mini turtlebot3 world")
  export TURTLEBOT3_MODEL=happy_mini
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

3. TurtleBot3 House  
![happy mini turtlebot3 house](https://github.com/demulab/happy_mini_turtlebot3_sim/blob/main/happy_mini_house.png "happy mini turtlebot3 house")
  export TURTLEBOT3_MODEL=happy_mini
  ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

4. 切换机器人模型  
- 使用 Waffle Pi 模型时：
  export TURTLEBOT3_MODEL=waffle_pi

5. 修改机器人初始位置  
  ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py x_pose:=初始位置的x坐标 y_pose:=初始位置的y坐标

6. 修改机器人初始朝向  
请使用改进后的 `turtlebot3_house2.launch.py`，它支持设置初始 Yaw 角度：
  ros2 launch turtlebot3_gazebo turtlebot3_house2.launch.py x_pose:=初始位置的x坐标 y_pose:=初始位置的y坐标 yaw_pose:=初始朝向的Yaw角

## 历史  
- 2024-10-13: 初始版本  

## 许可证  
Apache License 2.0，完整许可证文本位于项目根目录下的 LICENSE 文件中。

## 参考文献  
- 暂无