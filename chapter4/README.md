# 第４章　导航（修订第2版）
## 概要
《ROS 2与Python动手学AI机器人入门 修订第2版》（出村・萩原・升谷・坦 著，讲谈社）第４章的示例程序与补充信息等。

## 目录结构
- **[amcl_subscriber](amcl_subscriber)**: 订阅 /amcl 和 /odom 的软件包
- **[happy_lidar](happy_lidar)**: 简单的 LiDAR 软件包及开门示例
- **[happy_move](happy_move)**: 简单的移动控制软件包
- **[happy_teleop](happy_teleop)**: 自制远程操作软件包
- **[map](map)**: 存放地图文件的目录
- **[path_planning](path_planning)**: 路径规划示例程序
- **[waypoint_navi](waypoint_navi)**: 路点导航示例软件包

## 安装
使用以下命令安装第4章的所有软件包。
- 假设 ROS 工作空间为 `~/airobot_ws`。
  cd ~/airobot_ws/src

- 获取 Chapter4 的代码仓库
  git clone https://github.com/AI-Robot-Book-Humble/chapter4

## 补充信息
- 运行导航相关示例程序前，请按照以下说明预先安装地图文件。
  - [https://github.com/AI-Robot-Book-Humble/chapter4/tree/main/map](https://github.com/AI-Robot-Book-Humble/chapter4/tree/main/map)