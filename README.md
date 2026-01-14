# 中文版《ROS 2与Python动手学AI机器人入门（修订第2版）》配套示例程序

本仓库为日文原版书籍《ROS 2とPythonで作って学ぶAIロボット入門 改訂第2版》（出村・萩原・升谷・タン 著，讲谈社）的中文翻译版提供配套 ROS 2 示例代码。

## 原书信息

- 日文原版 GitHub：https://github.com/AI-Robot-Book-Humble  
- 中文翻译版：由嘉兴市木星机器人翻译整理（非官方出版物），旨在帮助中文读者更轻松地学习 ROS 2 与 AI 机器人开发。
- 适用 ROS 2 版本：Humble Hawksbill（Ubuntu 22.04）

## 为什么需要这个仓库？

原书配套代码以日文注释和环境配置为主。本仓库提供：

- 完整中文注释的示例程序  
- 与原书章节一一对应的代码结构（第1章~第7章 + 附录）

## 快速开始

1. 创建或进入你的 ROS 2 工作空间（例如 `airobot_ws`）：
   mkdir -p ~/airobot_ws/src
   cd ~/airobot_ws/src

2. 在 `src` 目录下克隆本仓库：

gitee(国内推荐):
git clone https://gitee.com/jupiterobot/AI-Robot-Book-Humble-zh.git

github:
git clone https://github.com/jupiterobot/AI-Robot-Book-Humble-zh.git

3. 返回工作空间根目录并编译：
   cd ~/airobot_ws
   colcon build --symlink-install

## Docker镜像相关文件

- Docker 镜像及相关环境支持请访问：

gitee(国内推荐):
https://gitee.com/jupiterobot/docker-ros2-desktop-ai-robot-book-humble-zh

github:
https://github.com/jupiterobot/docker-ros2-desktop-ai-robot-book-humble-zh.git

## 仓库结构

ai-robot-book-humble-zh/
├── chapter2/       # 第2章：ROS 2 入门
├── chapter3/       # 第3章：语音识别与合成（中文适配）
├── chapter4/       # 第4章：导航
├── chapter5/       # 第5章：计算机视觉
├── chapter6/       # 第6章：机械臂操作
├── chapter7/       # 第7章：任务规划
├── appendixB/    # 附录：Launch 文件、Action、运动学等
├── appendixE/    # 附录：Launch 文件、Action、运动学等
└── README.md

所有代码均基于原书日文版修改，关键部分添加了中文注释，并适配中文语音/文本处理。

## 相关资源

- 原书官网（日文）：https://www.kspub.co.jp/book/detail/5386163.html
- 原书 GitHub（日文）：https://github.com/AI-Robot-Book-Humble
