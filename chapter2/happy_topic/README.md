# happy_topic

## 概要  
第2章的示例程序，演示 ROS 2 中的 **Topic 通信**。  
- `happy_topic/happy_publisher_node.py`（程序列表 2.7）：发布消息的节点  
- `happy_topic/happy_subscriber_node.py`（程序列表 2.8）：订阅消息的节点  

## 安装  
本包属于第2章整体代码的一部分，请统一安装和构建所有包：  
- 参见 [第2章 安装说明](https://github.com/AI-Robot-Book/chapter2)

## 运行  
在终端中执行以下命令启动发布者节点：
cd ~/airobot_ws
source install/setup.bash
ros2 run happy_topic happy_publisher_node

> 若要运行订阅者节点，请使用：
ros2 run happy_topic happy_subscriber_node

## Bug 信息  
- 目前无已知问题。

## 帮助  
- 暂无帮助信息。

## 作者  
出村 公成

## 历史  
- 2024-03-27: 在 ROS 2 Humble 环境下完成运行验证  
- 2022-11-08: 修正 `happy_subscriber_node.py` 中的节点类名错误（原误写为 `HappyPublisher`），同步修改第7行和第20行  
- 2022-08-29: 初始版本

## 许可证  
Copyright (c) 2022–2025, Kosei Demura  
All rights reserved.  
本项目采用 Apache License 2.0 授权，完整许可证文本位于项目根目录的 `LICENSE` 文件中。

## 参考文献  
- 暂无