# 第2章　はじめてのROS2（改訂第2版）

## 概要  
本目录包含《ROS 2与Python实战：AI机器人入门（改订第2版）》（出村、萩原、升谷、タン 著，讲谈社）第2章的示例程序及相关补充资料。

## 目录结构  
- **[airobot_interfaces](airobot_interfaces)**: 为本书定义的自定义接口（程序列表 2.12, 2.15, 2.16）（升谷 保博）  
- **[bringme_action](bringme_action)**: 使用 airobot_interfaces 的 Action 通信示例包（程序列表 2.20, 2.21, 2.22）  
- **[bringme_service](bringme_service)**: 使用 airobot_interfaces 的 Service 通信示例包（程序列表 2.14, 2.15, 2.16）  
- **[happy](happy)**: 第一个 ROS 2 包（程序列表 2.4–2.8）  
- **[happy_action](happy_action)**: Action 通信示例包（程序列表 2.20–2.22）  
- **[happy_action_interfaces](happy_action_interfaces)**: happy_action 所需的自定义 Action 接口定义  
- **[happy_interfaces](happy_interfaces)**: happy 包使用的自定义接口定义  
- **[happy_pub_sub](happy_pub_sub)**: Topic 通信示例包（程序列表 2.13）  
- **[happy_service](happy_service)**: Service 通信示例包  
- **[happy_topic](happy_topic)**: Topic 通信示例包（程序列表 2.9–2.12）  
- **[hello](hello)**: 由 `ros2 pkg create` 自动生成的包（程序列表 2.1–2.3）  
- **[turtlesim_launch](turtlesim_launch)**: turtlesim 的 launch 文件示例  

## 示例程序列表  
- 程序列表 2.1 [package.xml](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/hello/package.xml)  
- 程序列表 2.2 [setup.py](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/hello/setup.py)  
- 程序列表 2.3 [hello_node.py](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/hello/hello/hello_node.py)  
- 程序列表 2.4 [happy_node.py](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/happy/happy/happy_node.py)  
- 程序列表 2.5 [happy_node2.py](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/happy/happy/happy_node2.py)  
- 程序列表 2.6 [setup.py](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/happy/setup.py)  
- 程序列表 2.7 [happy_node3.py](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/happy/happy/happy_node3.py)  
- 程序列表 2.8 [setup.py](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/happy/setup.py)（重复项，内容同 2.6）  
- 程序列表 2.9 [happy_publisher_node.py](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/happy_topic/happy_topic/happy_publisher_node.py)  
- 程序列表 2.10 [happy_subscriber_node.py](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/happy_topic/happy_topic/happy_subscriber_node.py)  
- 程序列表 2.11 [package.xml](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/happy_topic/package.xml)  
- 程序列表 2.12 [setup.py](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/happy_topic/setup.py)  
- 程序列表 2.13 [happy_pub_sub_node.py](https://github.com/AI-Robot-Book-Humble/chapter2/tree/master/happy_pub_sub/happy_pub_sub)  
- 程序列表 2.14 [StringCommand.srv](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/airobot_interfaces/srv/StringCommand.srv)  
- 程序列表 2.15 [bringme_service_node.py](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/bringme_service/bringme_service/bringme_service_node.py)  
- 程序列表 2.16 [bringme_client_node.py](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/bringme_service/bringme_service/bringme_client_node.py)  
- 程序列表 2.17 [CMakeLists.txt](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/airobot_interfaces/CMakeLists.txt)  
- 程序列表 2.18 [package.xml](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/airobot_interfaces/package.xml)  
- 程序列表 2.19 [setup.py](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/bringme_service/setup.py)  
- 程序列表 2.20 [StringCommand.action](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/airobot_interfaces/action/StringCommand.action)  
- 程序列表 2.21 [bringme_action_server_node.py](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/bringme_action/bringme_action/bringme_action_server_node.py)  
- 程序列表 2.22 [bringme_action_client_node.py](https://github.com/AI-Robot-Book-Humble/chapter2/blob/master/bringme_action/bringme_action/bringme_action_client_node.py)  

## 安装  
请按以下步骤安装并构建第2章所有包（假设 ROS 2 工作空间为 `~/airobot_ws`）：
cd ~/airobot_ws/src
git clone https://github.com/AI-Robot-Book-Humble/chapter2
cd ~/airobot_ws
colcon build



## Bug 信息
- 目前未发现已知问题。
