# bringme_service:  
## 概要
第2章的示例程序  
使用 airobot_interfaces 的示例包

## 安装
Chapter2的包需要全部一起安装和构建。  
- 请参考[第2章 安装](https://github.com/AI-Robot-Book/chapter2)。

## 运行
- 将终端分为两个。
- 在第一个终端中执行以下命令：  
ros2 run bringme_service bringme_service_node  
- 在第二个终端中执行以下命令：
ros2 run bringme_service bringme_client_node
请问您要我拿什么过来：

此时会被询问“请问您要我拿什么过来：”，请输入想要机器人取来的英文单词。  
如果输入 'apple'、'banana' 或 'candy'，将收到回复 **"好的，这是。"**；  
输入其他内容则会收到回复 **"未能找到。"**。

## 帮助
- 目前没有。

## 作者
升谷 保博，出村 公成

## 历史
- 2024-10-15: 初始版本

## 许可证
Copyright (c) 2025, Yasuhiro Masutani and Kosei Demura, All rights reserved. This project is licensed under the Apache-2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献
- 目前没有。