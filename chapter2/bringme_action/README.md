# bringme_action:  
## 概要
第2章的示例程序  
使用 airobot_interfaces 的 Action 通信示例包

## 安装
Chapter2的包需要全部一起安装和构建。
- 请参考[第2章 安装](https://github.com/AI-Robot-Book/chapter2)。

## 运行
- 将终端分为两个。
- 在第一个终端中执行以下命令：  
ros2 run bringme_action bringme_action_server_node  
- 在第二个终端中执行以下命令：
ros2 run bringme_action bringme_action_client_node
请问您要我拿什么过来：

此时会被询问“请问您要我拿什么过来：”，请输入想要机器人取来的英文单词。  
若输入 'apple'、'banana' 或 'candy' 中的任意一个，将显示如下信息：  
[INFO] [1728888853.238853119] [bringme_action_client]: 目标已批准  
[INFO] [1728888857.243842144] [bringme_action_client]: 正在接收反馈：剩余5秒  
[INFO] [1728888858.245358757] [bringme_action_client]: 正在接收反馈：剩余4秒  
[INFO] [1728888859.247032373] [bringme_action_client]: 正在接收反馈：剩余3秒  
[INFO] [1728888860.248677167] [bringme_action_client]: 正在接收 feedback：剩余2秒  
[INFO] [1728888861.250357551] [bringme_action_client]: 正在接收反馈：剩余1秒  
[INFO] [1728888862.252739181] [bringme_action_client]: 目标结果：好的，这是〇〇（apple/banana/candy）。

若输入其他内容，则显示如下信息：  
[INFO] [1728889050.299714176] [bringme_action_client]: 目标已批准  
[INFO] [1728889051.292081616] [bringme_action_client]: 正在接收反馈：剩余5秒  
[INFO] [1728889052.293734543] [bringme_action_client]: 正在接收反馈：剩余4秒  
[INFO] [1728889053.295324400] [bringme_action_client]: 正在接收反馈：剩余3秒  
[INFO] [1728889054.296944174] [bringme_action_client]: 正在接收反馈：剩余2秒  
[INFO] [1728889055.298635793] [bringme_action_client]: 正在接收反馈：剩余1秒  
[INFO] [1728889056.300734920] [bringme_action_client]: 目标结果：未能找到 orange。

## 帮助
- 目前没有。

## 作者
升谷 保博，出村 公成

## 历史
- 2024-10-14: 初始版本

## 许可证
Copyright (c) 2025, Yasuhiro Masutani and Kosei Demura, All rights reserved. This project is licensed under the Apache-2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献
- 目前没有。