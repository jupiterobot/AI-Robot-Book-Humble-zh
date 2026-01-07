# 第6章 操作（修订第2版）

## 概要

《ROS 2与Python动手学AI机器人入门 修订第2版》（出村・萩原・升谷・坦 著，讲谈社）第6章的示例程序与补充信息等。

## 目录结构

- [crane_plus_commander](crane_plus_commander)： 使用 CRANE+ V2 专用 ROS 2 节点的简易控制节点

- [simple_arm/simple_arm_description](simple_arm/simple_arm_description)： 简易的2自由度机器人臂模型

## 示例程序列表
- 程序清单6.1 [kinematics.py 的一部分](crane_plus_commander/crane_plus_commander/kinematics.py#L59-L68)
- 程序清单6.2 [kinematics.py 的一部分](crane_plus_commander/crane_plus_commander/kinematics.py#L71-L95)
- 程序清单6.3 [simple_arm.urdf](simple_arm/simple_arm_description/urdf/simple_arm.urdf)
- 6.5.5 控制关节运动的程序 [commander1.py](crane_plus_commander/crane_plus_commander/commander1.py)
- 6.5.6 控制末端执行器运动的程序 [commander2.py](crane_plus_commander/crane_plus_commander/commander2.py)
- 6.5.7 接收机器人状态的程序 [commander3.py](crane_plus_commander/crane_plus_commander/commander3.py)
- 6.5.8 使用 ROS 2 动作通信的程序 [commander4.py](crane_plus_commander/crane_plus_commander/commander4.py)
- 6.6.3 使用 tf 的程序 [commander5.py](crane_plus_commander/crane_plus_commander/commander5.py)
- 6.7.4 使用 MoveIt 进行运动学计算的程序 [commander2_moveit.py](crane_plus_commander/crane_plus_commander/commander2_moveit.py)
- 6.7.5 使用 MoveIt 控制末端移动的程序 [commander5_moveit.py](crane_plus_commander/crane_plus_commander/commander5_moveit.py)
- 6.8 接收其他节点指令并执行动作的程序 [commander6_moveit.py](crane_plus_commander/crane_plus_commander/commander6_moveit.py)

## 补充信息

- 已添加 [使用 CRANE+ V2 实机的准备工作](crane_plus_commander#准备) 说明。（2022/12/7）