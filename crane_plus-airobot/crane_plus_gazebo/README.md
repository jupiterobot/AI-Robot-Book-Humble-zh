# crane_plus_gazebo

本软件包用于在 [Gazebo](https://gazebosim.org/home) 中对 CRANE+ V2 进行仿真。

## 启动节点

运行以下命令将启动 Gazebo，并加载包含 CRANE+ V2 机器人、桌子（Table）和方块（Box）的仿真场景：

ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py

> **注意**：首次启动时，系统会从网络下载 Table 和 Box 的模型文件，因此模型显示可能需要较长时间，请耐心等待。

此仿真环境完全独立运行，**无需连接实体机器人**，也**无需事先运行** `crane_plus_examples/launch/demo.launch.py`。

![crane_plus_ignition](https://rt-net.github.io/images/crane-plus/crane_plus_ignition.png)