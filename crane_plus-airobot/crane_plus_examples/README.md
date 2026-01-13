# crane_plus_examples

本软件包是 CRANE+ V2 ROS 2 软件包的示例代码集合。

## 准备工作（使用实体机器人时）

![crane_plus](https://rt-net.github.io/images/crane-plus/CRANEV2-500x500.png)

### 1. 将 CRANE+ V2 本体连接到 PC

请将 CRANE+ V2 本体通过 USB 连接到您的计算机。  
具体连接方式请参考产品手册。

**※ 请确保周围有足够空间，防止 CRANE+ V2 在运行过程中与周围物体发生碰撞。**

### 2. 确认 USB 通信端口设置

USB 通信端口的配置方法请参考 `crane_plus_control` 包的  
[README](../crane_plus_control/README.md)。

**⚠️ 若未正确配置，可能导致 CRANE+ V2 无法动作、异常振动或出现不稳定行为，请务必注意。**

### 3. 启动 move_group 与控制器

#### 使用标准 CRANE+ V2 时

运行以下命令启动 MoveIt 的 `move_group`（来自 `crane_plus_moveit_config`）  
和底层控制器（来自 `crane_plus_control`）：

ros2 launch crane_plus_examples demo.launch.py port_name:=/dev/ttyUSB0

#### 使用带网络摄像头的型号时

若使用的是带摄像头的 CRANE+ V2 型号，请运行以下命令。  
其中 `video_device` 参数用于指定所用摄像头设备（如 `/dev/video0`）：

ros2 launch crane_plus_examples demo.launch.py port_name:=/dev/ttyUSB0 use_camera:=true video_device:=/dev/video0

## 准备工作（使用 Gazebo 仿真时）
=======
![crane_plus_ignition](https://rt-net.github.io/images/crane-plus/crane_plus_ignition.png)

### 1. 启动 move_group 与 Gazebo

运行以下命令以同时启动 MoveIt (`crane_plus_moveit_config`) 和 Gazebo 仿真环境：

ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py

## 运行示例程序

完成上述准备后，即可运行各类示例程序。  
例如，控制夹爪开合的示例如下：

ros2 launch crane_plus_examples example.launch.py example:='gripper_control'

按 `Ctrl+c` 可终止程序。

## 在 Gazebo 中运行示例程序

在 Gazebo 中运行示例时，需添加 `use_sim_time:=true` 参数：

ros2 launch crane_plus_examples example.launch.py example:='gripper_control' use_sim_time:=true

## 示例列表

在 `demo.launch.py` 已启动的前提下，可运行以下任一示例：

- gripper_control
- pose_groupstate
- joint_values
- pick_and_place

您可通过以下命令查看所有可用示例：

ros2 launch crane_plus_examples example.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

    'example':
        Set an example executable name: [gripper_control, pose_groupstate, joint_values, pick_and_place]
        (default: 'gripper_control')

---

### gripper_control

控制夹爪开合的示例。

执行命令：
ros2 launch crane_plus_examples example.launch.py example:='gripper_control'

<img src=https://rt-net.github.io/images/crane-plus/gripper_control.gif width=500px />

[返回示例列表](#examples)

---

### pose_groupstate

使用 SRDF 中定义的 `group_state`（预设姿态）的示例。  
机器人将依次移动至 `home` 和 `vertical` 姿态，  
这些姿态定义在文件：  
`crane_plus_moveit_config/config/crane_plus.srdf`

执行命令：
ros2 launch crane_plus_examples example.launch.py example:='pose_groupstate'

<img src=https://rt-net.github.io/images/crane-plus/pose_groupstate.gif width=500px />

[返回示例列表](#examples)

---

### joint_values

逐个设置各关节角度的示例。

执行命令：
ros2 launch crane_plus_examples example.launch.py example:='joint_values'

<img src=https://rt-net.github.io/images/crane-plus/joint_values.gif width=500px />

[返回示例列表](#examples)

---

### pick_and_place

完整的“抓取–提起–搬运–放置”操作示例。

执行命令：
ros2 launch crane_plus_examples example.launch.py example:='pick_and_place'

<img src=https://rt-net.github.io/images/crane-plus/pick_and_place.gif width=500px />

[返回示例列表](#examples)

---

## 摄像头示例（Camera Examples）

适用于带摄像头型号的视觉功能示例。

请先按照 [“使用带网络摄像头的型号”](#使用带网络摄像头的型号时) 的步骤  
启动 `demo.launch.py`，然后运行以下任一视觉示例：

- aruco_detection
- color_detection

查看可用摄像头示例列表：

ros2 launch crane_plus_examples camera_example.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

    'example':
        Set an example executable name: [color_detection]
        (default: 'color_detection')

---

### aruco_detection

通过摄像头检测贴在物体上的 ArUco 标记，并据此抓取目标的示例。  
标记图案见 [aruco_markers.pdf](./aruco_markers.pdf)，建议打印在 A4 纸上，  
并粘贴到边长为 50mm 的立方体上使用。

检测到的标记位姿会以 TF 坐标系形式发布，  
其 `frame_id` 格式为 `target_<ID>`（例如 ID 为 0 的标记对应 `target_0`）。  
当前抓取目标固定为 `target_0`。  
标记检测基于 [OpenCV ArUco 模块](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)。

执行命令：
ros2 launch crane_plus_examples camera_example.launch.py example:='aruco_detection'

#### 演示视频
[![crane_plus_aruco_detection_demo](https://rt-net.github.io/images/crane-plus/aruco_detection.gif)](https://youtu.be/m9dus6LCocc)

[返回示例列表](#examples)

---

### color_detection

检测特定颜色物体并进行抓取的示例。

默认检测红色物体，其位置以 TF 坐标系 `target_0` 发布。  
颜色检测基于 [OpenCV 阈值分割](https://docs.opencv.org/4.x/db/d8e/tutorial_threshold.html) 实现。

执行命令：
ros2 launch crane_plus_examples camera_example.launch.py example:='color_detection'

#### 演示视频
[![crane_plus_color_detection_demo](https://rt-net.github.io/images/crane-plus/color_detection.gif)](https://youtu.be/Kn0eWA7sALY)

[返回示例列表](#examples)