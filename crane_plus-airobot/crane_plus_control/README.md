# crane_plus_control

本软件包是基于 [ros2_control](https://github.com/ros-controls/ros2_control) 的 CRANE+ V2 控制器软件包。

## ros2_control 相关文件

- `crane_plus_control::CranePlusHardware`（`crane_plus_hardware`）
  - 本软件包导出的 [Hardware Components](https://ros-controls.github.io/control.ros.org/getting_started.html#hardware-components)
  - 与 CRANE+ V2 实机通信
  - 由 [crane_plus_description/urdf/crane_plus.ros2_control.xacro](../crane_plus_description/urdf/crane_plus.ros2_control.xacro) 加载
- [launch/crane_plus_control.launch.py](./launch/crane_plus_control.launch.py)
  - 启动 [Controller Manager](https://ros-controls.github.io/control.ros.org/getting_started.html#controller-manager) 和控制器的 launch 文件
- [config/crane_plus_controllers.yaml](./config/crane_plus_controllers.yaml)
  - Controller Manager 的参数文件

## 实机的设置

为了让 `crane_plus_hardware` 能与 CRANE+ V2 实机通信，需要对 PC 和 CRANE+ V2 进行设置。

如果设置不正确，CRANE+ V2 可能无法动作、发生振动，或出现其他不稳定的行为，请务必注意。

### USB 通信端口的设置

`crane_plus_hardware` 通过 USB 通信端口（`/dev/ttyUSB*`）与 CRANE+ V2 通信。

使用以下命令更改访问权限：

# 使用 /dev/ttyUSB0 的情况
$ sudo chmod 666 /dev/ttyUSB0

### latency_timer 的设置

为了以 100 Hz 周期控制 CRANE+ V2，需要更改 USB 通信端口和伺服电机的设置。

执行以下命令更改 USB 通信端口的 `latency_timer`。

参考资料：https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#usb-latency-setting

# 使用 /dev/ttyUSB0 的情况

# 切换到 root
$ sudo su

# echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
1
# exit

### Return Delay Time 的设置

CRANE+ V2 搭载的伺服电机 [Dynamixel AX-12A](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/)  
具有名为 `Return Delay Time` 的参数。

默认值为 250，表示伺服电机从接收到 `Instruction Packet` 到发送 `Status Packet` 之间有 500 微秒的延迟。

使用 [Dynamixel Wizard 2](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)  
将 `Return Delay Time` 设置得更小，可以加快控制周期。

![Setting Return Delay Time](https://rt-net.github.io/images/crane-plus/setting_return_delay_time.png)

## 节点的启动

执行 `crane_plus_control.launch.py` 后，将启动 `Controller Manager` 节点，并加载以下控制器：

- crane_plus_joint_state_broadcaster (`joint_state_broadcaster/JointStateBroadcaster`)
- crane_plus_arm_controller (`joint_trajectory_controller/JointTrajectoryController`)
- crane_plus_gripper_controller (`joint_trajectory_controller/JointTrajectoryController`)

节点启动后，可通过以下命令显示关节角度信息（`joint_states`）：

$ ros2 topic echo /joint_states

## Controller Manager 的参数

`Controller Manager` 的参数在 [config/crane_plus_controllers.yaml](./config/crane_plus_controllers.yaml) 中设置。

yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    crane_plus_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    crane_plus_gripper_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

### 控制周期

`update_rate` 设置控制周期。

由于 CRANE+ V2 所用伺服电机的规格限制，无法以超过 100 Hz 的周期进行控制。

### 控制器

为 CRANE+ V2 的机械臂设置了 `crane_plus_arm_controller`，  
为夹爪设置了 `crane_plus_gripper_controller`。

## crane_plus_hardware 的参数

`crane_plus_hardware` 的参数在  
[crane_plus_description/urdf/crane_plus.urdf.xacro](../crane_plus_description/urdf/crane_plus.urdf.xacro) 中设置。

xml
<xacro:arg name="use_gazebo" default="false" />
<xacro:arg name="port_name" default="/dev/ttyUSB0" />
<xacro:arg name="baudrate" default="1000000" />
<xacro:arg name="timeout_seconds" default="5.0" />
<xacro:arg name="read_velocities" default="0" />
<xacro:arg name="read_loads" default="0" />
<xacro:arg name="read_voltages" default="0" />
<xacro:arg name="read_temperatures" default="0" />

### USB 通信端口

`port_name` 设置用于与 CRANE+ V2 通信的 USB 通信端口。

### 波特率

`baudrate` 设置与 CRANE+ V2 搭载的 Dynamixel 通信的波特率。

默认值设为 Dynamixel AX-12A 的最高波特率 `1000000` (1 Mbps)。

### 通信超时

`timeout_seconds` 设置通信超时时间（秒）。

如果 `crane_plus_hardware` 在一定时间（默认 5 秒）内持续通信失败，  
将停止 read/write 操作。在 USB 线缆或电源线脱落等情况下有效。

### 伺服参数

`read_velocities`、`read_loads`、`read_voltages`、`read_temperatures`  
是用于读取伺服电机转速、电压、负载和温度的参数。

设置为 `1` 时，将读取对应的伺服参数。

读取这些参数会增加通信数据量，导致控制周期低于 100 Hz。

读取到的参数将作为 `dynamic_joint_states` 话题发布。

$ ros2 topic echo /dynamic_joint_states

---

[back to top](#crane_plus_control)