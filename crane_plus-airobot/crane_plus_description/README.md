# crane_plus_description

本软件包包含 CRANE+ V2 机器人的模型数据（xacro 格式）。

## 显示机器人模型

运行以下命令可启动 `robot_state_publisher`、`joint_state_publisher` 和 `rviz2`，  
在 RViz 中可视化 CRANE+ V2 的 3D 模型，便于调试 xacro 文件：

ros2 launch crane_plus_description display.launch.py

若使用的是带网络摄像头的模型，请运行以下命令：

ros2 launch crane_plus_description display.launch.py use_camera:=true

![display.launch.py](https://rt-net.github.io/images/crane-plus/display_launch.png)

## 配置伺服电机角度限制

当您使用 CRANE+ V2 实体机器人时，  
请务必提前设置各伺服电机内部的角度限制（`CW Angle Limit` 和 `CCW Angle Limit`）。

CRANE+ V2 使用的伺服电机为 ROBOTIS 公司的 AX-12A，  
可通过 [DYNAMIXEL Wizard 2](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) 工具进行配置。

![dynamixel wizard2](https://rt-net.github.io/images/crane-plus/dynamixel_wizard2.png)

在 [crane_plus.urdf.xacro](./urdf/crane_plus.urdf.xacro) 文件中，  
各关节的角度限制已按如下方式定义。  
建议您将伺服电机的实际角度限制设置为与以下值尽可能接近：

<xacro:property name="M_PI" value="3.14159"/>
<xacro:property name="TO_RADIAN" value="${M_PI / 180.0}"/>
<xacro:property name="SERVO_HOME" value="${TO_RADIAN * 150.0}"/>
<xacro:property name="JOINT_VELOCITY_LIMIT" value="2.0"/>
<xacro:property name="JOINT_1_LOWER_LIMIT" value="${0.0 * TO_RADIAN - SERVO_HOME}"/>
<xacro:property name="JOINT_1_UPPER_LIMIT" value="${300.0 * TO_RADIAN - SERVO_HOME}"/>
<xacro:property name="JOINT_2_LOWER_LIMIT" value="${45.45 * TO_RADIAN - SERVO_HOME}"/>
<xacro:property name="JOINT_2_UPPER_LIMIT" value="${252.20 * TO_RADIAN - SERVO_HOME}"/>
<xacro:property name="JOINT_3_LOWER_LIMIT" value="${3.52 * TO_RADIAN - SERVO_HOME}"/>
<xacro:property name="JOINT_3_UPPER_LIMIT" value="${290.62 * TO_RADIAN - SERVO_HOME}"/>
<xacro:property name="JOINT_4_LOWER_LIMIT" value="${44.57 * TO_RADIAN - SERVO_HOME}"/>
<xacro:property name="JOINT_4_UPPER_LIMIT" value="${251.32 * TO_RADIAN - SERVO_HOME}"/>
<xacro:property name="JOINT_HAND_LOWER_LIMIT" value="${109.38 * TO_RADIAN - SERVO_HOME}"/>
<xacro:property name="JOINT_HAND_UPPER_LIMIT" value="${188.27 * TO_RADIAN - SERVO_HOME}"/>