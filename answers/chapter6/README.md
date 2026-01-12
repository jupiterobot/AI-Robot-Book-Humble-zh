# 第6章のチャレンジの解答例
# 第6章挑战的解答示例

- 将本文件夹中的 .py 文件复制到 ~/airobot_ws/src/chapter6/crane_plus_commander/crane_plus_commander 目录下。
- 在 ~/airobot_ws/src/chapter6/crane_plus_commander/setup.py 文件的 entry_points 配置列表中，添加以下行：
  ```
            'challenge6_1 = crane_plus_commander.challenge6_1:main',
            'challenge6_2 = crane_plus_commander.challenge6_2:main',
            'challenge6_4 = crane_plus_commander.challenge6_4:main',
            'challenge6_5 = crane_plus_commander.challenge6_5:main',
            'challenge6_6 = crane_plus_commander.challenge6_6:main',
            'challenge6_7 = crane_plus_commander.challenge6_7:main',
            'challenge6_8 = crane_plus_commander.challenge6_8:main',
  ```
- 在终端中执行以下命令：
  ```
  cd ~/airobot_ws
  colcon build --packages-select crane_plus_commander
  source install/setup.bash
  ```
- 在启动 CRANE+ 的实体机器人或仿真器相关节点之后，在终端中运行以下命令（X = 1,2,4,5,6,7,8）：
  ```
  ros2 run crane_plus_commander challenge6_X
  ```