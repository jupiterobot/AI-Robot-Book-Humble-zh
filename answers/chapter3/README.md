# 第3章的解答示例

- 将本文件夹中的``` .py ```文件复制到``` ~/airobot_ws/src/chapter3/speech_action/speech_action ```目录下。
- 在 ```~/airobot_ws/src/chapter3/speech_action/setup.py``` 文件的 entry_points 配置列表中添加以下行：
```
            'challenge_3_1 = speech_action.challenge_3_1:main',
```

- 在终端中执行以下命令：`````````
```
cd ~/airobot_ws
source install/setup.bash
```

- 首先，打开两个终端，分别启动语音识别服务器和语音合成服务器：
```
ros2 run speech_action speech_recognition_server
ros2 run speech_action speech_synthesis_server
```

- 然后，在另一个终端中运行以下命令以启动 challenge_3_1：
```
ros2 run speech_action challenge_3_1
```

