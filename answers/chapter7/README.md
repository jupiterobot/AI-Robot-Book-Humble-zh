## 第7章的解答示例

本包提供了第7章中所列挑战的解答。

> [!IMPORTANT]
> 不支持 bringme_sm_advanced_smach 这个包。
> 关于该包的源代码问题，请在 AI Robot Book 各章解答示例 (https://github.com/AI-Robot-Book/answers/tree/master/chapter7) 仓库中提交 Issue。

### 挑战 7.1

挑战 7.1 要求我们将 Voice 状态原本连接的语音伪节点，替换为第3章中创建的真实语音识别节点，并验证状态机是否能正常工作。

1. 首先，请将原始状态机 bringme_action_behavior_sm.py 修改为 challenge_7_1_bringme_action_behavior_sm.py。

   在此文件中，我们将第95行中指定的语音识别节点名称修改为 /speech_recognition。

修改前：第95行
VoiceActionState(timeout=timeout, action_topic="ps_voice/command"),

修改后：第95行
VoiceActionState(timeout=timeout, action_topic="/speech_recognition/command"),

2. 接下来，请将原始的语音伪节点文件 voice_action_state.py 修改为 challenge_7_1_voice_action_state.py。

   此处我们不进行复杂的自然语言处理，仅检查从语音识别结果中是否包含目标物体 cup 和目标地点 kitchen。
   如果语音内容中缺少其中任意一个关键词，则状态返回失败，并重新听取语音指令。

修改前：第99行 - 第104行
# 音声認識の結果を処理する必要があります
userdata.target      = 'cup'
userdata.destination = 'kitchen'

self._return = 'done'
return self._return # 'done'という結果を返します

修改后：第99行 - 第111行
# 音声認識の結果を処理する必要があります
# 今回は，カップとキッチンの単語が含まれているかを確認します
userdata.target      = 'cup' if 'cup' in userdata.text else ''
userdata.destination = 'kitchen' if 'kitchen' in userdata.text else ''
# Challenge3.1にご参照ください．
# userdata.target, userdata.destination = search_object_and_place(userdata.text)

if len(userdata.target) > 0 and len(userdata.destination) > 0:    
    self._return = 'done' # 'done'という結果を設定します
else:        
    self._return = 'failed' # 'failed'という結果を設定します

return self._return # 'done'または'failed'という結果を返します

> [!NOTE]
> 若希望使用更智能的自然语言处理功能，可调用 search_object_and_place() 函数，详情请参考 challenge_3_1.py。

3. 此外，请将用于启动原始伪节点的启动文件 bringme_nodes.launch.py 修改为 challenge_7_1_bringme_nodes.launch.py。

   此处我们将原本启动的 voice_node 伪节点，替换为第3章中的真实语音识别服务器 speech_recognition_server.py。

修改前：第22行 - 第25行
Node(
    package='pseudo_node_action',
    executable='voice_node',
)

修改后：第22行 - 第25行
Node(
    package='speech_action',
    executable='speech_recognition_server',
)

4. 修改完程序后，请编译并加载该包：
cd ~/ai_robot_ws
colcon build
source install/setup.bash

5. 接下来，启动语音识别节点及其他伪节点：
ros2 launch pseudo_node_action bringme_nodes.launch.py

6. 在另一个终端中启动 FlexBe WebUI，并在 Load Behavior 中选择 Bringme Action Behavior 状态机：
ros2 launch flexbe_webui flexbe_full.launch.py

7. 在 FlexBe WebUI 的 State Machine Editor 中，确认 Voice 状态的参数，确保 action_topic 字段已设置为 "/speech_recognition/command"。

8. 最后，切换到 FlexBe WebUI 的 Runtime Control 页面，设置 listen_time（语音监听时长），然后点击 Start Execution 开始执行。

> [!NOTE]
> 默认的语音监听时间为 10 秒。

## 目录结构

- bringme_sm_advanced_flexbe: Bring me 任务的状态机挑战程序（FlexBE 版）
- (存档) bringme_sm_advanced_smach: Bring me 任务的状态机挑战程序（Smach 版）