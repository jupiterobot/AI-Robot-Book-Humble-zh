#!/usr/bin/env python

# Copyright 2024 Keith Valentin
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""语音动作 FlexBE 状态"""

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from airobot_interfaces.action import StringCommand


class VoiceActionState(EventState):
    """
    通过 Action 通信启动语音识别，并将识别结果存入 userdata.text。

    启动方式:
        要运行此状态，请先启动对应的 Action 服务器：
        $ ros2 run pseudo_node_action voice_node

        可通过以下命令查看当前可用的 Action 列表：
        $ ros2 action list

    参数
    -- timeout             最大允许等待时间（秒）
    -- action_topic        语音识别 Action 的话题名称

    输出结果（Outcomes）
    <= done                语音识别成功完成
    <= failed              因任何原因失败（如识别失败、关键词缺失等）
    <= canceled            用户请求取消
    <= timeout             超过最大允许等待时间

    Userdata 接口
    ># time        string  语音识别的持续时间（秒），以字符串形式传入（输入）
    #> text        string  语音识别的原始结果文本（输出）
    #> target      string  从语音中提取的目标物体（如 'cup'）（输出）
    #> destination string  从语音中提取的目标地点（如 'kitchen'）（输出）
    """

    def __init__(self, timeout, action_topic="/speech_recognition/command"):
        super().__init__(
            outcomes=['done', 'failed', 'canceled', 'timeout'],
            input_keys=['time'],
            output_keys=['text', 'target', 'destination']
        )

        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout
        self._topic = action_topic

        self._error = False      # 标记是否在发送 Goal 时出错
        self._return = None      # 若状态被阻塞，暂存返回值
        self._start_time = None  # 记录状态进入时的时间

        # 使用 FlexBE 的 ProxyActionClient 初始化 Action 客户端
        ProxyActionClient.initialize(VoiceActionState._node)
        self._client = ProxyActionClient({self._topic: StringCommand}, wait_duration=0.0)

    def execute(self, userdata):
        '''
        在状态运行期间，持续检查 Action 是否完成，并根据结果决定状态转移。
        '''

        # 检查是否在 on_enter 阶段发生错误
        if self._error:
            return 'failed'

        # 如果之前已确定返回值（例如因超时或结果处理），直接返回
        if self._return is not None:
            return self._return

        # 检查 Action 是否已完成
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)
            userdata.text = result.answer

            # 若识别结果为 'failed'，视为失败
            if userdata.text == 'failed':
                Logger.logwarn('语音识别失败')
                self._return = 'failed'
                return self._return
            else:
                Logger.loginfo(f'语音识别结果: {userdata.text}')

                # 处理识别文本：检查是否包含关键词 'cup' 和 'kitchen'
                userdata.target = 'cup' if 'cup' in userdata.text else ''
                userdata.destination = 'kitchen' if 'kitchen' in userdata.text else ''
                # 更高级的自然语言解析可参考 Challenge 3.1 中的 search_object_and_place() 函数
                # userdata.target, userdata.destination = search_object_and_place(userdata.text)

                # 仅当同时识别到目标物体和目的地时，才视为成功
                if len(userdata.target) > 0 and len(userdata.destination) > 0:
                    self._return = 'done'
                else:
                    self._return = 'failed'

                return self._return

        # 检查是否超时
        elapsed = self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds
        if elapsed > self._timeout.nanoseconds:
            Logger.loginfo('已超过最大允许等待时间')
            self._return = 'timeout'
            return self._return

        # Action 尚未完成，继续等待
        return None

    def on_enter(self, userdata):
        """状态进入时的初始化逻辑"""
        self._error = False
        self._return = None

        # 检查必需的输入 userdata.time 是否存在
        if 'time' not in userdata:
            self._error = True
            Logger.logwarn("执行 VoiceActionState 需要 userdata.time")
            return

        # 检查 userdata.time 是否为字符串类型
        if not isinstance(userdata.time, str):
            self._error = True
            Logger.logwarn('输入类型为 %s，但要求为 string 类型', type(userdata.time).__name__)
            return

        # 记录开始时间
        self._start_time = self._node.get_clock().now()

        # 构造并发送 Goal
        goal = StringCommand.Goal()
        goal.command = str(userdata.time)  # 将监听时长作为命令传给 Action 服务器

        try:
            self._client.send_goal(self._topic, goal, wait_duration=self._timeout_sec)
        except Exception as exc:
            Logger.logwarn(f"发送 Goal 失败:\n  {type(exc)} - {exc}")
            self._error = True

    def on_exit(self, userdata):
        """状态退出时的清理逻辑"""
        # 如果 Action 仍在运行（例如用户手动中断），则取消它
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('正在取消运行中的 Action。')