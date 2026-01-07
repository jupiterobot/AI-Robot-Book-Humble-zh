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

"""Voice Action FlexBE State."""

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from airobot_interfaces.action import StringCommand


class VoiceActionState(EventState):
    """
    通过 Action 通信启动语音识别，并将其结果赋值给 userdata.text

    启动方法:
        为执行此状态，请启动所需的 Action Server：
        $ ros2 run pseudo_node_action voice_node

        为显示可执行的 Action 列表，请运行以下命令：
        $ ros2 action list

    参数
    -- timeout             最大允许时间（秒）
    -- action_topic        语音识别的 Action 名称

    输出
    <= done                语音识别成功时
    <= failed              因某种原因失败时
    <= canceled            用户发出取消请求时
    <= timeout             超过目的地移动的最大允许时间时

    Userdata
    ># time        string  语音识别的执行时间（秒数）（string 类型）（输入）
    #> text        string  语音识别的结果（string 类型）（输出）
    #> target      string  语音识别得出的抓取物体结果（string 类型）（输出）
    #> destination string  语音识别得出的目的地结果（string 类型）（输出）

    """

    def __init__(self, timeout, action_topic="/ps_voice/command"):
        super().__init__(outcomes=['done', 'failed', 'canceled', 'timeout'],
                         input_keys=['time'],
                         output_keys=['text', 'target', 'destination'])

        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout
        self._topic = action_topic

        self._error      = False # 从 Action 客户端发送 Goal 失败时
        self._return     = None  # 当操作员阻止结果输出时，保存返回值
        self._start_time = None  # 初始化开始时间

        # 使用 FlexBE 的 ProxyActionClient 创建 Action 客户端
        ProxyActionClient.initialize(VoiceActionState._node)
        self._client = ProxyActionClient({self._topic: StringCommand},
                                         wait_duration=0.0)

    def execute(self, userdata):
        '''
        运行期间，检查 Action 是否已结束，并根据其结果决定 outcome
        '''

        # 检查是否发生 _error
        if self._error:
            return 'failed' # 返回 'failed'

        # 如果状态迁移被阻塞，则返回之前的返回值
        if self._return is not None:
            return self._return

        # 检查 Action 是否已结束
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic) # 获取 Action 的结果
            userdata.text = result.answer

            if userdata.text == 'failed':
                Logger.logwarn('语音识别失败了')
                self._return = 'failed'
                return self._return # 返回 'failed'
            else:
                Logger.loginfo(f'语音识别的结果: {userdata.text}')

                # 需要处理语音识别的结果
                userdata.target      = 'cup'
                userdata.destination = 'kitchen'

                self._return = 'done'
                return self._return # 返回 'done'

        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
            # 检查是否超过最大允许时间
            Logger.loginfo('已超过最大允许时间')
            self._return = 'timeout'
            return self._return # 返回 'timeout'

        # 如果 Action 尚未结束，则不终止状态
        return None

    def on_enter(self, userdata):
        # 初始化数据
        self._error = False
        self._return = None

        # 检查 userdata 中是否存在 time 信息
        if 'time' not in userdata:
            self._error = True
            Logger.logwarn("要执行 VoiceActionState，需要 userdata.time！")
            return

        if not isinstance(userdata.time, (str)):
            self._error = True
            Logger.logwarn('输入的类型是 %s。要求为 string 类型', type(userdata.target).__name__)

        # 记录开始时间
        self._start_time = self._node.get_clock().now()

        # 向 Action Server 发送 Goal
        goal = StringCommand.Goal()
        goal.command = str(userdata.time)

        try:
            self._client.send_goal(self._topic, goal, wait_duration=self._timeout_sec)
        except Exception as exc:  # pylint: disable=W0703
            Logger.logwarn(f"Goal 发送失败:\n  {type(exc)} - {exc}")
            self._error = True

    def on_exit(self, userdata):
        # 确认 Action 是否未启动
        # 如果 Action 正在运行，则认为是操作员手动输出了结果
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('正在取消运行中的 Action。')