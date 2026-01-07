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

"""Vision Action FlexBE State."""

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from airobot_interfaces.action import StringCommand


class VisionActionState(EventState):
    """
    通过 Action 通信启动物体识别，并将结果存入 userdata.text

    启动方法:
        为执行此状态，请先启动对应的 Action Server：
        $ ros2 run pseudo_node_action vision_node

        要查看可用的 Action 列表，请运行以下命令：
        $ ros2 action list

    参数
    -- timeout             最大允许时间（秒）
    -- action_topic        物体识别的 Action 名称

    输出
    <= done                物体识别成功时
    <= failed              因任何原因失败时
    <= canceled            用户请求取消时
    <= timeout             超过最大允许时间时

    Userdata
    ># target   string     要识别的物体名称（string 类型）（输入）
    #> text     string     物体识别的结果（string 类型）（输出）

    """

    def __init__(self, timeout, action_topic="/ps_vision/command"):
        super().__init__(outcomes=['done', 'failed', 'canceled', 'timeout'],
                         input_keys=['target'],
                         output_keys=['text'])

        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout
        self._topic = action_topic

        self._error      = False  # 当从 Action Client 发送 Goal 失败时置为 True
        self._return     = None   # 若状态转换被阻塞，则暂存返回值
        self._start_time = None   # 初始化开始时间

        # 使用 FlexBE 的 ProxyActionClient 创建 Action 客户端
        ProxyActionClient.initialize(VisionActionState._node)
        self._client = ProxyActionClient({self._topic: StringCommand},
                                         wait_duration=0.0)

    def execute(self, userdata):
        '''
        在运行期间，检查 Action 是否已完成，并根据结果决定 outcome
        '''

        # 检查是否发生错误
        if self._error:
            return 'failed'  # 返回 'failed'

        # 如果状态转换被阻塞，则返回之前保存的结果
        if self._return is not None:
            return self._return

        # 检查 Action 是否已完成
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)  # 获取 Action 结果
            userdata.text = result.answer

            if userdata.text == 'failed':
                Logger.logwarn('物体识别失败')
                self._return = 'failed'
                return self._return  # 返回 'failed'
            else:
                Logger.loginfo(f'物体识别结果: {userdata.text}')
                self._return = 'done'
                return self._return  # 返回 'done'

        # 检查是否超时
        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
            Logger.loginfo('已超过最大允许时间')
            self._return = 'timeout'
            return self._return  # 返回 'timeout'

        # Action 尚未完成，保持当前状态
        return None

    def on_enter(self, userdata):
        # 初始化数据
        self._error = False
        self._return = None

        # 检查 userdata 中是否存在 'target'
        if 'target' not in userdata:
            self._error = True
            Logger.logwarn("执行 VisionActionState 需要 userdata.target！")
            return

        # 检查输入值是否为 string 类型
        if not isinstance(userdata.target, (str)):
            self._error = True
            Logger.logwarn('输入类型为 %s，但需要 string 类型', type(userdata.target).__name__)

        # 记录开始时间
        self._start_time = self._node.get_clock().now()

        # 向 Action Server 发送 Goal
        goal = StringCommand.Goal()
        goal.command = str(userdata.target)

        try:
            self._client.send_goal(self._topic, goal, wait_duration=self._timeout_sec)
        except Exception as exc:  # pylint: disable=W0703
            Logger.logwarn(f"发送 Goal 失败:\n  {type(exc)} - {exc}")
            self._error = True

    def on_exit(self, userdata):
        # 确认 Action 是否仍在运行
        # 如果仍在运行，说明是被操作员手动中断的
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('正在取消运行中的 Action。')