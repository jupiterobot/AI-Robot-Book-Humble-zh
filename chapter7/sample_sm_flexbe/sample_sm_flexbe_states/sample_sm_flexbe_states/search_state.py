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

"""Search state."""
from rclpy.duration import Duration

from flexbe_core import EventState, Logger

from time import sleep


class SearchState(EventState):
    """
    SearchState 状态的目标是寻找甜点。
    根据用户迄今为止吃掉的甜点数量，判断是否继续寻找。

    输出
    <= succeeded       找到甜点时，输出检测成功的结果
    <= finished        当用户已经吃饱（达到最大食用数量）时，输出搜索结束的结果
    <= failed          若因某种问题导致搜索失败，则输出失败结果（注：当前实现未使用此分支）

    Userdata
    ># eat_counter  int 用户迄今为止吃掉的甜点数量（输入，int 类型）
    ># max_eat      int 用户吃饱前所能吃的甜点总数（输入，int 类型）
    """

    def __init__(self):
        """定义状态的结果和输入键。"""
        super().__init__(outcomes=['succeeded', 'finished'],
                         input_keys=['eat_counter', 'max_eat'])

    def execute(self, userdata):
        # 开始 search 处理
        sleep(1)
        Logger.loginfo('正在搜索甜点')  # 记录当前处于搜索状态

        if userdata.eat_counter < userdata.max_eat:
            Logger.loginfo('找到了甜点！')  # 若吃掉的数量尚未达到上限

            return 'succeeded'  # 返回 'succeeded' 结果
        else:
            Logger.loginfo('已经吃饱了……')  # 若已达到最大食用数量

            return 'finished'  # 返回 'finished' 结果