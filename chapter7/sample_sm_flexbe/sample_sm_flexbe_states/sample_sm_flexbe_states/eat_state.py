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

"""Eat state."""
from rclpy.duration import Duration

from flexbe_core import EventState, Logger

from time import sleep


class EatState(EventState):
    """
    EatState 状态的目标是吃掉前一状态中找到的零食。
    它不考虑用户迄今为止已吃掉的零食数量，而是随机决定是否吃掉（注：当前实现实际为确定性增加，但文档保留原意）。

    输出
    <= succeeded       表示 Eat 状态已成功完成

    Userdata
    ># eat_counter  int 用户迄今为止吃掉的零食数量（输入，int 类型）
    #> eat_counter  int 更新并输出吃掉的零食数量（输出，int 类型）
    """

    def __init__(self):
        """定义状态的结果和输入/输出键。"""
        super().__init__(outcomes=['succeeded'],
                         input_keys=['eat_counter'],
                         output_keys=['eat_counter'])

    def execute(self, userdata):
        # 开始 eat 处理
        sleep(1)
        Logger.loginfo('吃掉1个甜点！')  # 记录已进食的日志
        userdata.eat_counter += 1  # 更新 eat_counter
        Logger.loginfo('截至目前，已吃掉 {} 个甜点！'.format(userdata.eat_counter))  # 记录当前吃掉的甜点总数

        return 'succeeded'  # 返回 'succeeded' 结果