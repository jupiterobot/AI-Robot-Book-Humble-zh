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

"""Grasp state."""
from rclpy.duration import Duration

from flexbe_core import EventState, Logger

import random
from time import sleep


class GraspState(EventState):
    """
    GraspState 状态的目标是抓取甜点。

    输出
    <= succeeded       表示成功抓取了找到的甜点
    <= failed          表示由于某种问题导致抓取失败
    """

    def __init__(self):
        """定义状态的结果和输入键。"""
        super().__init__(outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        # 开始 grasp 处理
        sleep(1)  # 为了可视化处理过程，暂停 1 秒
        Logger.loginfo('尝试抓取甜点')  # 记录当前处于抓取状态

        prob = random.random()  # 生成一个 [0,1] 范围内的随机值
        if 0.5 > prob:
            Logger.loginfo('成功抓取了甜点！')  # Grasp 状态成功时

            return 'succeeded'  # 返回 'succeeded' 结果
        else:
            Logger.loginfo('未能抓取甜点……再试一次！')  # Grasp 状态失败时

            return 'failed'  # 返回 'failed' 结果