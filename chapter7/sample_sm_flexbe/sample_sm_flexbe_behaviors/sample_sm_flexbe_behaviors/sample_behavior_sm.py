#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define Sample Behavior.

Created on Tue May 28 2024
@author: Keith Valentin
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from sample_sm_flexbe_states.eat_state import EatState
from sample_sm_flexbe_states.search_state import SearchState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class SampleBehaviorSM(Behavior):
    """
    Define Sample Behavior.

    用于理解FlexBE基础的状态机示例

    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Sample Behavior'

        # parameters of this behavior
        self.add_parameter('max_eat', 5)

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        EatState.initialize_ros(node)
        SearchState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        # x:96 y:170
        _state_machine = OperatableStateMachine(outcomes=['finished'])
        _state_machine.userdata.max_eat = self.max_eat
        _state_machine.userdata.eat_counter = 0

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:337 y:37
            OperatableStateMachine.add('Search',
                                       SearchState(),
                                       transitions={'succeeded': 'Eat', 'finished': 'finished'},
                                       autonomy={'succeeded': Autonomy.Off, 'finished': Autonomy.Off},
                                       remapping={'eat_counter': 'eat_counter', 'max_eat': 'max_eat'})

            # x:338 y:141
            OperatableStateMachine.add('Eat',
                                       EatState(),
                                       transitions={'succeeded': 'Search'},
                                       autonomy={'succeeded': Autonomy.Off},
                                       remapping={'eat_counter': 'eat_counter'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
