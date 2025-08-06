#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

from dataclasses import dataclass, field

from ..config import TeleoperatorConfig

from typing import Dict

from lerobot.motors.dynamixel import (
    DriveMode,
)

@TeleoperatorConfig.register_subclass("gello")
@dataclass
class GelloConfig(TeleoperatorConfig):
    # Port to connect to the arm
    port: str = None
    serial_number: str = None

    end_effector_open_pos: float = 50.0

    # 关节偏移量，按 joint 名称映射偏移值
    # the default value is for cs66.
    # the value is robot dependent, see teleoperate.py: TeleoperateConfig.combinations
    joint_offsets: Dict[str, float] = field(default_factory=lambda: {
        "joint_1": 0.0,
        "joint_2": -90.0,
        "joint_3": 0.0,
        "joint_4": -90.0,
        "joint_5": 0.0,
        "joint_6": 0.0,
    })

    # This is used during calibration.
    # /!\ IMPORTANT: Redo calibration if this is changed.
    joint_inversions: Dict[str, DriveMode] = field(default_factory=lambda: {
        "joint_1": DriveMode.NON_INVERTED.value,
        "joint_2": DriveMode.INVERTED.value,
        "joint_3": DriveMode.INVERTED.value,
        "joint_4": DriveMode.INVERTED.value,
        "joint_5": DriveMode.NON_INVERTED.value,
        "joint_6": DriveMode.NON_INVERTED.value,
    })

    def __post_init__(self):
        self.id = self.serial_number
