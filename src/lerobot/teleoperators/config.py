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

import abc
from dataclasses import dataclass, field
from pathlib import Path

import draccus


@dataclass(kw_only=True)
class TeleoperatorConfig(draccus.ChoiceRegistry, abc.ABC):
    # Allows to distinguish between different teleoperators of the same type
    id: str | None = None
    # Directory to store calibration file
    calibration_dir: Path | None = None

    combinations: dict = field(default_factory=lambda: {
        "gello": {
            "FTAN1LRN": {
                "cs66": {
                    "192.168.101.11": {
                        "joint_offsets": {
                            "joint_1": 0.0,
                            "joint_2": -90.0,
                            "joint_3": 0.0,
                            "joint_4": -90.0,
                            "joint_5": 0.0,
                            "joint_6": 0.0,
                        },
                        "joint_inversions": {
                            "joint_1": 0,
                            "joint_2": 1,
                            "joint_3": 1,
                            "joint_4": 1,
                            "joint_5": 0,
                            "joint_6": 0,
                        },
                    },
                },
            }, 
        },
    })


    @property
    def type(self) -> str:
        return self.get_choice_name(self.__class__)
