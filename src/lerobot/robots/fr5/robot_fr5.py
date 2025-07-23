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


import logging
import time
from functools import cached_property
from typing import Any

import numpy as np
from fairino import Robot as FairinoRobot

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.constants import OBS_IMAGES, OBS_STATE
from lerobot.datasets.utils import get_nested_item

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .configuration_fr5 import FairinoV5Config

logger = logging.getLogger(__name__)

class FairinoV5(Robot):
    """
    Cobot Fairino V5 (Fr5) developed by [Fairino](https://fairino.com/).
    """

    config_class = FairinoV5Config
    name = "fr5"

    def __init__(self, config: FairinoV5Config):
        # raise NotImplementedError
        super().__init__(config)

        self.config = config
        # self.robot_type = self.config.type
        
        # 初始化相机参数
        self.cameras = make_cameras_from_configs(self.config.cameras)

        self._is_connected = False
        self._calibrated = False
        self._last_suction_command = None

    # 机器人标定 fr5 机器人功能完成标定
    def calibrate(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        logger.info(f"{self} calibration completed.")
        self._calibrated = True

    # fr5和相机连接
    def connect(self) -> None:
        # if not self.robot.connect_to_robot():
        #     raise ConnectionError()
        # 连接机器人
        self.robot = FairinoRobot.RPC(ip=self.config.ip)

        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        for cam in self.cameras.values():
            cam.connect()

        self.is_connected = True
        logger.info(f"{self} connected. ip address: {self.config.ip}")

        _, defaulttransvel = self.robot.GetDefaultTransVel()
        print("默认速度为：", defaulttransvel)

        self.configure()
        err, joint_states = self.robot.GetActualJointPosDegree()
        if err != 0:
            raise RuntimeError (f"{self} Find Error Code: {err} in Fairino SDK Comparison Table")
        else:
            print(f"{self} joint states: {joint_states}")


    def configure(self):
        # Configure the fr5.
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        # 这里猜测需要使用伺服模式运行
        # 启动伺服模式
        err = self.robot.ServoMoveStart()
        if err != 0:
            raise RuntimeError(f"{self} failed to start servo mode. Error code: {err}")
        
    def get_observation(self, features: list[str] | None = None) -> dict[str, Any]:
        # raise NotImplementedError
        """
        Get the current observation from the robot and cameras.

        Args:
            features (list[str] | None): List of features to return. If None, return all features.

        Returns:
            dict[str, Any]: Observation dictionary with joint positions and camera images.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        # 获取关节位置
        err, joint_states = self.robot.GetActualJointPosDegree()
        if err != 0:
            raise RuntimeError(f"{self} failed to get joint states. Error code: {err}")
        
        obs_dict = {f"joint_{i+1}.pos": float(state) for i, state in enumerate(joint_states)}
        error, [_, obs_dict["end_effector.pos"]] = self.robot.GetDO()
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def disconnect(self):
        """Disconnect the robot and cameras."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        # 关闭伺服模式
        err = self.robot.ServoMoveEnd()
        if err != 0:
            raise RuntimeError(f"{self} failed to stop servo mode. Error code: {err}")

        # 关闭RPC连接
        self.robot.CloseRPC()

        for cam in self.cameras.values():
            cam.disconnect()
        
        logger.info(f"{self} disconnected.")

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        if not self.is_connected:
            raise ConnectionError()
        
        epos = [0.0,0.0,0.0,0.0]
        dt = 0.0167

        ee_pose = action.get("end_effector.pos", None)
        joint_keys = [f"joint_{i+1}.pos" for i in range(6)]
        #########################################################
        # 增加安全行程保护
        goal_present_pos = {}
        _, joint_states = self.robot.GetActualJointPosDegree() # 当前关节状态，单位:角度
        if joint_states is None:
            raise RuntimeError(f"{self} failed to get joint states.")
        
        for i, key in enumerate(joint_keys):
            if key in action:
                goal_deg = float(action[key])+ self.config.joint_offsets.get(key.split('.')[0], 0.0)
                goal_present_pos[key] = (goal_deg, joint_states[i])

        safe_goal_positions = ensure_safe_goal_position(goal_present_pos, max_relative_target=180 * dt)

        # 替换 action 中的目标值
        for key in safe_goal_positions:
            action[key] = safe_goal_positions[key]

        cmd_robot_joints = [
            float(action[key]) for key in joint_keys if key in action
        ]
        #########################################################
        #  简单控制，发送接收到的关节位置指令，没有增加安全行程保护
        # cmd_robot_joints = [
        #     float(action[key]) + self.config.joint_offsets.get(key.split('.')[0], 0.0) for key in joint_keys if key in action
        # ]
        #########################################################
        
        # === 吸盘控制逻辑 ===
        _, [_, suction_state] = self.robot.GetDO() # 0 或 1，转成 float
        if ee_pose is not None:
            target_suction = ee_pose < 50.0  # 小于 50 表示吸附
            # 如果目标状态和实际状态不一致，发出切换命令
            if suction_state != float(target_suction):
                self.robot.SetDO(0, int(target_suction))  # 设置吸盘状态
                self._last_suction_command = target_suction
                time.sleep(0.05)

        self.robot.ServoJ(cmd_robot_joints, epos, cmdT=dt)

        print(cmd_robot_joints)
        return action
        # return cmd_robot_joints


#########################################################

    @property
    def is_connected(self) -> bool:
        """Check if robot is connected."""
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        """Set connection status."""
        self._is_connected = value

    @property
    def is_calibrated(self) -> bool:
        return getattr(self, "_calibrated", False)

    
    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }
    
    @property
    def _motor_ft(self) -> dict[str, type]:
        joint_states = self.robot.GetActualJointPosDegree(flag=1)
        motors = {f"joint_{i+1}.pos": float for i in range(len(joint_states))}
        motors["end_effector.pos"] = float # 定义末端类型
        # if joint_states is None:
        #     raise RuntimeError(f"{self} failed to get joint states.")
        return motors
    
    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        """Joint position types and camera observation shapes."""
        return {
            **self._motor_ft,
            **self._cameras_ft
        }
    
    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motor_ft