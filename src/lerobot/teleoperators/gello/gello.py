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

import os
import io
import logging
import time

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.dynamixel import (
    DriveMode,
    DynamixelMotorsBus,
    OperatingMode,
)

from ..teleoperator import Teleoperator
from .config_gello import GelloConfig

logger = logging.getLogger(__name__)

class Gello(Teleoperator):
    """
    Developed by [GELLO](https://wuphilipp.github.io/gello_site/). 
    A General, Low-Cost, and Intuitive Teleoperation Framework for Robot Manipulators.
    """

    config_class = GelloConfig
    name = "gello"

    def __init__(self, config: GelloConfig):
        super().__init__(config)
        self.config = config

        if self.config.serial_number is not None:
            self.config.port = self.discover_device(self.config.serial_number)
        elif self.config.port is not None:
            logger.warning("No serial_number specified, this is discouraged. Port numbers are not bind to specific devices and would change between runs.")
            self.config.serial_number = self.reverse_discover_device(self.config.port)
            self.config.id = self.config.serial_number

        self.bus = DynamixelMotorsBus(
            port=self.config.port,
            motors={
                "joint_1": Motor(1, "xl330-m288", MotorNormMode.DEGREES),
                "joint_2": Motor(2, "xl330-m288", MotorNormMode.DEGREES),
                "joint_3": Motor(3, "xl330-m288", MotorNormMode.DEGREES),
                "joint_4": Motor(4, "xl330-m288", MotorNormMode.DEGREES),
                "joint_5": Motor(5, "xl330-m288", MotorNormMode.DEGREES),
                "joint_6": Motor(6, "xl330-m288", MotorNormMode.DEGREES),
                "end_effector": Motor(7, "xl330-m077", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )

    @staticmethod
    def reverse_discover_device(port):
        """ find device serial by port """

        sys_bus = "/sys/bus/usb/devices"
        usb_dev = os.listdir(sys_bus)

        _, dev_name = os.path.split(port)  # dev_name == "ttyUSB*"

        assert dev_name.startswith("ttyUSB")

        for dev_id in usb_dev:
            if ":" in dev_id:  # usb device function
                continue

            dev_path = os.path.join(sys_bus, dev_id)
            f_serial = os.path.join(dev_path, "serial")

            if not os.path.exists(f_serial):
                continue

            with io.open(f_serial) as f:
                dev_serial = f.read().strip()

            for fn in os.listdir(dev_path):
                if not fn.startswith(dev_id + ":"):
                    continue

                fn_path = os.path.join(sys_bus, dev_id, fn)
                if not os.path.isdir(fn_path):
                    continue

                devfs_files = os.listdir(fn_path)

                if dev_name in devfs_files:
                    pass

                return dev_serial

        raise ValueError(f"could not determine the serial number for port `{port}`.")

    @staticmethod
    def discover_device(serial_number):
        """ find device port by its serial """

        sys_bus = "/sys/bus/usb/devices"
        usb_dev = os.listdir(sys_bus)

        found_serial_match = False

        for dev_id in usb_dev:
            if ":" in dev_id:  # usb device function
                continue

            dev_path = os.path.join(sys_bus, dev_id)
            f_serial = os.path.join(dev_path, "serial")

            if not os.path.exists(f_serial):
                continue

            with io.open(f_serial) as f:
                dev_serial = f.read().strip()

            if dev_serial != serial_number:
                # This is not the device we searched for.
                continue

            found_serial_match = True

            for fn in os.listdir(dev_path):
                if not fn.startswith(dev_id + ":"):
                    continue

                fn_path = os.path.join(sys_bus, dev_id, fn)
                if not os.path.isdir(fn_path):
                    continue

                devfs_files = os.listdir(fn_path)
                tty_list = list(filter(
                    lambda x: x.startswith("ttyUSB") and len(x) > len("ttyUSB"),
                    devfs_files
                ))

                if len(tty_list) == 0:
                    # not a ttyUSB device/function.
                    continue

                tty_list.sort()
                return f"/dev/{tty_list[0]}"

        if found_serial_match:
            raise ValueError(f"The device with serial number `{serial_number}` is found, but it is not a ttyUSB device.")

        raise ValueError(f"Could not find the device with serial number: `{serial_number}`.")

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def check_drive_mode_consistency(self) -> None:
        drive_mode_consistency = True
        for motor_id in self.calibration:            
            drive_mode_consistency &= (self.calibration[motor_id].drive_mode == self.config.joint_inversions[motor_id])

        if not drive_mode_consistency:
            raise ValueError("`joint_inversions` values in configuration (GelloConfig) mismatch with calibration values, please redo calibration.")

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        # N.B.: delayed check on drive_mode consistency before connect:
        # this is because there are combination (teleoperator + robot) specific configurations that is ensured after initializing both side.
        self.check_drive_mode_consistency()

        self.bus.connect()
        for motor_id in self.bus.motors:
            # drive mode is handled by software.
            self.bus.write("Drive_Mode", motor_id, DriveMode.NON_INVERTED.value)

        if not self.is_calibrated and calibrate:
            self.calibrate()

        # self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value)

        drive_modes = {
            motor: self.config.joint_inversions.get(motor, DriveMode.NON_INVERTED.value) 
            for motor in self.bus.motors
        }

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()

        full_turn_motors = ["joint_1","joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        unknown_range_motors = [motor for motor in self.bus.motors if motor not in full_turn_motors]
        print(
            f"Move all joints except {full_turn_motors} sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion(unknown_range_motors)
        for motor in full_turn_motors:
            range_mins[motor] = 0
            range_maxes[motor] = 4095

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=drive_modes[motor],
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        logger.info(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        self.bus.disable_torque()
        self.bus.configure_motors()
        for motor in self.bus.motors:
            if motor != "end_effector":
                # Use 'extended position mode' for all motors except end_effector, because in joint mode the servos
                # can't rotate more than 360 degrees (from 0 to 4095) And some mistake can happen while
                # assembling the arm, you could end up with a servo with a position 0 or 4095 at a crucial
                # point
                self.bus.write("Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value)

        # Use 'position control current based' for end_effector to be limited by the limit of the current.
        # For the follower gripper, it means it can grasp an object without forcing too much even tho,
        # its goal position is a complete grasp (both gripper fingers are ordered to join and reach a touch).
        # For the leader gripper, it means we can use it as a physical trigger, since we can force with our finger
        # to make it move, and it will move back to its original target position when we release the force.
        self.bus.write("Operating_Mode", "end_effector", OperatingMode.CURRENT_POSITION.value)
        # Set end_effector's goal pos in current position mode so that we can use it as a trigger.
        self.bus.enable_torque("end_effector")
        if self.is_calibrated:
            self.bus.write("Goal_Position", "end_effector", self.config.end_effector_open_pos)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        action = self.bus.sync_read("Present_Position")
        action = {f"{motor}.pos": val for motor, val in action.items()}

        # apply offset.
        for key in action:
            action[key] = action[key] + self.config.joint_offsets.get(key, 0)

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect()
        logger.info(f"{self} disconnected.")
