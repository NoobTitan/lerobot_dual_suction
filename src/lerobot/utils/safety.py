import math
import time


from lerobot.teleoperators import Teleoperator
from lerobot.robots import Robot
from lerobot.utils.robot_utils import busy_wait


def sync_pose_slow(teleop: Teleoperator, robot: Robot, fps: int, wait_homing=False):
    if wait_homing:
        print("Please hold teleoperator near home position ...")

        homing_position = {
            "joint_1.pos": +90,
            "joint_2.pos": -90,
            "joint_3.pos": -90,
            "joint_4.pos": -90,
            "joint_5.pos": +90,
            "joint_6.pos": 0,
        }  # robot space

        within_range = False
        home_timer = 0
        notify = 1
        HOLD_TIME = 3
        THRESHOLD = 30
        while home_timer < HOLD_TIME:
            loop_start = time.perf_counter() 
            action = teleop.get_action()  # degrees

            within_range_next = True
            for key in action:
                if not (key.startswith("joint_") and key.endswith(".pos")):
                    continue

                j_offset = robot.config.joint_offsets.get(key.split('.')[0], 0.0)
                action_compensated = action[key] + j_offset  # -> gello space
                print(key, action_compensated)
                within_range_next &= abs(homing_position[key] - action_compensated) < THRESHOLD / 2
        
            loop_time = time.perf_counter() - loop_start

            if not within_range_next:
                print(f"Teleoperator pose out of range, timer reset: {0}/{HOLD_TIME}...")
                home_timer = 0
                notify = 1

            if within_range and within_range_next:
                home_timer += 1 / fps
                if home_timer >= notify:
                    print(f"hold steady: {notify}/{HOLD_TIME}...")
                    notify += 1
            
            within_range = within_range_next
            busy_wait(1 / fps - loop_time)

    print("Sync pose in progress, please keep teleoperator steady...")
    max_joint_diff = 1000  # degree
    while max_joint_diff > 1:
        loop_start = time.perf_counter()
        observation = robot.get_observation()  # radians, robot space
        action = teleop.get_action()  # degrees, gello space
        
        # there are offsets between gello space and robot space, read from robot.config.joint_offsets and compensate

        max_joint_diff = 0
        for key in action:
            if not (key.startswith("joint_") and key.endswith(".pos")):
                continue

            joffset = robot.config.joint_offsets.get(key.split('.')[0], 0.0)  # degrees
            action_value = action[key] + joffset  # gello space -> robot space
            jpose_value = observation[key] / math.pi * 180  # robot space
            j_diff = action_value - jpose_value  # 
            
            jdiff_sign = (j_diff >= 0) * 2 - 1
            jdiff_abs = abs(j_diff)
            # jupdate = jdiff_sign * max(jdiff_abs, jdiff_abs / 10)
            max_joint_diff = max(max_joint_diff, jdiff_abs)
            action[key] = jpose_value + min((50 / fps), jdiff_abs / 10) * jdiff_sign - joffset  # robot space -> gello space 

        robot.send_action(action)
        loop_time = time.perf_counter() - loop_start
        busy_wait(1 / fps - loop_time)

