from queue import Queue
from spacemouse_shared_memory import Spacemouse
from multiprocessing.managers import SharedMemoryManager
import numpy as np
import multiprocessing as mp
import time

from transforms3d import euler
import argparse

GRIPPER_MAX = 0.08


class TeleopNode(mp.Process):
    def __init__(
        self,
        spacemouse: Spacemouse,
        ori_speed=0.3,
        pos_speed=0.1,
        gripper_speed=0.04,
        ctrl_freq=50.0,
        spacemouse_window_size=10,
    ):
        super().__init__()
        self.spacemouse = spacemouse
        self.ori_speed = ori_speed
        self.pos_speed = pos_speed
        self.gripper_speed = gripper_speed

        self.ctrl_freq = ctrl_freq
        self.spacemouse_queue = Queue(spacemouse_window_size)
        self.started = False
        self.eef_pose = None  # [x, y, z, roll, pitch, yaw]
        self.gripper_pos = None
        self.robot_tick = None
        self.start_robot_tick = None
        self.start_time = None
        self.start_pose = None
        self.target_translation = None
        self.target_rotation_rpy = None
        self.target_gripper_pos = None
        
        self.manager = mp.Manager()
        self.target_pose = self.manager.dict()

    def eef_state_update(self, msg):
        translation = np.array(msg.eef_pose[:3])
        rotation_rpy = euler.quat2euler(msg.eef_pose[3:])
        self.eef_pose = np.concatenate([translation, rotation_rpy])
        self.gripper_pos = msg.gripper_pos
        self.robot_tick = msg.tick

    def update_target_pose(self):
        return dict(self.target_pose)

    def run(self):
        try:
            while True:
                if not self.started:
                    button_left = self.spacemouse.is_button_pressed(0)
                    button_right = self.spacemouse.is_button_pressed(1)
                    state = self.get_filtered_spacemouse_output()
                    if state.any() or button_left or button_right:
                        if self.eef_pose is not None:
                            print(f"Start tracking!")
                            self.started = True
                            self.start_robot_tick = self.robot_tick
                            self.start_time = time.monotonic()
                            self.start_pose = self.eef_pose
                            self.target_translation = self.start_pose[:3]
                            self.target_rotation_rpy = self.start_pose[3:]
                            self.target_gripper_pos = self.gripper_pos
                        else:
                            print(f"EEF state not received yet!")
                            continue
                else:
                    spacemouse_state = self.get_filtered_spacemouse_output()
                    button_left = self.spacemouse.is_button_pressed(0)
                    button_right = self.spacemouse.is_button_pressed(1)
                    if self.eef_pose is not None and button_left and button_right:
                        print(f"Resetting target pose to current pose!")
                        self.target_translation = self.eef_pose[:3]
                        self.target_rotation_rpy = self.eef_pose[3:]
                        self.target_gripper_pos = self.gripper_pos
                        return
                    if button_left and not button_right:
                        gripper_cmd = 1
                    elif button_right and not button_left:
                        gripper_cmd = -1
                    else:
                        gripper_cmd = 0

                    assert self.target_translation is not None, "Target pose is None!"
                    # Hack: remap the directions
                    self.target_translation += (
                        spacemouse_state[:3] * self.pos_speed * (1 / self.ctrl_freq)
                    )
                    # ) * np.array([-1, -1, 1])
                    self.target_rotation_rpy += (
                        spacemouse_state[3:] * self.ori_speed * (1 / self.ctrl_freq)
                    )
                    assert self.target_gripper_pos is not None, "Target gripper pos is None!"
                    self.target_gripper_pos += (
                        gripper_cmd * self.gripper_speed * (1 / self.ctrl_freq)
                    )
                    self.target_gripper_pos = np.clip(self.target_gripper_pos, 0, GRIPPER_MAX)

                    frame = {}
                    target_quat_wxyz = euler.euler2quat(*self.target_rotation_rpy)
                    self.target_pose['eef_pose'] = np.concatenate([self.target_translation, target_quat_wxyz])
                    self.target_pose['gripper_pos'] = self.target_gripper_pos
                    self.target_pose['tick'] = (
                        self.start_robot_tick
                        + int((time.monotonic() - self.start_time) * 1000)
                        + 100
                    )
                time.sleep(0.01)
        except:
            print('space mouse controller ends')

    def get_filtered_spacemouse_output(self):
        state = self.spacemouse.get_motion_state_transformed()
        if (
            self.spacemouse_queue.maxsize > 0
            and self.spacemouse_queue._qsize() == self.spacemouse_queue.maxsize
        ):
            self.spacemouse_queue._get()
        self.spacemouse_queue.put_nowait(state)
        return np.mean(np.array(list(self.spacemouse_queue.queue)), axis=0)


if __name__ == "__main__":
    with SharedMemoryManager() as smm:
        with Spacemouse(shm_manager=smm, deadzone=0.3, max_value=500) as spacemouse:
            node = TeleopNode(spacemouse)
            while True:
                print(node.update_target_pose())
