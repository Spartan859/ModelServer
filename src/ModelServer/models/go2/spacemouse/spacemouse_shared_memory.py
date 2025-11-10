import multiprocessing as mp
from queue import Queue
import time
from multiprocessing.managers import SharedMemoryManager

import numpy as np
from transforms3d import euler
from spnav import (
    SpnavButtonEvent,
    SpnavMotionEvent,
    spnav_close,
    spnav_open,
    spnav_poll_event,
)

from shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer

GRIPPER_MAX = 0.08


class Spacemouse(mp.Process):
    def __init__(
        self,
        shm_manager,
        get_max_k=30,
        frequency=200,
        max_value=500,
        deadzone=(0, 0, 0, 0, 0, 0),
        dtype=np.float32,
        n_buttons=2,
        spacemouse_window_size=10,
        ori_speed=0.3,
        pos_speed=0.1,
        gripper_speed=0.04,
    ):
        """
        Continuously listen to 3D connection space naviagtor events
        and update the latest state.

        max_value: {300, 500} 300 for wired version and 500 for wireless
        deadzone: [0,1], number or tuple, axis with value lower than this value will stay at 0

        front
        z
        ^   _
        |  (O) space mouse
        |
        *----->x right
        y
        """
        super().__init__()
        if np.issubdtype(type(deadzone), np.number):
            deadzone = np.full(6, fill_value=deadzone, dtype=dtype)
        else:
            deadzone = np.array(deadzone, dtype=dtype)
        assert (deadzone >= 0).all()

        # copied variables
        self.frequency = frequency
        self.max_value = max_value
        self.dtype = dtype
        self.deadzone = deadzone
        self.n_buttons = n_buttons
        self.ori_speed = ori_speed
        self.pos_speed = pos_speed
        self.gripper_speed = gripper_speed
        # self.motion_event = SpnavMotionEvent([0,0,0], [0,0,0], 0)
        # self.button_state = defaultdict(lambda: False)
        self.tx_zup_spnav = np.array([[0, 0, -1], [1, 0, 0], [0, 1, 0]], dtype=dtype)
        self.target_pose_increment = np.zeros((7,), dtype=np.float32)

        example = {
            # 3 translation, 3 rotation, 1 period
            "motion_event": np.zeros((7,), dtype=np.int64),
            # left and right button
            "button_state": np.zeros((n_buttons,), dtype=bool),
            "receive_timestamp": time.monotonic(),
            # 3 translation, 3 rotation, 1 gripper
            "target_pose_increment": np.zeros((7,), dtype=np.float32),
        }
        ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency,
        )

        # shared variables
        self.ready_event = mp.Event()
        self.stop_event = mp.Event()
        self.ring_buffer = ring_buffer

        self.spacemouse_queue = Queue(spacemouse_window_size)

    # ======= get state APIs ==========

    def get_motion_state(self):
        state = self.ring_buffer.get()
        motion_event = state["motion_event"]
        assert isinstance(motion_event, np.ndarray)
        state = np.array(motion_event[:6], dtype=self.dtype) / self.max_value
        is_dead = (-self.deadzone < state) & (state < self.deadzone)
        state[is_dead] = 0
        return state

    def get_motion_state_transformed(self):
        """
        Return in right-handed coordinate
        z
        *------>y right
        |   _
        |  (O) space mouse
        v
        x
        back

        """
        state = self.get_motion_state()
        tf_state = np.zeros_like(state)
        tf_state[:3] = self.tx_zup_spnav @ state[:3]
        tf_state[3:] = self.tx_zup_spnav @ state[3:]
        return tf_state

    def get_button_state(self):
        state = self.ring_buffer.get()
        return state["button_state"]

    def is_button_pressed(self, button_id):
        button_state = self.get_button_state()
        assert isinstance(button_state, np.ndarray)
        return button_state[button_id]

    # ========== start stop API ===========

    def start(self, wait=True):
        super().start()
        if wait:
            self.ready_event.wait()

    def stop(self, wait=True):
        self.stop_event.set()
        if wait:
            self.join()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def get_filtered_spacemouse_output(self):
        state = self.get_motion_state_transformed()
        if (
            self.spacemouse_queue.maxsize > 0
            and self.spacemouse_queue._qsize() == self.spacemouse_queue.maxsize
        ):
            self.spacemouse_queue._get()
        self.spacemouse_queue.put_nowait(state)
        return np.mean(np.array(list(self.spacemouse_queue.queue)), axis=0)

    def update_increment_pose(self):
        spacemouse_state = self.get_filtered_spacemouse_output()
        button_left = self.is_button_pressed(0)
        button_right = self.is_button_pressed(1)

        if button_left and not button_right:
            gripper_cmd = 1
        elif button_right and not button_left:
            gripper_cmd = -1
        else:
            gripper_cmd = 0

        # Hack: remap the directions
        self.target_pose_increment[:3] += (
            spacemouse_state[:3] * self.pos_speed * (1 / self.frequency)
        )
        # ) * np.array([-1, -1, 1])
        self.target_pose_increment[3:-1] += (
            spacemouse_state[3:] * self.ori_speed * (1 / self.frequency)
        )
        self.target_pose_increment[-1] += (
            gripper_cmd * self.gripper_speed * (1 / self.frequency)
        )
        self.target_pose_increment[-1] = np.clip(self.target_pose_increment[-1], 0, GRIPPER_MAX)

    def get_target_pose(self, eef_pose, gripper_pos):
        state = self.ring_buffer.get()
        target_pose_increment = state["target_pose_increment"]

        target_translation = eef_pose[:3] + target_pose_increment[:3]
        target_rotation_rpy = eef_pose[3:-1] + target_pose_increment[3:-1]
        target_gripper_pos = gripper_pos + target_pose_increment[-1]

        target_quat_wxyz = euler.euler2quat(*target_rotation_rpy)
        return {
            'eef_pose': np.concatenate([target_translation, target_quat_wxyz]), 
            'gripper_pos': target_gripper_pos
        }

    # ========= main loop ==========
    def run(self):
        spnav_open()
        try:
            motion_event = np.zeros((7,), dtype=np.int64)
            button_state = np.zeros((self.n_buttons,), dtype=bool)
            # send one message immediately so client can start reading
            self.ring_buffer.put(
                {
                    "motion_event": motion_event,
                    "button_state": button_state,
                    "receive_timestamp": time.monotonic(),  # type: ignore
                }
            )
            self.ready_event.set()

            while not self.stop_event.is_set():
                event = spnav_poll_event()
                receive_timestamp = time.monotonic()
                if isinstance(event, SpnavMotionEvent):
                    motion_event[:3] = event.translation
                    motion_event[3:6] = event.rotation
                    motion_event[6] = event.period
                elif isinstance(event, SpnavButtonEvent):
                    button_state[event.bnum] = event.press
                else:
                    # finish integrating this round of events
                    # before sending over
                    self.update_increment_pose()
                    self.ring_buffer.put(
                        {
                            "motion_event": motion_event,
                            "button_state": button_state,
                            "receive_timestamp": receive_timestamp,  # type: ignore
                            "target_pose_increment": self.target_pose_increment,
                        }
                    )
                    time.sleep(1 / self.frequency)
        finally:
            spnav_close()


def test():
    with SharedMemoryManager() as shm_manager:
        with Spacemouse(shm_manager=shm_manager, deadzone=0.3, max_value=500) as sm:
            for i in range(2000):
                # print(sm.get_motion_state())
                print(sm.get_motion_state_transformed())
                print(sm.is_button_pressed(0))
                time.sleep(1 / 100)


if __name__ == "__main__":
    test()
