#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-05-08
################################################################

import copy
import json
import threading
import multiprocessing
import time
import numpy as np

import rospy
from sensor_msgs.msg import JointState
from xpkg_arm_msgs.msg import XmsgArmJointParam
from xpkg_arm_msgs.msg import XmsgArmJointParamList


class JointTrack:

    def __init__(self):
        ### ros node
        rospy.init_node("joint_track", anonymous=True)
        
        ### subscriber
        self.__joint_state_sub = rospy.Subscriber(
            '/xtopic_arm/joint_states',
            JointState,
            self.__joint_state_callback,
        )

        ### variable
        self.__delta_lr_max = 0.10 * 1
        self.__delta_dm_max = 0.05 * 4
        # target
        self.__target_change_thresh = 0.2
        # deadzone - for each joint
        self.__deadzone_thresh = np.array([0.02, 0.02, 0.02, 0.02, 0.02, 0.02])
        
        self.__joint_num = 6
        # state
        self.__pos_lock = threading.Lock()
        self.__cur_pos = np.zeros(self.__joint_num)
        
        # target position shared between processes
        self.__target_lock = threading.Lock()
        self.__current_target = multiprocessing.Array('d', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.__target_reached = multiprocessing.Value('b', True)
        
        # control process
        self.__control_process = None
        self.__stop_event = multiprocessing.Event()
        
        # finish log
        print("joint track node init finished")

    def __joint_state_callback(self, msg: JointState):
        # update state
        with self.__pos_lock:
            self.__cur_pos = np.array(msg.position)
    
    @staticmethod
    def __control_process_func(current_target, target_reached, stop_event, 
                              delta_lr_max, delta_dm_max, deadzone_thresh, target_change_thresh):
        """Control process that runs independently and publishes ROS commands"""
        # Initialize ROS publisher in the subprocess
        joint_ctrl_pub = rospy.Publisher(
            '/xtopic_arm/joints_cmd',
            XmsgArmJointParamList,
            queue_size=10,
        )
        
        # Initialize joint state subscriber for this process
        cur_pos = np.zeros(6)
        pos_lock = threading.Lock()
        
        def joint_state_callback(msg: JointState):
            nonlocal cur_pos
            with pos_lock:
                cur_pos[:] = msg.position
        
        joint_state_sub = rospy.Subscriber(
            '/xtopic_arm/joint_states',
            JointState,
            joint_state_callback,
        )
        
        # Create control message template
        ctrl_msg = XmsgArmJointParamList()
        for _ in range(6):
            param = XmsgArmJointParam()
            param.mode = "position_mode"
            param.position = 0.0
            param.velocity = 0.0
            param.effort = 0.0
            param.extra_param = json.dumps({"braking_state": False, "mit_kp": 4.0, "mit_kd": 0.5})
            ctrl_msg.joints.append(param)
        
        rate = rospy.Rate(50.0)  # 50Hz control frequency
        
        while not stop_event.is_set() and not rospy.is_shutdown():
            # Get current target and position
            target_pos = np.array(current_target[:])
            with pos_lock:
                current_pos = cur_pos.copy()
            
            # Calculate delta position
            delta_pos = target_pos - current_pos
            delta_pos_norm = np.linalg.norm(delta_pos)
            
            # Check if target is reached
            if delta_pos_norm < target_change_thresh:
                target_reached.value = True
            else:
                target_reached.value = False
            
            # Calculate incremental position with clipping
            inc_pos = copy.deepcopy(delta_pos)
            
            # Clip first 3 joints
            inc_pos[:3] = np.clip(
                inc_pos[:3],
                -delta_lr_max,
                delta_lr_max,
            )
            
            # Clip last 3 joints
            inc_pos[3:] = np.clip(
                inc_pos[3:],
                -delta_dm_max,
                delta_dm_max,
            )
            
            control_pos = current_pos + inc_pos
            
            # Apply deadzone for first 3 joints
            for i in range(3):
                if abs(delta_pos[i]) < deadzone_thresh[i]:
                    control_pos[i] = target_pos[i]
            
            print(f"control_pos: {control_pos}")
            
            # Update control message and publish
            for i in range(6):
                ctrl_msg.joints[i].position = control_pos[i]
            joint_ctrl_pub.publish(ctrl_msg)
            
            rate.sleep()
    
    def start_control_process(self):
        """Start the control subprocess"""
        if self.__control_process is not None and self.__control_process.is_alive():
            print("Control process is already running")
            return
        
        self.__stop_event.clear()
        self.__control_process = multiprocessing.Process(
            target=self.__control_process_func,
            args=(
                self.__current_target,
                self.__target_reached,
                self.__stop_event,
                self.__delta_lr_max,
                self.__delta_dm_max,
                self.__deadzone_thresh,
                self.__target_change_thresh
            )
        )
        self.__control_process.start()
        print("Control process started")
    
    def stop_control_process(self):
        """Stop the control subprocess"""
        if self.__control_process is not None and self.__control_process.is_alive():
            self.__stop_event.set()
            self.__control_process.join(timeout=2.0)
            if self.__control_process.is_alive():
                self.__control_process.terminate()
                self.__control_process.join()
            print("Control process stopped")
    
    def set_target_position(self, target_pos):
        """Set global target position"""
        if len(target_pos) != 6:
            raise ValueError("Target position must have 6 elements")
        
        # Update shared target position
        for i in range(6):
            self.__current_target[i] = target_pos[i]
        
        # Reset target reached flag
        self.__target_reached.value = False
        print(f"Target position set to: {target_pos}")
    
    def is_target_reached(self):
        """Check if current target position is reached"""
        return self.__target_reached.value

    def __del__(self):
        """Destructor to ensure control process is stopped"""
        self.stop_control_process()


def main():
    # Define target positions
    target_positions = [
        np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([0.5, 0.523598775598, 2.09439265359, 1.57, -1.0472, 0.0]),
        np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([-0.5, 0.523598775598, 2.09439265359, -1.57, -1.0472, 0.0]),
    ]
    
    # Initialize joint tracker
    joint_track = JointTrack()
    
    # Start control process
    joint_track.start_control_process()
    
    try:
        current_target_idx = 0
        
        # Set initial target
        joint_track.set_target_position(target_positions[current_target_idx])
        
        while not rospy.is_shutdown():
            # Check if current target is reached
            if joint_track.is_target_reached():
                print(f"Target {current_target_idx} reached!")
                
                # Move to next target
                current_target_idx = (current_target_idx + 1) % len(target_positions)
                joint_track.set_target_position(target_positions[current_target_idx])
                print(f"Moving to target {current_target_idx}")
                
                # Wait a bit before checking again
                time.sleep(1.0)
            else:
                # Check every 100ms
                time.sleep(0.1)
                
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        # Stop control process
        joint_track.stop_control_process()
        print("Program terminated")


if __name__ == '__main__':
    main()

