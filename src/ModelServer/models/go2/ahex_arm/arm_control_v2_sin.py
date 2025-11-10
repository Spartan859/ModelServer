#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-05-08
################################################################

import copy
import json
import os
import subprocess
import threading
import time
import numpy as np

import rospy
from sensor_msgs.msg import JointState
from xpkg_arm_msgs.msg import XmsgArmJointParam
from xpkg_arm_msgs.msg import XmsgArmJointParamList


class AhexArm:

    def __init__(self):
        # Check and start arm driver if needed
        self._ensure_arm_driver_running()
        
        # Wait a moment for driver to initialize
        time.sleep(2.0)
        ### ros node
        rospy.init_node("joint_track", anonymous=True)
        
        ### subscriber
        self.__joint_state_sub = rospy.Subscriber(
            '/xtopic_arm/joint_states',
            JointState,
            self.__joint_state_callback,
        )

        ### variable
        self.__delta_lr_max = 0.10 * 1 * 8
        self.__delta_dm_max = 0.05 * 4 * 8
        # target
        self.__target_change_thresh = 0.2
        # deadzone - for each joint
        self.__deadzone_thresh = np.array([0.02, 0.02, 0.02, 0.02, 0.02, 0.02])
        
        self.__joint_num = 6
        # state
        self.__pos_lock = threading.Lock()
        self.__cur_pos = np.zeros(self.__joint_num)
        
        # default position
        self.__default_position = np.array([0.0, -1.5, 3.0, 0.0, 0.0, 0.0])
        
        # target position shared between threads
        self.__target_lock = threading.Lock()
        self.__current_target = self.__default_position.copy()
        self.__target_reached = threading.Event()
        
        # control thread
        self.__control_thread = None
        self.__stop_event = threading.Event()
        
        # ROS publisher (shared with control thread)
        self.__joint_ctrl_pub = rospy.Publisher(
            '/xtopic_arm/joints_cmd',
            XmsgArmJointParamList,
            queue_size=10,
        )

        self.start_control_process()
        
        # Move to default position on startup
        print("Moving to default position...")
        success = self.set_target_position(self.__default_position, timeout=20.0)
        if success:
            print("Successfully moved to default position")
        else:
            print("Warning: Failed to reach default position within timeout")
        
        # finish log
        print("joint track node init finished")

    def __joint_state_callback(self, msg: JointState):
        # update state
        with self.__pos_lock:
            self.__cur_pos = np.array(msg.position)
    
    def __control_thread_func(self):
        """Control thread that runs independently and publishes ROS commands"""
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
        
        while not self.__stop_event.is_set() and not rospy.is_shutdown():
            # Get current target and position
            with self.__target_lock:
                target_pos = self.__current_target.copy()
            
            with self.__pos_lock:
                current_pos = self.__cur_pos.copy()
            
            # Calculate delta position
            delta_pos = target_pos - current_pos
            delta_pos_norm = np.linalg.norm(delta_pos)
            
            # Check if target is reached
            if delta_pos_norm < self.__target_change_thresh:
                self.__target_reached.set()
            else:
                self.__target_reached.clear()
            
            # Calculate incremental position with clipping
            inc_pos = copy.deepcopy(delta_pos)
            
            # Clip first 3 joints
            inc_pos[:3] = np.clip(
                inc_pos[:3],
                -self.__delta_lr_max,
                self.__delta_lr_max,
            )
            
            # Clip last 3 joints
            inc_pos[3:] = np.clip(
                inc_pos[3:],
                -self.__delta_dm_max,
                self.__delta_dm_max,
            )
            
            control_pos = current_pos + inc_pos
            
            # Apply deadzone for first 3 joints
            for i in range(3):
                if abs(delta_pos[i]) < self.__deadzone_thresh[i]:
                    control_pos[i] = target_pos[i]
            
            # print(f"control_pos: {control_pos}")
            
            # Update control message and publish
            for i in range(6):
                ctrl_msg.joints[i].position = control_pos[i]
            
            # Debug: print message info
            # print(f"Publishing to /xtopic_arm/joints_cmd, num_subscribers: {self.__joint_ctrl_pub.get_num_connections()}")
            
            self.__joint_ctrl_pub.publish(ctrl_msg)
            
            rate.sleep()
    
    def start_control_process(self):
        """Start the control thread"""
        if self.__control_thread is not None and self.__control_thread.is_alive():
            print("Control thread is already running")
            return
        
        self.__stop_event.clear()
        self.__control_thread = threading.Thread(target=self.__control_thread_func)
        self.__control_thread.daemon = True
        self.__control_thread.start()
        print("Control thread started")
    
    def stop_control_process(self):
        """Stop the control thread"""
        if self.__control_thread is not None and self.__control_thread.is_alive():
            self.__stop_event.set()
            self.__control_thread.join(timeout=2.0)
            print("Control thread stopped")
    
    def set_target_position(self, target_pos, timeout=10.0):
        """Set global target position and wait for it to be reached
        
        Args:
            target_pos: Target joint positions (6 elements)
            timeout: Maximum time to wait in seconds (default: 10.0)
            
        Returns:
            bool: True if target reached within timeout, False otherwise
        """
        if len(target_pos) != 6:
            raise ValueError("Target position must have 6 elements")
        
        # Update shared target position
        with self.__target_lock:
            self.__current_target[:] = target_pos
        
        # Reset target reached flag
        self.__target_reached.clear()
        print(f"Target position set to: {target_pos}")
        
        # Wait for target to be reached with timeout
        return self.__target_reached.wait(timeout)
    
    def is_target_reached(self):
        """Check if current target position is reached"""
        return self.__target_reached.is_set()
    
    def set_target_position_smooth(self, target_pos, total_time=10.0, time_step=0.02):
        """Set target position with smooth non-linear trajectory
        
        使用平滑的非线性轨迹将机械臂从当前位置移动到目标位置。
        速度从0起步，中点速度最大，到达目标位置时速度降为0。
        
        Args:
            target_pos: 目标关节位置 (6个元素)
            total_time: 总运动时间，单位秒 (默认: 10.0)
            time_step: 时间步长，单位秒 (默认: 0.02，即50Hz)
            
        Returns:
            bool: 如果成功启动平滑运动返回True，否则返回False
        """
        if len(target_pos) != 6:
            raise ValueError("Target position must have 6 elements")
        
        # 获取当前位置作为起始位置
        start_pos = self.get_current_position()
        target_pos = np.array(target_pos)
        
        print(f"Starting smooth motion from {start_pos} to {target_pos} over {total_time}s")
        
        # 启动平滑运动线程
        smooth_thread = threading.Thread(
            target=self._smooth_motion_thread,
            args=(start_pos, target_pos, total_time, time_step),
            daemon=True
        )
        smooth_thread.start()
        
        return True
    
    def _smooth_motion_thread(self, start_pos, target_pos, total_time, time_step):
        """执行平滑运动的线程函数
        
        使用正弦函数的一半周期来生成平滑的S曲线轨迹。
        这样可以确保起始和结束速度都为0，中点速度最大。
        """
        start_pos = np.array(start_pos)
        target_pos = np.array(target_pos)
        position_delta = target_pos - start_pos
        
        # 计算总的时间步数
        num_steps = int(total_time / time_step)
        
        print(f"Executing smooth motion with {num_steps} steps, step size: {time_step}s")
        
        for step in range(num_steps + 1):
            # 计算当前时间比例 (0 到 1)
            t_ratio = step / num_steps
            
            # 使用正弦函数的一半周期生成S曲线
            # sin(π * t) 在 t=0 时导数为0，t=0.5时导数最大，t=1时导数又为0
            # 积分后得到位置曲线：(1 - cos(π * t)) / 2
            smooth_ratio = (1 - np.cos(np.pi * t_ratio)) / 2
            
            # 计算当前的中间目标位置
            current_target = start_pos + position_delta * smooth_ratio
            
            # 更新全局目标位置
            with self.__target_lock:
                self.__current_target[:] = current_target
            
            # 每10步打印一次进度信息
            if step % 10 == 0 or step == num_steps:
                progress = step / num_steps * 100
                print(f"Smooth motion progress: {progress:.1f}% - Target: {current_target}")
            
            # 如果不是最后一步，等待下一个时间步
            if step < num_steps:
                time.sleep(time_step)
        
        print("Smooth motion completed!")
        
        # 确保最终位置精确设置为目标位置
        with self.__target_lock:
            self.__current_target[:] = target_pos
    
    def get_current_position(self):
        """Get current joint positions"""
        with self.__pos_lock:
            return self.__cur_pos.copy()
    
    def get_default_position(self):
        """Get default joint positions"""
        return self.__default_position.copy()
    
    def _is_arm_driver_running(self):
        """Check if arm driver is already running by checking for specific ROS topics"""
        try:
            # Check if the joint state topic is available
            topics = rospy.get_published_topics()
            for topic, msg_type in topics:
                if topic == '/xtopic_arm/joint_states':
                    return True
            return False
        except Exception as e:
            print(f"Error checking ROS topics: {e}")
            return False
    
    def _start_arm_driver(self):
        """Start the arm driver using setup_arm.sh script"""
        try:
            # Get the directory where this script is located
            current_dir = os.path.dirname(os.path.abspath(__file__))
            setup_script = os.path.join(current_dir, 'setup_arm.sh')
            
            if not os.path.exists(setup_script):
                raise FileNotFoundError(f"Setup script not found at: {setup_script}")
            
            print(f"Starting arm driver using: {setup_script}")
            
            # Make the script executable
            os.chmod(setup_script, 0o755)
            
            # Start the setup script in background
            # Use nohup to prevent the process from being killed when parent exits
            cmd = f"nohup bash {setup_script} > /tmp/arm_setup.log 2>&1 &"
            subprocess.run(cmd, shell=True, check=True)
            
            print("Arm driver startup initiated. Waiting for initialization...")
            
            # Wait for the driver to start (check for topic availability)
            max_wait_time = 30.0  # 30 seconds timeout
            wait_interval = 1.0
            elapsed_time = 0.0
            
            while elapsed_time < max_wait_time:
                if self._is_arm_driver_running():
                    print("Arm driver is now running!")
                    return True
                
                time.sleep(wait_interval)
                elapsed_time += wait_interval
                print(f"Waiting for arm driver... ({elapsed_time:.1f}s)")
            
            print("Warning: Arm driver may not have started properly within timeout")
            return False
            
        except Exception as e:
            print(f"Error starting arm driver: {e}")
            return False
    
    def _ensure_arm_driver_running(self):
        """Ensure arm driver is running, start it if necessary"""
        print("Checking if arm driver is running...")
        
        if self._is_arm_driver_running():
            print("Arm driver is already running.")
            return True
        
        print("Arm driver not detected. Starting driver...")
        return self._start_arm_driver()

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
    joint_track = AhexArm()
    
    try:
        current_target_idx = 0
        
        while not rospy.is_shutdown():
            print(f"\n=== Moving to target {current_target_idx} ===")
            
            # 可以选择使用平滑运动或普通运动
            use_smooth_motion = True  # 设置为True使用平滑运动，False使用普通运动
            
            if use_smooth_motion:
                # 使用平滑运动，10秒完成运动
                print("Using smooth motion...")
                joint_track.set_target_position_smooth(
                    target_positions[current_target_idx], 
                    total_time=8.0  # 8秒完成运动
                )
                # 等待运动完成 (多等待2秒作为缓冲)
                time.sleep(10.0)
            else:
                # 使用原来的运动方式
                print("Using standard motion...")
                success = joint_track.set_target_position(target_positions[current_target_idx], timeout=15.0)
                
                if success:
                    print(f"Target {current_target_idx} reached successfully!")
                else:
                    print(f"Target {current_target_idx} timeout - moving to next target anyway")
            
            # Move to next target
            current_target_idx = (current_target_idx + 1) % len(target_positions)
            
            # Wait a bit before moving to next target
            time.sleep(2.0)
                
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        # Stop control process
        joint_track.stop_control_process()
        print("Program terminated")


if __name__ == '__main__':
    main()

