#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-05-08
# 
# AhexArm Hardware Interface
# 
# This module provides hardware-specific implementation for
# the Ahex robotic arm system using ROS communication.
################################################################

import copy
import json
import os
import subprocess
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from xpkg_arm_msgs.msg import XmsgArmJointParam
from xpkg_arm_msgs.msg import XmsgArmJointParamList

from .pid_controller import HardwareInterface


class AhexArmHardware(Node, HardwareInterface):
    """
    Hardware interface implementation for Ahex robotic arm.
    
    This class handles all hardware-specific operations including:
    - ROS communication setup
    - Joint state monitoring
    - Position command transmission
    - Driver management
    - Hardware status monitoring
    """

    def __init__(self):
        """Initialize Ahex arm hardware interface"""
        # Initialize rclpy if not already initialized
        if not rclpy.ok():
            rclpy.init()
        
        # Initialize ROS2 node first
        super().__init__('ahex_arm_hardware')
        
        print("Initializing AhexArm hardware interface...")
        
        # Hardware configuration
        self.__joint_num = 6
        self.__default_position = np.array([0.0, -1.5, 3.0, 0.0, 0.0, 0.0])
        
        # State management
        self.__pos_lock = threading.Lock()
        self.__cur_pos = np.zeros(self.__joint_num)
        self.__hardware_ready = False
        
        # Check and start arm driver if needed
        self._ensure_arm_driver_running()
        
        # Wait for driver to initialize
        time.sleep(2.0)
        
        # Create subscriber
        self.__joint_state_sub = self.create_subscription(
            JointState,
            '/xtopic_arm/joint_states',
            self.__joint_state_callback,
            10
        )

        # Create publisher
        self.__joint_ctrl_pub = self.create_publisher(
            XmsgArmJointParamList,
            '/xtopic_arm/joints_cmd',
            10
        )
        
        # Wait for initial joint state
        print("Waiting for joint state feedback...")
        start_time = time.time()
        while not self.__hardware_ready and (time.time() - start_time) < 10.0:
            self.spin_once(timeout_sec=0.1)
        
        if self.__hardware_ready:
            print("AhexArm hardware interface initialized successfully")
        else:
            print("Warning: No joint state feedback received within timeout")
            # Set hardware as ready anyway to allow operation
            self.__hardware_ready = True

    def __joint_state_callback(self, msg: JointState):
        """Callback to update current joint positions from ROS topic"""
        with self.__pos_lock:
            self.__cur_pos = np.array(msg.position)
            self.__hardware_ready = True

    def get_current_position(self):
        """Get current joint positions
        
        Returns:
            np.array: Current joint positions (6 elements)
        """
        with self.__pos_lock:
            return self.__cur_pos.copy()

    def send_position_command(self, positions):
        """Send position command to hardware
        
        Args:
            positions (np.array): Target joint positions (6 elements)
        """
        if len(positions) != self.__joint_num:
            raise ValueError(f"Position command must have {self.__joint_num} elements")
        
        # Create control message
        ctrl_msg = XmsgArmJointParamList()
        for i in range(self.__joint_num):
            param = XmsgArmJointParam()
            param.mode = "position_mode"
            param.position = float(positions[i])
            param.velocity = 0.0
            param.effort = 0.0
            param.extra_param = json.dumps({"braking_state": False, "mit_kp": 4.0, "mit_kd": 0.5})
            ctrl_msg.joints.append(param)
        
        # Publish command
        self.__joint_ctrl_pub.publish(ctrl_msg)

    def get_joint_count(self):
        """Get number of joints
        
        Returns:
            int: Number of joints (6 for Ahex arm)
        """
        return self.__joint_num

    def is_ready(self):
        """Check if hardware is ready
        
        Returns:
            bool: True if hardware is ready
        """
        return self.__hardware_ready
    
    def get_default_position(self):
        """Get default joint positions
        
        Returns:
            np.array: Default joint positions
        """
        return self.__default_position.copy()
    
    def _is_arm_driver_running(self):
        """Check if arm driver is already running by checking for specific ROS topics"""
        try:
            # Check if the joint state topic is available
            topic_names_and_types = self.get_topic_names_and_types()
            for topic_name, topic_types in topic_names_and_types:
                if topic_name == '/xtopic_arm/joint_states':
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
    
    def spin_once(self, timeout_sec=None):
        """Process ROS2 callbacks once"""
        rclpy.spin_once(self, timeout_sec=timeout_sec)
    
    def destroy_node_safe(self):
        """Safely destroy the ROS2 node"""
        try:
            self.destroy_node()
        except Exception as e:
            print(f"Error destroying node: {e}")


if __name__ == '__main__':
    print("AhexArmHardware module - import this module to use the AhexArmHardware class")
    print("Example usage:")
    print("  import rclpy")
    print("  from ahex_arm_hardware import AhexArmHardware")
    print("  from differential_controller import DifferentialController")
    print("  rclpy.init()")
    print("  hardware = AhexArmHardware()")
    print("  controller = DifferentialController(hardware)")
    print("  controller.start_control_process()")
    print("  rclpy.shutdown()")
