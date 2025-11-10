#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-05-08
# 
# AhexArm Controller with N-th Order Differential Control
# 
# This module provides a high-performance robotic arm controller
# with configurable n-th order differential constraints for
# smooth and precise motion control.
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
    """
    High-performance robotic arm controller with n-th order differential control.
    
    This class implements a sophisticated control system that can constrain position,
    velocity, acceleration, jerk, and higher-order derivatives using saturation functions.
    The control algorithm ensures smooth motion without abrupt changes in any derivative.
    
    Features:
    - Configurable n-th order differential control (n=0 to any order)
    - Thread-safe operation with real-time control loop
    - ROS integration for joint state feedback and command publishing
    - Automatic driver management
    - Flexible parameter configuration
    
    Control Modes:
    - n=0: Direct target tracking (no constraints)
    - n=1: Position-only control
    - n=2: Position + velocity control (PD-like)
    - n=3: Position + velocity + acceleration control
    - n=4: Position + velocity + acceleration + jerk control (default)
    - n>=5: Higher-order control for ultra-smooth motion
    """

    def __init__(self, control_frequency=50.0, n_derivatives=4):
        """
        Initialize AhexArm controller
        
        Args:
            control_frequency (float): Control loop frequency in Hz (default: 50.0)
            n_derivatives (int): Number of derivatives to control (default: 4)
                                n=0: no constraints (direct target tracking)
                                n=1: position constraint only
                                n=2: position + velocity constraints
                                n=3: position + velocity + acceleration constraints
                                n=4: position + velocity + acceleration + jerk constraints (matches original implementation)
                                etc.
        """
        # Store control parameters
        self.__control_frequency = control_frequency
        self.__n_derivatives = max(0, n_derivatives)  # Allow n=0 for no constraints
        
        # Check and start arm driver if needed
        self._ensure_arm_driver_running()
        
        # Wait for driver to initialize
        time.sleep(2.0)
        
        # Initialize ROS node and subscriber
        rospy.init_node("joint_track", anonymous=True)
        self.__joint_state_sub = rospy.Subscriber(
            '/xtopic_arm/joint_states',
            JointState,
            self.__joint_state_callback,
        )

        # Control parameters
        self.__target_change_thresh = 0.2
        
        self.__joint_num = 6
        self.__pos_lock = threading.Lock()
        self.__cur_pos = np.zeros(self.__joint_num)
        self.__default_position = np.array([0.0, -1.5, 3.0, 0.0, 0.0, 0.0])
        self.__vel_lock = threading.Lock()
        self.__cur_vel = np.zeros(self.__joint_num)
        # Target position and synchronization
        self.__target_lock = threading.Lock()
        self.__current_target = self.__default_position.copy()
        self.__target_reached = threading.Event()
        
        # Differential control parameters - maximum magnitudes for each derivative order
        # Index 0: position, 1: velocity, 2: acceleration, 3: jerk, etc.
        if self.__n_derivatives > 0:
            self.__max_derivatives = np.ones(self.__n_derivatives)  # Default: all set to 1.0
        else:
            self.__max_derivatives = np.array([])  # No constraints for n=0
        
        # Control gains for differential equations (k_0 for position error, k_1 for velocity error, etc.)
        if self.__n_derivatives > 1:
            self.__control_gains = np.ones(self.__n_derivatives - 1) / (1.0 / self.__control_frequency)
        else:
            self.__control_gains = np.array([])  # No gains needed for n<=1
        
        # Control state variables - store all derivatives up to (n-1)th order for each joint
        # derivatives[0] = velocity, derivatives[1] = acceleration, derivatives[2] = jerk, etc.
        self.__control_state_lock = threading.Lock()
        if self.__n_derivatives > 1:
            self.__current_derivatives = np.zeros((self.__n_derivatives - 1, self.__joint_num))
        else:
            self.__current_derivatives = np.zeros((0, self.__joint_num))  # Empty array for n<=1
        
        # Control thread management
        self.__control_thread = None
        self.__stop_event = threading.Event()
        
        # ROS publisher
        self.__joint_ctrl_pub = rospy.Publisher(
            '/xtopic_arm/joints_cmd',
            XmsgArmJointParamList,
            queue_size=10,
        )

        # Start control process and move to default position
        self.start_control_process()
        print("Moving to default position...")
        self.set_target_position(self.__default_position)
        print("AhexArm controller initialized successfully")

    def __joint_state_callback(self, msg: JointState):
        """Callback to update current joint positions from ROS topic"""
        with self.__pos_lock:
            self.__cur_pos = np.array(msg.position)
            self.__cur_vel = np.array(msg.velocity)
    
    def _sat(self, u, m):
        """
        Saturation function that clips values to [-m, m] range
        
        Args:
            u: Input value or array
            m: Maximum magnitude (positive value)
            
        Returns:
            Clipped value(s) within [-m, m] range
        """
        return np.clip(u, -m, m)
    
    def _compute_differential_control(self, current_pos, current_vel, target_pos, dt):
        """
        Compute control position using n-th order differential equations with constraints
        
        General n-th order differential control algorithm:
        1. Calculate position error: e_0 = x_target - x_current
        2. For i = 0 to n-2:
           - Calculate desired (i+1)-th derivative: d_{i+1} = sat(k_i * (d_desired_i - d_current_i), max_i+1)
        3. Calculate highest order derivative (n-1): d_{n-1} = sat(k_{n-2} * (d_desired_{n-2} - d_current_{n-2}), max_{n-1})
        4. Update all derivatives using Taylor expansion
        5. Update position using Taylor expansion with all derivatives
        
        Args:
            current_pos: Current joint positions
            current_vel: Current joint velocities
            target_pos: Target joint positions  
            dt: Time step
            
        Returns:
            control_pos: Next control position
        """
        with self.__control_state_lock:
            # Copy current derivatives (velocity, acceleration, jerk, etc.)
            current_derivs = self.__current_derivatives.copy()
        
        # Step 1: Calculate position error
        position_error = target_pos - current_pos
        
        # Initialize arrays for desired derivatives and new derivatives
        if self.__n_derivatives > 1:
            desired_derivs = np.zeros((self.__n_derivatives - 1, self.__joint_num))
            new_derivs = np.zeros((self.__n_derivatives - 1, self.__joint_num))
        else:
            desired_derivs = np.zeros((0, self.__joint_num))
            new_derivs = np.zeros((0, self.__joint_num))
        
        # Step 2: Calculate desired derivatives iteratively
        # For n=0: no constraints (direct target tracking)
        if self.__n_derivatives == 0:
            # Direct target tracking without any constraints
            control_pos = target_pos
        # For n=1: only position control
        elif self.__n_derivatives == 1:
            # Direct position control with saturation
            control_pos = self._sat(current_pos + position_error, self.__max_derivatives[0])
        else:
            # For n>=2: use derivative control
            # First desired derivative (velocity) based on position error
            desired_derivs[0] = self._sat(self.__control_gains[0] * position_error, self.__max_derivatives[1])
            
            # Calculate subsequent desired derivatives
            for i in range(1, self.__n_derivatives - 1):
                if i < len(self.__control_gains):
                    error = desired_derivs[i-1] - current_derivs[i-1]
                    desired_derivs[i] = self._sat(self.__control_gains[i] * error, self.__max_derivatives[i+1])
            
            # Step 3: Calculate highest order derivative (control input)
            highest_order_idx = self.__n_derivatives - 2
            if highest_order_idx < len(self.__control_gains):
                if highest_order_idx > 0:
                    error = desired_derivs[highest_order_idx-1] - current_derivs[highest_order_idx-1]
                    highest_deriv = self._sat(self.__control_gains[highest_order_idx] * error, 
                                            self.__max_derivatives[highest_order_idx+1])
                else:
                    highest_deriv = desired_derivs[0]
            else:
                highest_deriv = np.zeros(self.__joint_num)
            
            # Step 4: Update all derivatives using integration
            # Update from highest order to lowest order
            for i in range(self.__n_derivatives - 2, -1, -1):
                if i == self.__n_derivatives - 2:
                    # Highest order derivative update
                    new_derivs[i] = self._sat(current_derivs[i] + highest_deriv * dt, 
                                            self.__max_derivatives[i+1])
                else:
                    # Lower order derivatives update using Taylor expansion
                    update = current_derivs[i+1] * dt
                    if i+2 < self.__n_derivatives - 1:
                        update += 0.5 * current_derivs[i+2] * (dt**2)
                    if i+3 < self.__n_derivatives - 1:
                        update += (1.0/6.0) * current_derivs[i+3] * (dt**3)
                    
                    new_derivs[i] = self._sat(current_derivs[i] + update, self.__max_derivatives[i+1])
            
            # Step 5: Update position using Taylor expansion
            position_update = np.zeros(self.__joint_num)
            for i in range(self.__n_derivatives - 1):
                factorial = np.math.factorial(i + 1)
                position_update += new_derivs[i] * (dt**(i+1)) / factorial
            
            control_pos = self._sat(current_pos + position_update, self.__max_derivatives[0])
        
        # Update internal state variables
        with self.__control_state_lock:
            if self.__n_derivatives > 1:
                self.__current_derivatives[:] = new_derivs
        
        return control_pos
    
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
        
        rate = rospy.Rate(self.__control_frequency)  # Configurable control frequency
        dt = 1.0 / self.__control_frequency  # Time step
        
        while not self.__stop_event.is_set() and not rospy.is_shutdown():
            # Get current target and position
            with self.__target_lock:
                target_pos = self.__current_target.copy()
            
            with self.__pos_lock:
                current_pos = self.__cur_pos.copy()
            with self.__vel_lock:
                current_vel = self.__cur_vel.copy()
            
            # Calculate delta position for target reached check
            delta_pos = target_pos - current_pos
            delta_pos_norm = np.linalg.norm(delta_pos)
            
            # Check if target is reached
            if delta_pos_norm < self.__target_change_thresh:
                self.__target_reached.set()
            else:
                self.__target_reached.clear()
            
            # Use differential control algorithm to compute control position
            control_pos = self._compute_differential_control(current_pos, current_vel, target_pos, dt)
            
            # Update control message and publish
            for i in range(6):
                ctrl_msg.joints[i].position = control_pos[i]
            
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
    
    def set_target_position(self, target_pos, wait=False, timeout=10.0):
        """Set global target position
        
        The differential control algorithm will smoothly adjust velocity and acceleration
        to transition to the new target position without any abrupt changes.
        
        Args:
            target_pos: Target joint positions (6 elements)
            wait (bool): Whether to wait for target to be reached (default: False)
            timeout (float): Maximum time to wait if wait=True (default: 10.0)
            
        Returns:
            bool: If wait=True, returns True if target reached within timeout, False otherwise.
                  If wait=False, always returns True.
        """
        if len(target_pos) != 6:
            raise ValueError("Target position must have 6 elements")
        
        # Update shared target position
        with self.__target_lock:
            self.__current_target[:] = target_pos
        
        # Reset target reached flag
        self.__target_reached.clear()
        print(f"Target position set to: {target_pos}")
        
        # Optionally wait for target to be reached
        if wait:
            return self.__target_reached.wait(timeout)
        return True
    
    def is_target_reached(self):
        """Check if current target position is reached"""
        return self.__target_reached.is_set()
    
    def get_current_position(self):
        """Get current joint positions"""
        with self.__pos_lock:
            return self.__cur_pos.copy()
    
    def get_current_velocity(self):
        """Get current joint velocities"""
        with self.__vel_lock:
            return self.__cur_vel.copy()
    
    def get_default_position(self):
        """Get default joint positions"""
        return self.__default_position.copy()
    
    def get_control_frequency(self):
        """Get current control frequency in Hz"""
        return self.__control_frequency
    
    def set_control_frequency(self, frequency):
        """Set control frequency in Hz
        
        Note: This will take effect when control thread is restarted
        
        Args:
            frequency (float): New control frequency in Hz
        """
        if frequency <= 0:
            raise ValueError("Control frequency must be positive")
        self.__control_frequency = frequency
        
        # Update gains based on new frequency (only if we have gains)
        if self.__n_derivatives > 1:
            self.__control_gains = np.ones(self.__n_derivatives - 1) / (1.0 / frequency)
    
    def set_derivative_limits(self, limits):
        """Set maximum limits for all derivative orders
        
        Args:
            limits (list or np.array): Maximum limits for each derivative order
                                     [position_max, velocity_max, acceleration_max, jerk_max, ...]
                                     Length should match n_derivatives
        """
        if self.__n_derivatives == 0:
            print("Warning: No derivative limits can be set for n_derivatives=0 (no constraints mode)")
            return
            
        if len(limits) != self.__n_derivatives:
            raise ValueError(f"Limits array length ({len(limits)}) must match n_derivatives ({self.__n_derivatives})")
        
        for i, limit in enumerate(limits):
            if limit > 0:
                self.__max_derivatives[i] = limit
    
    def set_control_gains(self, gains):
        """Set control gains for differential equations
        
        Args:
            gains (list or np.array): Control gains for each derivative error
                                    [k_0, k_1, k_2, ...] where k_0 is position error gain
                                    Length should be (n_derivatives - 1)
        """
        if self.__n_derivatives <= 1:
            print("Warning: No control gains can be set for n_derivatives<=1 (no derivative control)")
            return
            
        if len(gains) != self.__n_derivatives - 1:
            raise ValueError(f"Gains array length ({len(gains)}) must be (n_derivatives - 1) = {self.__n_derivatives - 1}")
        
        for i, gain in enumerate(gains):
            if gain > 0:
                self.__control_gains[i] = gain
    
    def get_derivative_limits(self):
        """Get current derivative limits
        
        Returns:
            np.array: Array of maximum limits for each derivative order
        """
        return self.__max_derivatives.copy()
    
    def get_control_gains(self):
        """Get current control gains
        
        Returns:
            np.array: Array of control gains for each derivative error
        """
        return self.__control_gains.copy()
    
    def get_n_derivatives(self):
        """Get number of derivatives being controlled
        
        Returns:
            int: Number of derivatives (n_derivatives)
        """
        return self.__n_derivatives
    
    def get_current_derivatives(self):
        """Get current derivative states (velocity, acceleration, etc.)
        
        Returns:
            np.array: Array of shape (n_derivatives-1, joint_num) containing current derivatives
                     [0]: velocity, [1]: acceleration, [2]: jerk, etc.
        """
        with self.__control_state_lock:
            return self.__current_derivatives.copy()
    
    def get_derivative_names(self):
        """Get names of all derivatives being controlled
        
        Returns:
            list: List of derivative names
        """
        names = ['position']
        derivative_names = ['velocity', 'acceleration', 'jerk', 'snap', 'crackle', 'pop']
        
        for i in range(1, self.__n_derivatives):
            if i <= len(derivative_names):
                names.append(derivative_names[i-1])
            else:
                names.append(f'derivative_{i}')
        
        return names
    
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


if __name__ == '__main__':
    print("AhexArm control module - import this module to use the AhexArm class")
    print("Example usage:")
    print("  from arm_control_thread import AhexArm")
    print("  arm = AhexArm(control_frequency=50.0, n_derivatives=4)")
    print("  arm.set_target_position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])")

