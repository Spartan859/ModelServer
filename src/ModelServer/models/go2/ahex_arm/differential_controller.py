#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-05-08
# 
# Universal N-th Order Differential Controller
# 
# This module provides a hardware-agnostic differential control
# algorithm that can be applied to any robotic system.
################################################################

import threading
import time
import numpy as np
from abc import ABC, abstractmethod


class HardwareInterface(ABC):
    """Abstract base class for hardware interface"""
    
    @abstractmethod
    def get_current_position(self):
        """Get current joint positions
        
        Returns:
            np.array: Current joint positions
        """
        pass
    
    @abstractmethod
    def send_position_command(self, positions):
        """Send position command to hardware
        
        Args:
            positions (np.array): Target joint positions
        """
        pass
    
    @abstractmethod
    def get_joint_count(self):
        """Get number of joints
        
        Returns:
            int: Number of joints
        """
        pass
    
    @abstractmethod
    def is_ready(self):
        """Check if hardware is ready
        
        Returns:
            bool: True if hardware is ready
        """
        pass


class DifferentialController:
    """
    Universal N-th order differential controller for robotic systems.
    
    This class implements a sophisticated control system that can constrain position,
    velocity, acceleration, jerk, and higher-order derivatives using saturation functions.
    The control algorithm ensures smooth motion without abrupt changes in any derivative.
    
    Features:
    - Hardware-agnostic design (works with any HardwareInterface implementation)
    - Configurable n-th order differential control (n=0 to any order)
    - Thread-safe operation with real-time control loop
    - Flexible parameter configuration
    - Target tracking with customizable thresholds
    
    Control Modes:
    - n=0: Direct target tracking (no constraints)
    - n=1: Position-only control
    - n=2: Position + velocity control (PD-like)
    - n=3: Position + velocity + acceleration control
    - n=4: Position + velocity + acceleration + jerk control (default)
    - n>=5: Higher-order control for ultra-smooth motion
    """

    def __init__(self, hardware_interface, control_frequency=50.0, n_derivatives=4):
        """
        Initialize DifferentialController
        
        Args:
            hardware_interface (HardwareInterface): Hardware interface implementation
            control_frequency (float): Control loop frequency in Hz (default: 50.0)
            n_derivatives (int): Number of derivatives to control (default: 4)
                                n=0: no constraints (direct target tracking)
                                n=1: position constraint only
                                n=2: position + velocity constraints
                                n=3: position + velocity + acceleration constraints
                                n=4: position + velocity + acceleration + jerk constraints
                                etc.
        """
        if not isinstance(hardware_interface, HardwareInterface):
            raise TypeError("hardware_interface must implement HardwareInterface")
        
        self.__hardware = hardware_interface
        self.__control_frequency = control_frequency
        self.__n_derivatives = max(0, n_derivatives)
        
        # Wait for hardware to be ready
        if not self.__hardware.is_ready():
            print("Waiting for hardware to be ready...")
            while not self.__hardware.is_ready():
                time.sleep(0.1)
        
        # Get joint configuration
        self.__joint_num = self.__hardware.get_joint_count()
        
        # Control parameters
        self.__target_change_thresh = 0.2
        
        # Target position and synchronization
        self.__target_lock = threading.Lock()
        self.__current_target = np.zeros(self.__joint_num)
        self.__target_reached = threading.Event()
        
        # Control position state (independent of hardware feedback)
        self.__control_pos_lock = threading.Lock()
        
        # Initialize control position with current hardware position
        current_pos = self.__hardware.get_current_position()
        if current_pos is not None:
            self.__current_target = current_pos.copy()
            self.__control_pos = current_pos.copy()
        else:
            self.__control_pos = np.zeros(self.__joint_num)
        
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
        
        # Integral error accumulation for eliminating steady-state error
        self.__integral_error = np.zeros(self.__joint_num)
        self.__integral_gain = 0.1  # Default integral gain k_i
        
        # Control thread management
        self.__control_thread = None
        self.__stop_event = threading.Event()
        
        # Debug counter for periodic printing
        self.__debug_counter = 0
        
        print(f"DifferentialController initialized with {self.__n_derivatives}-order control")

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
    
    def _print_control_state(self, control_pos, target_pos):
        """Print current control state with derivatives"""
        # Get current derivatives and integral error
        with self.__control_state_lock:
            current_derivs = self.__current_derivatives.copy()
            integral_error = self.__integral_error.copy()
        
        # Calculate current position error
        position_error = target_pos - control_pos
        
        # Format position (control position)
        pos_str = f"Pos: [{', '.join([f'{x:.2f}' for x in control_pos])}]"
        
        # Format target
        target_str = f"Target: [{', '.join([f'{x:.2f}' for x in target_pos])}]"
        
        # Format position error
        error_str = f"Error: [{', '.join([f'{x:.2f}' for x in position_error])}]"
        
        # Format integral error
        integral_str = f"Integral: [{', '.join([f'{x:.2f}' for x in integral_error])}]"
        
        # Format derivatives based on available orders
        deriv_strs = []
        derivative_names = ['Vel', 'Acc', 'Jerk', 'Snap', 'Crackle', 'Pop']
        
        for i in range(min(len(current_derivs), len(derivative_names))):
            name = derivative_names[i]
            values = current_derivs[i]
            deriv_str = f"{name}: [{', '.join([f'{x:.2f}' for x in values])}]"
            deriv_strs.append(deriv_str)
        
        # Print all information
        print(f"Step {self.__debug_counter}: {pos_str}")
        print(f"         {target_str}")
        print(f"         {error_str}")
        print(f"         {integral_str}")
        for deriv_str in deriv_strs:
            print(f"         {deriv_str}")
        print()  # Empty line for readability
    
    def _compute_differential_control(self, target_pos, dt):
        """
        Compute control position using n-th order differential equations with constraints
        
        General n-th order differential control algorithm:
        1. Calculate position error: e_0 = x_target - x_control
        2. For i = 0 to n-2:
           - Calculate desired (i+1)-th derivative: d_{i+1} = sat(k_i * (d_desired_i - d_current_i), max_i+1)
        3. Calculate highest order derivative (n-1): d_{n-1} = sat(k_{n-2} * (d_desired_{n-2} - d_current_{n-2}), max_{n-1})
        4. Update all derivatives using Taylor expansion
        5. Update position using Taylor expansion with all derivatives
        
        Args:
            target_pos: Target joint positions  
            dt: Time step
            
        Returns:
            control_pos: Next control position
        """
        with self.__control_state_lock:
            # Copy current derivatives (velocity, acceleration, jerk, etc.)
            current_derivs = self.__current_derivatives.copy()
        
        with self.__control_pos_lock:
            # Use internal control position state, not hardware feedback
            control_pos = self.__control_pos.copy()
        
        # Step 1: Calculate position error based on control position
        position_error = target_pos - control_pos
        
        # Step 2: Update integral error (积分误差更新)
        with self.__control_state_lock:
            # e_i,k = e_i,k-1 + k_i * e_k * Δt
            self.__integral_error += self.__integral_gain * position_error * dt
        
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
            new_control_pos = target_pos
        # For n=1: only position control
        elif self.__n_derivatives == 1:
            # Direct position control with saturation
            new_control_pos = self._sat(control_pos + position_error, self.__max_derivatives[0])
        else:
            # For n>=2: use derivative control
            # Step 3: Calculate desired velocity with integral term (含积分项)
            # v_d = sat(k_p * e_k + k_pi * e_i,k, v_max)
            with self.__control_state_lock:
                integral_error = self.__integral_error.copy()
            desired_velocity = self.__control_gains[0] * position_error + integral_error
            desired_derivs[0] = self._sat(desired_velocity, self.__max_derivatives[1])
            
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
            
            new_control_pos = self._sat(control_pos + position_update, self.__max_derivatives[0])
        
        # Update internal state variables
        with self.__control_state_lock:
            if self.__n_derivatives > 1:
                self.__current_derivatives[:] = new_derivs
        
        # Update internal control position
        with self.__control_pos_lock:
            self.__control_pos[:] = new_control_pos
        
        return new_control_pos

    def __control_thread_func(self):
        """Control thread that runs independently and sends commands to hardware"""
        rate_sleep_time = 1.0 / self.__control_frequency
        dt = rate_sleep_time  # Time step
        
        while not self.__stop_event.is_set():
            try:
                # Get current target
                with self.__target_lock:
                    target_pos = self.__current_target.copy()
                
                # Get current control position for target reached check
                with self.__control_pos_lock:
                    control_pos = self.__control_pos.copy()
                
                # Calculate delta position for target reached check (based on control position)
                delta_pos = target_pos - control_pos
                delta_pos_norm = np.linalg.norm(delta_pos)
                
                # Check if target is reached
                if delta_pos_norm < self.__target_change_thresh:
                    self.__target_reached.set()
                else:
                    self.__target_reached.clear()
                
                # Use differential control algorithm to compute next control position
                new_control_pos = self._compute_differential_control(target_pos, dt)
                
                # Print debug information every 10 steps (every 0.2 seconds at 50Hz)
                self.__debug_counter += 1
                if self.__debug_counter % 10 == 0:
                    self._print_control_state(new_control_pos, target_pos)
                
                # Send command to hardware
                self.__hardware.send_position_command(new_control_pos)
                
                time.sleep(rate_sleep_time)
                
            except Exception as e:
                print(f"Control thread error: {e}")
                time.sleep(rate_sleep_time)
    
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
            target_pos: Target joint positions (should match joint count)
            wait (bool): Whether to wait for target to be reached (default: False)
            timeout (float): Maximum time to wait if wait=True (default: 10.0)
            
        Returns:
            bool: If wait=True, returns True if target reached within timeout, False otherwise.
                  If wait=False, always returns True.
        """
        target_pos = np.array(target_pos)
        if len(target_pos) != self.__joint_num:
            raise ValueError(f"Target position must have {self.__joint_num} elements")
        
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
        """Get current joint positions from hardware"""
        return self.__hardware.get_current_position()
    
    def get_control_position(self):
        """Get current control position (internal state)"""
        with self.__control_pos_lock:
            return self.__control_pos.copy()
    
    def get_current_target(self):
        """Get current target position"""
        with self.__target_lock:
            return self.__current_target.copy()
    
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
    
    def set_integral_gain(self, k_i):
        """Set integral gain for eliminating steady-state error
        
        Args:
            k_i (float): Integral gain (should be positive)
        """
        if k_i > 0:
            self.__integral_gain = k_i
        else:
            raise ValueError("Integral gain must be positive")
    
    def get_integral_gain(self):
        """Get current integral gain
        
        Returns:
            float: Current integral gain
        """
        return self.__integral_gain
    
    def reset_integral_error(self):
        """Reset accumulated integral error to zero"""
        with self.__control_state_lock:
            self.__integral_error.fill(0.0)
        print("Integral error reset to zero")
    
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

    def __del__(self):
        """Destructor to ensure control process is stopped"""
        self.stop_control_process()


if __name__ == '__main__':
    print("DifferentialController module - import this module to use the DifferentialController class")
    print("Example usage:")
    print("  from differential_controller import DifferentialController, HardwareInterface")
    print("  controller = DifferentialController(your_hardware_interface, control_frequency=50.0, n_derivatives=4)")
    print("  controller.start_control_process()")
    print("  controller.set_target_position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])")
