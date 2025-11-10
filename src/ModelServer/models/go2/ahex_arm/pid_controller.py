#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-05-08
# 
# Universal PID Controller for Robotic Systems
# 
# This module provides a hardware-agnostic PID control
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


class PIDController:
    """
    Universal PID controller for robotic systems.
    
    This class implements a classic PID (Proportional-Integral-Derivative) control system
    that provides smooth and accurate position control for robotic joints.
    
    Features:
    - Hardware-agnostic design (works with any HardwareInterface implementation)
    - Configurable PID gains (Kp, Ki, Kd) for each joint or globally
    - Thread-safe operation with real-time control loop
    - Anti-windup protection for integral term
    - Derivative filtering to reduce noise
    - Target tracking with customizable thresholds
    
    PID Control Equation:
    u(t) = Kp * e(t) + Ki * ∫e(τ)dτ + Kd * de(t)/dt
    
    Where:
    - e(t) = target_position - current_position (error)
    - Kp = Proportional gain
    - Ki = Integral gain  
    - Kd = Derivative gain
    """

    def __init__(self, hardware_interface, control_frequency=50.0, 
                 kp=None, ki=None, kd=None):
        """
        Initialize PIDController
        
        Args:
            hardware_interface (HardwareInterface): Hardware interface implementation
            control_frequency (float): Control loop frequency in Hz (default: 50.0)
            kp (float or np.array): Proportional gain(s) (default: joint-specific values)
            ki (float or np.array): Integral gain(s) (default: joint-specific values)
            kd (float or np.array): Derivative gain(s) (default: joint-specific values)
        """
        if not isinstance(hardware_interface, HardwareInterface):
            raise TypeError("hardware_interface must implement HardwareInterface")
        
        self.__hardware = hardware_interface
        self.__control_frequency = control_frequency
        
        # Wait for hardware to be ready
        if not self.__hardware.is_ready():
            print("等待硬件准备就绪...")
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
        
        # Initialize target and control point with current hardware position
        current_pos = self.__hardware.get_current_position()
        if current_pos is not None:
            self.__current_target = current_pos.copy()
        
        # Internal control point (independent of hardware feedback)
        self.__control_point_lock = threading.Lock()
        if current_pos is not None:
            self.__control_point = current_pos.copy()
        else:
            self.__control_point = np.zeros(self.__joint_num)
        
        # PID gains - can be per-joint or global
        self.__pid_lock = threading.Lock()
        
        # Set default PID values if not provided
        if kp is None:
            kp = self._get_default_kp()
        if ki is None:
            ki = self._get_default_ki()
        if kd is None:
            kd = self._get_default_kd()
        
        self.__kp_base = self._setup_gains(kp, "Kp")  # Base P values (p_0)
        self.__ki = self._setup_gains(ki, "Ki") 
        self.__kd = self._setup_gains(kd, "Kd")
        
        # Dynamic P values (will be updated in real-time)
        self.__kp = self.__kp_base.copy()
        
        # P value limits and thresholds
        self.__kp_max = 16.0  # Maximum P value
        
        # PID state variables (based on control point, not hardware position)
        self.__previous_error = np.zeros(self.__joint_num)
        self.__integral_error = np.zeros(self.__joint_num)
        self.__previous_time = None
        
        # Anti-windup limits for integral term
        self.__integral_limit = np.full(self.__joint_num, 10.0)  # Default limit
        
        # Derivative filter parameters (low-pass filter)
        self.__derivative_filter_alpha = 0.1  # Filter coefficient (0-1, lower = more filtering)
        self.__filtered_derivative = np.zeros(self.__joint_num)
        
        # Output limits
        self.__output_limit = np.full(self.__joint_num, np.inf)  # No limit by default
        
        # Control thread management
        self.__control_thread = None
        self.__stop_event = threading.Event()
        
        # Debug counter for periodic printing
        self.__debug_counter = 0
        
        # Set default limits and start control process
        self.set_output_limits(10.0)  # Default output limit
        self.set_integral_limits(5.0)  # Default integral limit
        
        print(f"PID控制器初始化完成，控制频率: {self.__control_frequency}Hz")
        print(f"PID基础参数 - Kp_base: {self.__kp_base}, Ki: {self.__ki}, Kd: {self.__kd}")
        print(f"动态P值上限: {self.__kp_max}")
        
        # Start control process automatically
        self.start_control_process()

    def _get_default_kp(self):
        """Get default Kp values for different joints"""
        if self.__joint_num < 3:
            # If less than 3 joints, use default value
            return np.full(self.__joint_num, 1.0)
        
        kp_defaults = np.zeros(self.__joint_num)
        # Joints 0,1: Kp = 1.0
        kp_defaults[0:2] = 0.8
        # Joint 2: Kp = 2.0  
        kp_defaults[2] = 1.6
        # Other joints: Kp = 4.0
        if self.__joint_num > 3:
            kp_defaults[3:] = 1.6
        
        return kp_defaults
    
    def _get_default_ki(self):
        """Get default Ki values for different joints (all zeros)"""
        return np.zeros(self.__joint_num)
    
    def _get_default_kd(self):
        """Get default Kd values for different joints (all zeros)"""
        return np.zeros(self.__joint_num)

    def _setup_gains(self, gain, name):
        """Setup gain parameters - can be scalar or per-joint array"""
        if np.isscalar(gain):
            return np.full(self.__joint_num, gain)
        else:
            gain_array = np.array(gain)
            if len(gain_array) != self.__joint_num:
                raise ValueError(f"{name} array length must match joint count ({self.__joint_num})")
            return gain_array

    def _print_control_state(self, control_point, target_pos, control_output, 
                           error, integral_error, derivative_error, hardware_pos=None):
        """Print current PID control state"""
        # Format control point (internal reference)
        control_str = f"控制点: [{', '.join([f'{x:.3f}' for x in control_point])}]"
        
        # Format target
        target_str = f"目标位置: [{', '.join([f'{x:.3f}' for x in target_pos])}]"
        
        # Format hardware position if available
        if hardware_pos is not None:
            hardware_str = f"硬件位置: [{', '.join([f'{x:.3f}' for x in hardware_pos])}]"
        else:
            hardware_str = "硬件位置: [无法获取]"
        
        # Format errors
        error_str = f"位置误差: [{', '.join([f'{x:.3f}' for x in error])}]"
        integral_str = f"积分误差: [{', '.join([f'{x:.3f}' for x in integral_error])}]"
        derivative_str = f"微分误差: [{', '.join([f'{x:.3f}' for x in derivative_error])}]"
        
        # Format control output
        output_str = f"控制输出: [{', '.join([f'{x:.3f}' for x in control_output])}]"
        
        # Format dynamic Kp values
        with self.__pid_lock:
            current_kp = self.__kp.copy()
        kp_str = f"动态Kp: [{', '.join([f'{x:.2f}' for x in current_kp])}]"
        
        # Print all information
        print(f"步骤 {self.__debug_counter}: {control_str}")
        print(f"         {target_str}")
        print(f"         {hardware_str}")
        print(f"         {error_str}")
        print(f"         {integral_str}")
        print(f"         {derivative_str}")
        print(f"         {kp_str}")
        print(f"         {output_str}")
        print()  # Empty line for readability

    def _update_dynamic_kp(self, error):
        """
        Update dynamic Kp values based on error magnitude
        
        动态P值调整规则:
        基本关系: p = p_0 / |e| (误差越大p值越小，成反比例关系)
        上限限制: p值绝对不能超过16
        
        Args:
            error: Current position error for each joint
        """
        with self.__pid_lock:
            kp_base = self.__kp_base.copy()
        
        # Calculate absolute error for each joint
        abs_error = np.abs(error)
        
        # Initialize new kp values
        new_kp = np.zeros(self.__joint_num)
        
        for i in range(self.__joint_num):
            # 使用反比例关系: p = p_0 / |e|
            if abs_error[i] > 1e-6:  # Avoid division by zero
                new_kp[i] = kp_base[i] / abs_error[i]
            else:
                new_kp[i] = self.__kp_max  # Very small error, use maximum gain
            
            # 应用上限限制: 绝对不能超过16
            new_kp[i] = min(new_kp[i], self.__kp_max)
        
        # Update dynamic kp values
        with self.__pid_lock:
            self.__kp[:] = new_kp

    def _compute_pid_control(self, target_pos, dt):
        """
        Compute PID control output based on internal control point
        
        PID控制算法 (基于内部控制点):
        1. 获取内部控制点位置: control_point (独立于硬件反馈)
        2. 计算位置误差: e(t) = target - control_point  
        3. 更新积分项: ∫e(τ)dτ += e(t) * dt (带积分限幅)
        4. 计算微分项: de(t)/dt = (e(t) - e(t-1)) / dt (带滤波)
        5. 计算控制输出: u(t) = Kp*e(t) + Ki*∫e(τ)dτ + Kd*de(t)/dt
        6. 更新控制点: control_point += u(t) * dt
        7. 应用输出限幅
        
        Args:
            target_pos: Target joint positions  
            dt: Time step
            
        Returns:
            tuple: (new_control_point, control_output, error, integral_error, derivative_error)
        """
        with self.__pid_lock:
            kp = self.__kp.copy()
            ki = self.__ki.copy()
            kd = self.__kd.copy()
        
        # Get current control point
        with self.__control_point_lock:
            control_point = self.__control_point.copy()
        
        # Step 1: Calculate position error based on control point
        error = target_pos - control_point
        
        # Step 1.5: Update dynamic Kp values based on error magnitude
        self._update_dynamic_kp(error)
        
        # Step 2: Update integral error with anti-windup
        self.__integral_error += error * dt
        # Apply integral limits (anti-windup)
        self.__integral_error = np.clip(self.__integral_error, 
                                      -self.__integral_limit, 
                                      self.__integral_limit)
        
        # Step 3: Calculate derivative error
        if self.__previous_time is not None:
            derivative_error = (error - self.__previous_error) / dt
            
            # Apply low-pass filter to derivative term to reduce noise
            self.__filtered_derivative = (self.__derivative_filter_alpha * derivative_error + 
                                        (1 - self.__derivative_filter_alpha) * self.__filtered_derivative)
        else:
            derivative_error = np.zeros(self.__joint_num)
            self.__filtered_derivative = np.zeros(self.__joint_num)
        
        # Step 4: Calculate PID control output
        proportional_term = kp * error
        integral_term = ki * self.__integral_error
        derivative_term = kd * self.__filtered_derivative
        
        control_output = proportional_term + integral_term + derivative_term
        
        # Step 5: Apply output limits
        control_output = np.clip(control_output, -self.__output_limit, self.__output_limit)
        
        # Step 6: Update internal control point
        new_control_point = control_point + control_output * dt
        
        # Update internal control point state
        with self.__control_point_lock:
            self.__control_point[:] = new_control_point
        
        # Update previous values for next iteration
        self.__previous_error = error.copy()
        self.__previous_time = time.time()
        
        return new_control_point, control_output, error, self.__integral_error.copy(), self.__filtered_derivative.copy()

    def __control_thread_func(self):
        """Control thread that runs independently and sends commands to hardware"""
        rate_sleep_time = 1.0 / self.__control_frequency
        dt = rate_sleep_time  # Time step
        
        # Initialize timing
        self.__previous_time = time.time()
        
        while not self.__stop_event.is_set():
            try:
                # Get current target
                with self.__target_lock:
                    target_pos = self.__current_target.copy()
                
                # Get current control point for target reached check
                with self.__control_point_lock:
                    control_point = self.__control_point.copy()
                
                # Calculate delta position for target reached check (based on control point)
                delta_pos = target_pos - control_point
                delta_pos_norm = np.linalg.norm(delta_pos)
                
                # Check if target is reached
                if delta_pos_norm < self.__target_change_thresh:
                    self.__target_reached.set()
                else:
                    self.__target_reached.clear()
                
                # Compute PID control output based on internal control point
                new_control_point, control_output, error, integral_error, derivative_error = self._compute_pid_control(
                    target_pos, dt)
                
                # Get hardware position for monitoring (optional, doesn't affect control)
                hardware_pos = self.__hardware.get_current_position()
                
                # Print debug information every 10 steps (every 0.2 seconds at 50Hz)
                # self.__debug_counter += 1
                # if self.__debug_counter % 10 == 0:
                #     self._print_control_state(new_control_point, target_pos, control_output,
                #                             error, integral_error, derivative_error, hardware_pos)
                
                # Send control point as command to hardware (not hardware position + control output)
                self.__hardware.send_position_command(new_control_point)
                
                time.sleep(rate_sleep_time)
                
            except Exception as e:
                print(f"控制线程错误: {e}")
                time.sleep(rate_sleep_time)

    def start_control_process(self):
        """Start the control thread"""
        if self.__control_thread is not None and self.__control_thread.is_alive():
            print("控制线程已在运行")
            return
        
        self.__stop_event.clear()
        self.__control_thread = threading.Thread(target=self.__control_thread_func)
        self.__control_thread.daemon = True
        self.__control_thread.start()
        print("控制线程已启动")

    def stop_control_process(self):
        """Stop the control thread"""
        if self.__control_thread is not None and self.__control_thread.is_alive():
            self.__stop_event.set()
            self.__control_thread.join(timeout=2.0)
            print("控制线程已停止")

    def set_target_position(self, target_pos, wait=False, timeout=10.0):
        """Set target position
        
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
            raise ValueError(f"目标位置必须有 {self.__joint_num} 个元素")
        
        # Update shared target position
        with self.__target_lock:
            self.__current_target[:] = target_pos
        
        # Reset target reached flag
        self.__target_reached.clear()
        print(f"目标位置设定为: {target_pos}")
        
        # Optionally wait for target to be reached
        if wait:
            return self.__target_reached.wait(timeout)
        return True

    def is_target_reached(self):
        """Check if current target position is reached"""
        return self.__target_reached.is_set()

    def get_current_position(self):
        """Get current control point position (not hardware position)"""
        with self.__control_point_lock:
            return self.__control_point.copy()
    
    def get_current_velocity(self):
        """Get current control point velocity (not hardware velocity)"""
        return self.__hardware.get_current_velocity()
    
    def get_hardware_position(self):
        """Get actual hardware joint positions"""
        return self.__hardware.get_current_position()

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
            raise ValueError("控制频率必须为正数")
        self.__control_frequency = frequency

    def set_pid_gains(self, kp=None, ki=None, kd=None):
        """Set PID gains
        
        Args:
            kp (float or np.array): Base proportional gain(s) (will be used for dynamic adjustment)
            ki (float or np.array): Integral gain(s)
            kd (float or np.array): Derivative gain(s)
        """
        with self.__pid_lock:
            if kp is not None:
                self.__kp_base = self._setup_gains(kp, "Kp_base")
                self.__kp = self.__kp_base.copy()  # Initialize dynamic kp with base values
            if ki is not None:
                self.__ki = self._setup_gains(ki, "Ki")
            if kd is not None:
                self.__kd = self._setup_gains(kd, "Kd")
        
        print(f"PID参数已更新 - Kp_base: {self.__kp_base}, Ki: {self.__ki}, Kd: {self.__kd}")

    def get_pid_gains(self):
        """Get current PID gains
        
        Returns:
            tuple: (kp_base, ki, kd, kp_dynamic) arrays
        """
        with self.__pid_lock:
            return self.__kp_base.copy(), self.__ki.copy(), self.__kd.copy(), self.__kp.copy()
    
    def get_base_pid_gains(self):
        """Get base PID gains (before dynamic adjustment)
        
        Returns:
            tuple: (kp_base, ki, kd) arrays
        """
        with self.__pid_lock:
            return self.__kp_base.copy(), self.__ki.copy(), self.__kd.copy()
    
    def get_dynamic_kp(self):
        """Get current dynamic Kp values
        
        Returns:
            np.array: Current dynamic Kp values
        """
        with self.__pid_lock:
            return self.__kp.copy()

    def set_integral_limits(self, limits):
        """Set integral anti-windup limits
        
        Args:
            limits (float or np.array): Maximum absolute values for integral terms
        """
        if np.isscalar(limits):
            self.__integral_limit = np.full(self.__joint_num, limits)
        else:
            limits_array = np.array(limits)
            if len(limits_array) != self.__joint_num:
                raise ValueError(f"积分限制数组长度必须匹配关节数量 ({self.__joint_num})")
            self.__integral_limit = limits_array
        
        print(f"积分限制设定为: {self.__integral_limit}")

    def set_output_limits(self, limits):
        """Set output limits
        
        Args:
            limits (float or np.array): Maximum absolute values for control outputs
        """
        if np.isscalar(limits):
            self.__output_limit = np.full(self.__joint_num, limits)
        else:
            limits_array = np.array(limits)
            if len(limits_array) != self.__joint_num:
                raise ValueError(f"输出限制数组长度必须匹配关节数量 ({self.__joint_num})")
            self.__output_limit = limits_array
        
        print(f"输出限制设定为: {self.__output_limit}")

    def set_derivative_filter(self, alpha):
        """Set derivative filter coefficient
        
        Args:
            alpha (float): Filter coefficient (0-1), lower values = more filtering
        """
        if not 0 <= alpha <= 1:
            raise ValueError("滤波系数必须在0到1之间")
        self.__derivative_filter_alpha = alpha
        print(f"微分滤波系数设定为: {alpha}")
    
    def set_kp_max(self, max_value):
        """Set maximum P value limit
        
        Args:
            max_value (float): Maximum P value (default: 16.0)
        """
        if max_value <= 0:
            raise ValueError("P值上限必须为正数")
        self.__kp_max = max_value
        print(f"P值上限设定为: {max_value}")
    
    def get_kp_max(self):
        """Get maximum P value limit"""
        return self.__kp_max

    def reset_pid_state(self):
        """Reset PID internal state (integral error, previous error, etc.)"""
        self.__previous_error.fill(0.0)
        self.__integral_error.fill(0.0)
        self.__filtered_derivative.fill(0.0)
        self.__previous_time = None
        print("PID状态已重置")

    def get_pid_errors(self):
        """Get current PID error terms
        
        Returns:
            tuple: (current_error, integral_error, derivative_error)
        """
        # Get current control point and target
        control_point = self.get_current_position()  # This now returns control point
        target_pos = self.get_current_target()
        
        if control_point is not None:
            current_error = target_pos - control_point
            return current_error, self.__integral_error.copy(), self.__filtered_derivative.copy()
        else:
            return None, None, None
    
    def get_control_point(self):
        """Get current internal control point position"""
        with self.__control_point_lock:
            return self.__control_point.copy()
    
    def set_control_point(self, position):
        """Set internal control point position
        
        Args:
            position: New control point position (should match joint count)
        """
        position = np.array(position)
        if len(position) != self.__joint_num:
            raise ValueError(f"控制点位置必须有 {self.__joint_num} 个元素")
        
        with self.__control_point_lock:
            self.__control_point[:] = position
        
        print(f"控制点设定为: {position}")
    
    def sync_control_point_to_hardware(self):
        """Synchronize control point with current hardware position"""
        hardware_pos = self.__hardware.get_current_position()
        if hardware_pos is not None:
            with self.__control_point_lock:
                self.__control_point[:] = hardware_pos
            print(f"控制点已同步到硬件位置: {hardware_pos}")
        else:
            print("警告: 无法获取硬件位置，控制点同步失败")

    def get_target_threshold(self):
        """Get target reached threshold"""
        return self.__target_change_thresh

    def set_target_threshold(self, threshold):
        """Set target reached threshold
        
        Args:
            threshold (float): Distance threshold for considering target reached
        """
        if threshold > 0:
            self.__target_change_thresh = threshold
            print(f"目标到达阈值设定为: {threshold}")
        else:
            raise ValueError("阈值必须为正数")

    def __del__(self):
        """Destructor to ensure control process is stopped"""
        self.stop_control_process()


if __name__ == '__main__':
    print("PID控制器模块 - 导入此模块以使用PIDController类")
    print("使用示例:")
    print("  from pid_controller import PIDController, HardwareInterface")
    print("  controller = PIDController(your_hardware_interface, control_frequency=50.0)")
    print("  controller.set_pid_gains(kp=1.0, ki=0.1, kd=0.05)")
    print("  controller.start_control_process()")
    print("  controller.set_target_position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])")
