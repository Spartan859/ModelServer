#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-10
# 
# Direct Controller for Robotic Systems
# 
# This module provides a direct control approach that immediately
# sends target positions to hardware without PID computation.
# WARNING: This can be dangerous as it provides no smoothing!
################################################################

import threading
import time
import numpy as np
from .pid_controller import HardwareInterface


class DirectController:
    """
    Direct controller for robotic systems.
    
    This class provides immediate position control without PID computation.
    Target positions are sent directly to hardware without any smoothing,
    interpolation, or feedback control.
    
    WARNING: This controller can be dangerous as it:
    - Provides no motion smoothing
    - Has no error correction
    - May cause sudden jerky movements
    - Does not account for hardware limitations
    
    Use with extreme caution!
    """

    def __init__(self, hardware_interface, control_frequency=50.0):
        """
        Initialize DirectController
        
        Args:
            hardware_interface (HardwareInterface): Hardware interface implementation
            control_frequency (float): Control loop frequency in Hz (default: 50.0)
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
        
        # Target position and synchronization
        self.__target_lock = threading.Lock()
        self.__current_target = np.zeros(self.__joint_num)
        self.__target_reached = threading.Event()
        
        # Initialize target with current hardware position
        current_pos = self.__hardware.get_current_position()
        if current_pos is not None:
            self.__current_target = current_pos.copy()
        
        # Target reached threshold
        self.__target_change_thresh = 0.2
        
        # Control thread management
        self.__control_thread = None
        self.__stop_event = threading.Event()
        
        print(f"直接控制器初始化完成，控制频率: {self.__control_frequency}Hz")
        print("警告: 直接控制器不提供运动平滑，使用时请格外小心！")
        
        # Start control process automatically
        self.start_control_process()

    def __control_thread_func(self):
        """Control thread that continuously sends current target position to hardware"""
        rate_sleep_time = 1.0 / self.__control_frequency
        
        while not self.__stop_event.is_set():
            try:
                # Get current target position
                with self.__target_lock:
                    target_pos = self.__current_target.copy()
                
                # Get current hardware position for target reached check
                hardware_pos = self.__hardware.get_current_position()
                
                # Check if target is reached (based on hardware position)
                if hardware_pos is not None:
                    delta_pos = target_pos - hardware_pos
                    delta_pos_norm = np.linalg.norm(delta_pos)
                    
                    if delta_pos_norm < self.__target_change_thresh:
                        self.__target_reached.set()
                    else:
                        self.__target_reached.clear()
                
                # Send current target position directly to hardware
                # This ensures hardware always receives position commands
                self.__hardware.send_position_command(target_pos)
                
                time.sleep(rate_sleep_time)
                
            except Exception as e:
                print(f"直接控制线程错误: {e}")
                time.sleep(rate_sleep_time)

    def start_control_process(self):
        """Start the control thread"""
        if self.__control_thread is not None and self.__control_thread.is_alive():
            print("直接控制线程已在运行")
            return
        
        self.__stop_event.clear()
        self.__control_thread = threading.Thread(target=self.__control_thread_func)
        self.__control_thread.daemon = True
        self.__control_thread.start()
        print("直接控制线程已启动")

    def stop_control_process(self):
        """Stop the control thread"""
        if self.__control_thread is not None and self.__control_thread.is_alive():
            self.__stop_event.set()
            self.__control_thread.join(timeout=2.0)
            print("直接控制线程已停止")

    def set_target_position(self, target_pos, wait=False, timeout=10.0):
        """Set target position (will be sent continuously by control thread)
        
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
        
        # Update shared target position (control thread will send it continuously)
        with self.__target_lock:
            self.__current_target[:] = target_pos
        
        # Reset target reached flag
        self.__target_reached.clear()
        print(f"直接控制器: 目标位置设定为 {target_pos}")
        
        # Optionally wait for target to be reached
        if wait:
            return self.__target_reached.wait(timeout)
        return True

    def is_target_reached(self):
        """Check if current target position is reached"""
        return self.__target_reached.is_set()

    def get_current_position(self):
        """Get current hardware position"""
        return self.__hardware.get_current_position()
    
    def get_current_velocity(self):
        """Get current hardware velocity"""
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
        """Set PID gains (no-op for direct controller)"""
        print("直接控制器不使用PID参数")

    def get_pid_gains(self):
        """Get PID gains (returns zeros for direct controller)"""
        zeros = np.zeros(self.__joint_num)
        return zeros, zeros, zeros, zeros
    
    def get_base_pid_gains(self):
        """Get base PID gains (returns zeros for direct controller)"""
        zeros = np.zeros(self.__joint_num)
        return zeros, zeros, zeros
    
    def get_dynamic_kp(self):
        """Get dynamic Kp values (returns zeros for direct controller)"""
        return np.zeros(self.__joint_num)

    def set_integral_limits(self, limits):
        """Set integral limits (no-op for direct controller)"""
        print("直接控制器不使用积分限制")

    def set_output_limits(self, limits):
        """Set output limits (no-op for direct controller)"""
        print("直接控制器不使用输出限制")

    def set_derivative_filter(self, alpha):
        """Set derivative filter (no-op for direct controller)"""
        print("直接控制器不使用微分滤波")
    
    def set_kp_max(self, max_value):
        """Set maximum P value (no-op for direct controller)"""
        print("直接控制器不使用P值上限")
    
    def get_kp_max(self):
        """Get maximum P value (returns 0 for direct controller)"""
        return 0.0

    def reset_pid_state(self):
        """Reset PID state (no-op for direct controller)"""
        print("直接控制器无PID状态需要重置")

    def get_pid_errors(self):
        """Get PID errors (returns None for direct controller)"""
        return None, None, None
    
    def get_control_point(self):
        """Get control point (returns current target for direct controller)"""
        return self.get_current_target()
    
    def set_control_point(self, position):
        """Set control point (equivalent to set_target_position for direct controller)"""
        position = np.array(position)
        if len(position) != self.__joint_num:
            raise ValueError(f"控制点位置必须有 {self.__joint_num} 个元素")
        
        self.set_target_position(position)
    
    def sync_control_point_to_hardware(self):
        """Sync control point to hardware (updates target to current hardware position)"""
        hardware_pos = self.__hardware.get_current_position()
        if hardware_pos is not None:
            with self.__target_lock:
                self.__current_target[:] = hardware_pos
            print(f"直接控制器目标位置已同步到硬件位置: {hardware_pos}")
        else:
            print("警告: 无法获取硬件位置，同步失败")

    def get_target_threshold(self):
        """Get target reached threshold"""
        return self.__target_change_thresh

    def set_target_threshold(self, threshold):
        """Set target reached threshold"""
        if threshold > 0:
            self.__target_change_thresh = threshold
            print(f"目标到达阈值设定为: {threshold}")
        else:
            raise ValueError("阈值必须为正数")

    def __del__(self):
        """Destructor to ensure control process is stopped"""
        self.stop_control_process()


if __name__ == '__main__':
    print("直接控制器模块 - 导入此模块以使用DirectController类")
    print("使用示例:")
    print("  from direct_controller import DirectController")
    print("  controller = DirectController(your_hardware_interface)")
    print("  controller.set_target_position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])")
    print("警告: 直接控制器不提供运动平滑，使用时请格外小心！")
