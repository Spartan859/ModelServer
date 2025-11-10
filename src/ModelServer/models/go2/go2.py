from .robot_interface import GO2Interface
from .gui_controller import GUIController
from .gopro.gopro10 import init_gopro10
# from .ahex_arm.ahex_arm_hardware import AhexArmHardware
# from .ahex_arm.pid_controller import PIDController
from .gripper.gripper_control import Gripper
import time
import numpy as np


class Go2:
    def __init__(self, ros_version='foxy', arm_controller_type='direct', init_gripper=True):
        """Initialize Go2 robot system
        
        Args:
            ros_version (str): ROS version ('foxy' or 'noetic')
            arm_controller_type (str): Arm controller type ('pid' or 'direct')
                - 'pid': Use PID controller with smooth motion (safer, default)
                - 'direct': Use direct controller with immediate positioning (dangerous!)
            init_gripper (bool): Whether to initialize the gripper (default: False)
        """
        print("Initializing Go2 robot system...")
        # receive the robot perception information in one process
        if ros_version == 'foxy':
            self.robot = GO2Interface('eth0')
            print("Robot interface initialized")
            if init_gripper:
                self.gripper = Gripper()
                print("Gripper initialized")
            else:
                self.gripper = None
                print("Gripper not initialized")
            self.gopro10 = init_gopro10(width=224, height=224)
            print("GoPro10 initialized")
        elif ros_version == 'noetic':
            self._init_arm_control(arm_controller_type)
            pass

        self.joint_stand = [-0.009106457233428955, 0.7097946405410767, -1.4127792119979858, -0.011134743690490723, 0.7262511253356934, -1.5128371715545654, -0.011184871196746826, 0.7382053136825562, -1.438386082649231, 0.10946577787399292, 0.6780438423156738, -1.4266014099121094]
        self.joint_sit = [0.04559052735567093, 1.2599834203720093, -2.7768003940582275, -0.06560397148132324, 1.3015544414520264, -2.795393228530884, -0.2913428544998169, 1.297098159790039, -2.8097052574157715, 0.30260616540908813, 1.3035025596618652, -2.818251371383667]
        self.joint_zero = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.arm_default = np.array([0.0, -1.5, 3.0, 0.0, 0.0, 0.0])
        self.arm_up = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.state = 'stop'

        print('Go2 initialized')
    
    def _init_arm_control(self, controller_type='pid'):
        """Initialize arm control system
        
        Args:
            controller_type (str): Controller type ('pid' or 'direct')
        """
        print(f"Initializing arm control system with {controller_type.upper()} controller...")
        from .ahex_arm.ahex_arm_hardware import AhexArmHardware
        
        self.arm_hardware = AhexArmHardware()
        self.arm_controller_type = controller_type
        
        if controller_type == 'pid':
            from .ahex_arm.pid_controller import PIDController
            # Initialize PID controller with appropriate gains
            self.arm_controller = PIDController(
                self.arm_hardware, 
                control_frequency=50.0,  # Higher frequency for better performance
                # kp=0.4,    # Proportional gain
                # ki=0.1,    # Integral gain  
                # kd=0.05    # Derivative gain
            )
            print("PID控制器初始化完成 - 提供平滑运动控制")
        elif controller_type == 'direct':
            from .ahex_arm.direct_controller import DirectController
            # Initialize direct controller
            self.arm_controller = DirectController(self.arm_hardware)
            print("直接控制器初始化完成 - 警告：无运动平滑，请谨慎使用！")
        else:
            raise ValueError(f"不支持的控制器类型: {controller_type}. 支持的类型: 'pid', 'direct'")
        
        # Move to default position
        default_pos = self.arm_hardware.get_default_position()
        self.arm_controller.set_target_position(default_pos)
        print("Arm control system initialized")

    def remote_control_loop(self):
        self.controller = GUIController(self.robot)
        self.controller.rollout()
    
    def get_gopro_frame(self, num_frames=1):
        return self.gopro10.read_frame(num_frames)

    def get_camera_image(self, resolution=None):
        """Get image from the robot's onboard camera (cached for low latency).

        Args:
            resolution: tuple (width, height) or None for original resolution
            
        Returns:
            numpy.ndarray or None: RGB image with specified resolution, or None if unavailable.
        """
        return self.robot.get_camera_image(resolution)
    
    def get_pc(self):
        pc = self.robot.get_pointcloud() 
        return pc
    
    def get_imu(self):
        robot_state = self.robot.get_sportstate()
        return robot_state.imu_state
    
    def get_sportstate(self):
        robot_state = self.robot.get_sportstate()
        return robot_state
    
    def get_position(self):
        robot_state = self.robot.get_sportstate()
        return robot_state.position

    def get_odom(self):
        odom = self.robot.get_odom()
        return odom

    def get_pose(self):
        odom = self.robot.get_pose()
        return odom
    
    def get_joint(self):
        low_state = self.robot.get_lowstate()
        joint_pose = [low_state.motor_state[i].q for i in range(20)]
        return joint_pose
    
    def set_joint(self, joint_pos):
        self.robot.joint_control(joint_pos)
    
    def start(self):
        self.robot.start()
    
    def stop(self):
        self.robot.stop()
    
    def stand(self):
        self.set_joint_smooth(self.joint_stand, duration=2.0)
    
    def sit(self):
        self.set_joint_smooth(self.joint_sit, duration=2.0)

    def zero(self):
        self.set_joint_smooth(self.joint_zero, duration=2.0)
    
    def set_joint_smooth(self, target_joint, duration=2.0):
        current_joint = self.get_joint()
        self._interpolate_to_target(current_joint, target_joint, duration=duration)
    
    def set_coordinate(self, x, y):
        self.robot.get_to_coordination_goal(x, y)
    
    def move(self, vx, vy, vyaw):
        self.robot.move(vx, vy, vyaw)
    
    # def get_arm_joint(self):
    #     low_state = self.robot.get_lowstate()
    #     arm_joint_pose = [low_state.motor_state[i].q for i in range(10)]
    #     return arm_joint_pose
    
    # def set_arm_joint(self, arm_joint_pos):
    #     self.robot.arm_joint_control(arm_joint_pos)
    
    def get_arm(self):
        """Get current arm joint positions"""
        state = {
            'pos': self.arm_controller.get_current_position(),
            'vel': self.arm_controller.get_current_velocity()
        }
        return state
    
    def get_arm_controller_type(self):
        """Get current arm controller type"""
        return getattr(self, 'arm_controller_type', 'unknown')
    
    def get_arm_pid_gains(self):
        """Get current PID gains"""
        return self.arm_controller.get_pid_gains()
    
    def set_arm_pid_gains(self, kp=None, ki=None, kd=None):
        """Set PID gains for arm control
        
        Args:
            kp: Proportional gain(s)
            ki: Integral gain(s) 
            kd: Derivative gain(s)
        """
        return self.arm_controller.set_pid_gains(kp=kp, ki=ki, kd=kd)
    
    def get_arm_limits(self):
        """Get current PID controller limits"""
        # Note: PID controller doesn't expose limit getters directly
        # This is a placeholder that returns current PID gains instead
        kp, ki, kd = self.arm_controller.get_pid_gains()
        return {
            'kp': kp,
            'ki': ki,
            'kd': kd
        }
    
    def set_arm_output_limits(self, limits):
        """Set output limits for arm control"""
        return self.arm_controller.set_output_limits(limits)
    
    def set_arm_integral_limits(self, limits):
        """Set integral limits for arm control"""
        return self.arm_controller.set_integral_limits(limits)
    
    def reset_arm_pid(self):
        """Reset PID controller state"""
        return self.arm_controller.reset_pid_state()
    
    def set_arm(self, arm_joint_pos, wait=False, timeout=10.0):
        """Set arm target position
        
        Args:
            arm_joint_pos: Target joint positions (6 elements)
        """
        return self.arm_controller.set_target_position(arm_joint_pos, wait=wait, timeout=timeout)
    
    def set_arm_default(self, wait=False, timeout=10.0):
        """Set arm to default position"""
        default_pos = self.arm_hardware.get_default_position()
        return self.arm_controller.set_target_position(default_pos, wait=wait, timeout=timeout)
    
    def _interpolate_to_target(self, start_joint, target_joint, duration=2.0, frequency=50):
        """
        Interpolate from start joint positions to target joint positions over given duration
        
        Args:
            start_joint: Current joint positions (list of 20 values)
            target_joint: Target joint positions (list of 20 values) 
            duration: Total time in seconds (default 2.0s)
            frequency: Control frequency in Hz (default 50Hz)
        """
        start_joint = np.array(start_joint)[:12]
        target_joint = np.array(target_joint)
        
        steps = int(duration * frequency)
        dt = duration / steps
        
        for i in range(steps + 1):
            # Linear interpolation
            alpha = i / steps
            current_pos = start_joint + alpha * (target_joint - start_joint)
            
            # Send joint command
            self.robot.joint_control(current_pos.tolist())
            
            # Wait for next step
            if i < steps:  # Don't sleep after the last step
                time.sleep(dt)
    
    # gripper
    def open_gripper(self):
        if self.gripper is None:
            raise RuntimeError("Gripper not initialized")
        self.set_gripper(1.0)
    
    def close_gripper(self):
        if self.gripper is None:
            raise RuntimeError("Gripper not initialized")
        self.set_gripper(0.0)
    
    def get_gripper(self):
        if self.gripper is None:
            raise RuntimeError("Gripper not initialized")
        return self.gripper.get_state()
    
    def set_gripper(self, state):
        if self.gripper is None:
            raise RuntimeError("Gripper not initialized")
        self.gripper.control_Pos_Vel(state)
    
    # echo
    def echo(self, value = None):
        return value
    
    def start_policy(self):
        self.state = 'start'
    
    def stop_policy(self):
        self.state = 'stop'
    
    def get_state(self):
        return self.state