import time
import numpy as np
from typing import Optional, Tuple
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_, Pose_, Point_, Quaternion_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import (
    WirelessController_,
    PathPoint_,
    SportModeState_,
    LowState_,
    LowCmd_,
    IMUState_
)
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_
from unitree_sdk2py.idl.builtin_interfaces.msg.dds_ import Time_


class IMUOdometry:
    """基于IMU数据的里程计算法类"""
    
    def __init__(self):
        # 位置状态 (x, y, z)
        self.position = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        # 速度状态 (vx, vy, vz)
        self.velocity = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        # 姿态四元数 (x, y, z, w)
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        
        # 上一次的时间戳
        self.last_timestamp: Optional[float] = None
        # 上一次的加速度（用于梯形积分）
        self.last_acceleration: Optional[np.ndarray] = None
        
        # 重力向量（会在初始化时自动估计）
        self.gravity_magnitude = 0.0  # 重力加速度大小
        self.gravity_direction = np.array([0.0, 0.0, -1.0], dtype=np.float64)  # 重力方向（初始假设向下）
        
        # 滤波参数
        self.acc_bias = np.array([0.0, 0.0, 0.0], dtype=np.float64)  # 加速度偏置
        self.gyro_bias = np.array([0.0, 0.0, 0.0], dtype=np.float64)  # 陀螺仪偏置
        
        # 初始化计数器（用于偏置和重力估计）
        self.init_count = 0
        self.init_samples = 200  # 增加样本数用于更准确的估计
        self.gravity_samples = []  # 用于收集重力样本
        
        # 速度漂移补偿参数
        self.velocity_decay_factor = 0.99  # 速度衰减因子，模拟摩擦力
        self.zero_velocity_threshold = 0.05  # 零速度检测阈值 (m/s²)
        self.zero_velocity_window = 20  # 零速度检测窗口大小
        self.recent_accelerations = []  # 最近的加速度历史
        self.velocity_correction_factor = 0.95  # 速度修正因子
        
        # 运动状态检测
        self.is_stationary = False
        self.stationary_count = 0
        self.stationary_threshold = 10  # 连续静止检测次数
        
    def quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """四元数乘法 q1 * q2"""
        w1, x1, y1, z1 = q1[3], q1[0], q1[1], q1[2]
        w2, x2, y2, z2 = q2[3], q2[0], q2[1], q2[2]
        
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        
        return np.array([x, y, z, w], dtype=np.float64)
    
    def quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """四元数转换为旋转矩阵"""
        x, y, z, w = q
        
        # 归一化四元数
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm > 0:
            x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # 构建旋转矩阵
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        ], dtype=np.float64)
        
        return R
    
    def update_orientation(self, gyroscope: np.ndarray, dt: float):
        """基于陀螺仪数据更新姿态"""
        # 去除偏置
        gyro_corrected = gyroscope - self.gyro_bias
        
        # 计算角度变化
        angle_change = gyro_corrected * dt
        angle_magnitude = np.linalg.norm(angle_change)
        
        if angle_magnitude > 1e-8:  # 避免除零
            # 构建增量四元数
            axis = angle_change / angle_magnitude
            half_angle = angle_magnitude / 2.0
            sin_half = np.sin(half_angle)
            cos_half = np.cos(half_angle)
            
            delta_q = np.array([
                axis[0] * sin_half,
                axis[1] * sin_half, 
                axis[2] * sin_half,
                cos_half
            ], dtype=np.float64)
            
            # 更新四元数
            self.orientation = self.quaternion_multiply(self.orientation, delta_q)
            
            # 归一化四元数
            norm = np.linalg.norm(self.orientation)
            if norm > 0:
                self.orientation = self.orientation / norm
    
    def update_position(self, acceleration: np.ndarray, dt: float):
        """基于加速度数据更新位置"""
        # 去除偏置
        acc_corrected = acceleration - self.acc_bias
        
        # 将加速度从机体坐标系转换到世界坐标系
        R = self.quaternion_to_rotation_matrix(self.orientation)
        acc_world = R @ acc_corrected
        
        # 减去重力影响（使用估计的重力向量）
        gravity_vector = self.gravity_direction * self.gravity_magnitude
        acc_world = acc_world - gravity_vector
        
        # 如果有上一次的加速度，使用梯形积分
        if self.last_acceleration is not None:
            # 梯形积分更新速度
            avg_acc = (acc_world + self.last_acceleration) / 2.0
            self.velocity += avg_acc * dt
        else:
            # 第一次更新，使用简单积分
            self.velocity += acc_world * dt
        
        # 更新位置（使用梯形积分）
        self.position += self.velocity * dt + 0.5 * acc_world * dt * dt
        
        # 保存当前加速度
        self.last_acceleration = acc_world.copy()
    
    def estimate_bias_and_gravity(self, acceleration: np.ndarray, gyroscope: np.ndarray):
        """估计IMU偏置和重力向量（静止状态下）"""
        if self.init_count < self.init_samples:
            # 累积陀螺仪偏置估计（静止时陀螺仪读数应该为0）
            self.gyro_bias += (gyroscope - self.gyro_bias) / (self.init_count + 1)
            
            # 收集加速度样本用于重力估计
            self.gravity_samples.append(acceleration.copy())
            
            self.init_count += 1
            
            # 显示初始化进度
            if self.init_count % 50 == 0:
                progress = (self.init_count / self.init_samples) * 100
                print(f"初始化进度: {progress:.1f}% ({self.init_count}/{self.init_samples})")
            
            # 在初始化完成时计算重力向量和加速度偏置
            if self.init_count == self.init_samples:
                self._finalize_gravity_estimation()
                print(f"初始化完成！")
                print(f"估计的重力大小: {self.gravity_magnitude:.3f} m/s²")
                print(f"估计的重力方向: [{self.gravity_direction[0]:.3f}, {self.gravity_direction[1]:.3f}, {self.gravity_direction[2]:.3f}]")
                print(f"陀螺仪偏置: [{self.gyro_bias[0]:.6f}, {self.gyro_bias[1]:.6f}, {self.gyro_bias[2]:.6f}] rad/s")
                print(f"加速度偏置: [{self.acc_bias[0]:.6f}, {self.acc_bias[1]:.6f}, {self.acc_bias[2]:.6f}] m/s²")
                print("开始实时位姿估计...")
            
            return False  # 还在初始化
        return True  # 初始化完成
    
    def _finalize_gravity_estimation(self):
        """完成重力向量估计"""
        if not self.gravity_samples:
            # 如果没有样本，使用默认值
            self.gravity_magnitude = 9.81
            self.gravity_direction = np.array([0.0, 0.0, -1.0], dtype=np.float64)
            self.acc_bias = np.array([0.0, 0.0, 0.0], dtype=np.float64)
            return
        
        # 计算所有样本的平均值（这应该是重力向量 + 偏置）
        gravity_samples = np.array(self.gravity_samples)
        mean_acceleration = np.mean(gravity_samples, axis=0)
        
        # 重力大小就是平均加速度的模长
        self.gravity_magnitude = np.linalg.norm(mean_acceleration)
        
        # 重力方向（归一化）
        if self.gravity_magnitude > 0:
            self.gravity_direction = mean_acceleration / self.gravity_magnitude
        else:
            self.gravity_direction = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        
        # 加速度偏置设为0（因为我们直接用测量值作为重力）
        # 这样做的假设是：静止时，加速度计测量值 = 重力向量 + 偏置
        # 我们把整个测量值当作重力，所以偏置为0
        self.acc_bias = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        
        # 清理样本数据
        self.gravity_samples.clear()
    
    def detect_zero_velocity(self, acceleration: np.ndarray) -> bool:
        """检测是否处于静止状态（零速度更新）"""
        # 将当前加速度添加到历史记录
        self.recent_accelerations.append(acceleration.copy())
        
        # 保持窗口大小
        if len(self.recent_accelerations) > self.zero_velocity_window:
            self.recent_accelerations.pop(0)
        
        # 需要足够的历史数据
        if len(self.recent_accelerations) < self.zero_velocity_window:
            return False
        
        # 计算最近加速度的方差
        recent_acc = np.array(self.recent_accelerations)
        acc_variance = np.var(recent_acc, axis=0)
        total_variance = np.sum(acc_variance)
        
        # 如果方差很小，认为是静止状态
        is_stationary = total_variance < self.zero_velocity_threshold
        
        if is_stationary:
            self.stationary_count += 1
        else:
            self.stationary_count = 0
        
        # 需要连续检测到静止状态
        self.is_stationary = self.stationary_count >= self.stationary_threshold
        
        return self.is_stationary
    
    def apply_velocity_corrections(self, dt: float):
        """应用速度漂移补偿"""
        # 1. 零速度更新 - 如果检测到静止，将速度设为零
        if self.is_stationary:
            # 逐渐将速度拉向零，而不是直接设为零
            self.velocity *= 0.8  # 快速衰减
        else:
            # 2. 速度衰减 - 模拟空气阻力和摩擦力
            self.velocity *= self.velocity_decay_factor
        
        # 3. 速度限制 - 防止速度过大
        max_velocity = 2.0  # 最大合理速度 (m/s)
        velocity_magnitude = np.linalg.norm(self.velocity)
        if velocity_magnitude > max_velocity:
            self.velocity = self.velocity / velocity_magnitude * max_velocity
            print(f"速度限制激活，当前速度被限制到: {max_velocity} m/s")
    
    def update(self, imu_state: IMUState_, timestamp: float) -> Tuple[np.ndarray, np.ndarray]:
        """更新里程计状态"""
        acceleration = np.array(imu_state.accelerometer, dtype=np.float64)
        gyroscope = np.array(imu_state.gyroscope, dtype=np.float64)
        
        # 如果是第一次更新
        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            return self.position.copy(), self.orientation.copy()
        
        # 计算时间差
        dt = timestamp - self.last_timestamp
        if dt <= 0 or dt > 0.1:  # 防止异常时间差
            self.last_timestamp = timestamp
            return self.position.copy(), self.orientation.copy()
        
        # 估计偏置和重力（前几个样本）
        if not self.estimate_bias_and_gravity(acceleration, gyroscope):
            self.last_timestamp = timestamp
            return self.position.copy(), self.orientation.copy()
        
        # 检测零速度状态（用于漂移补偿）
        self.detect_zero_velocity(acceleration)
        
        # 更新姿态
        self.update_orientation(gyroscope, dt)
        
        # 更新位置
        self.update_position(acceleration, dt)
        
        # 应用速度漂移补偿
        self.apply_velocity_corrections(dt)
        
        # 更新时间戳
        self.last_timestamp = timestamp
        
        return self.position.copy(), self.orientation.copy()


# 全局里程计实例
imu_odometry = IMUOdometry()


def publish_pose(msg: LowState_):
    """处理IMU数据并发布机器人pose"""
    t = msg.tick / 1000  # tick是整数，代表1ms，而且都是偶数，说明严格2ms收到一次消息，频率严格等于500Hz
    
    # 获取IMU状态
    imu_state = msg.imu_state
    
    # 打印调试信息（可选）
    if False:  # 设置为True来启用调试输出
        print("time_msg: ", t)
        print("time_system: ", time.monotonic())
        print("quaternion: ", imu_state.quaternion)  # list, len = 4
        print("gyroscope: ", imu_state.gyroscope)  # list, len = 3
        print("accelerometer: ", imu_state.accelerometer)  # list, len = 3
        print("rpy: ", imu_state.rpy)  # list, len = 3
        print("temperature: ", imu_state.temperature)
    
    # 使用里程计算法更新pose
    calculated_position, calculated_orientation = imu_odometry.update(imu_state, t)
    
    # 创建时间戳
    sec = int(msg.tick // 1000)
    nanosec = int((msg.tick % 1000) * 1000000)
    
    # 创建位置点 - 使用计算出的位置
    position = Point_(
        x=float(calculated_position[0]),
        y=float(calculated_position[1]), 
        z=float(calculated_position[2])
    )
    
    # 创建姿态四元数 - 使用计算出的姿态
    orientation = Quaternion_(
        x=float(calculated_orientation[0]),
        y=float(calculated_orientation[1]),
        z=float(calculated_orientation[2]),
        w=float(calculated_orientation[3])
    )
    
    # 构建并发布pose消息
    pose_stamped = PoseStamped_(
        header=Header_(
            stamp=Time_(
                sec=sec,
                nanosec=nanosec
            ),
            frame_id='odom'
        ),
        pose=Pose_(
            position=position,
            orientation=orientation
        )
    )
    
    # 发布pose
    pose_publisher.Write(pose_stamped)
    
    # 打印计算结果（可选）
    if True:  # 设置为False来禁用输出
        velocity = imu_odometry.velocity
        stationary_status = " [静止]" if imu_odometry.is_stationary else ""
        
        # 准备位置和速度字符串
        position_str = f"Position: x={calculated_position[0]:+07.4f} (vx={velocity[0]:+07.4f}), y={calculated_position[1]:+07.4f} (vy={velocity[1]:+07.4f}), z={calculated_position[2]:+07.4f} (vz={velocity[2]:+07.4f}){stationary_status}"
        
        # 准备姿态字符串
        orientation_str = f"Orientation: x={calculated_orientation[0]:+06.3f}, y={calculated_orientation[1]:+06.3f}, z={calculated_orientation[2]:+06.3f}, w={calculated_orientation[3]:+06.3f}"
        
        # 同时打印位置和姿态
        print(f"{position_str}\n{orientation_str}")


ip_address = 'eth0'
ChannelFactoryInitialize(0, ip_address)
pose_publisher = ChannelPublisher("rt/utlidar/robot_pose", PoseStamped_)
pose_publisher.Init()

lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
lowstate_subscriber.Init(publish_pose, 10)

print("IMU Odometry system started. Initializing...")
print("The first 200 samples will be used for bias and gravity estimation.")
print("Please keep the robot stationary during initialization.")
print("This will automatically measure the actual gravity magnitude from your sensor.")

time.sleep(10000)  # 给更多时间进行初始化

