# Go2 Robot Interface Documentation

## 概述

`Go2` 类是 Unitree Go2 四足机器人的主要接口类，提供了对机器人运动控制、传感器数据获取、机械臂控制和夹爪操作的完整封装。

## 类定义

```python
class Go2:
    def __init__(self, ros_version='foxy', arm_controller_type='direct', init_gripper=True)
```

## 初始化参数

| 参数 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `ros_version` | str | 'foxy' | ROS版本 ('foxy' 或 'noetic') |
| `arm_controller_type` | str | 'direct' | 机械臂控制器类型 ('pid' 或 'direct') |
| `init_gripper` | bool | True | 是否初始化夹爪 |

### ROS版本说明
- **'foxy'**: 使用ROS 2 Foxy，支持完整的传感器集成（GoPro相机、夹爪等）
- **'noetic'**: 使用ROS 1 Noetic，主要用于机械臂控制

### 控制器类型说明
- **'pid'**: PID控制器，提供平滑运动控制（推荐，安全）
- **'direct'**: 直接控制器，立即定位（危险，无运动平滑）

## 公共方法

### 机器人控制

#### `start()`
启动机器人控制系统
- **返回值**: None

#### `stop()`
停止机器人控制系统
- **返回值**: None

#### `stand()`
使机器人站立到预设站立姿态
- **参数**: None
- **返回值**: None
- **持续时间**: 2.0秒

#### `sit()`
使机器人坐到预设坐姿姿态
- **参数**: None
- **返回值**: None
- **持续时间**: 2.0秒

#### `zero()`
使机器人关节归零
- **参数**: None
- **返回值**: None
- **持续时间**: 2.0秒

### 关节控制

#### `get_joint()`
获取当前关节位置
- **返回值**: list - 20个关节的位置值

#### `set_joint(joint_pos)`
直接设置关节位置（立即执行）
- **参数**:
  - `joint_pos`: list - 20个关节的目标位置
- **返回值**: None

#### `set_joint_smooth(target_joint, duration=2.0)`
平滑地设置关节位置
- **参数**:
  - `target_joint`: list - 目标关节位置（20个值）
  - `duration`: float - 运动持续时间（秒，默认2.0）
- **返回值**: None

### 运动控制

#### `move(vx, vy, vyaw)`
控制机器人移动
- **参数**:
  - `vx`: float - X方向速度
  - `vy`: float - Y方向速度
  - `vyaw`: float - 旋转角速度
- **返回值**: None

#### `set_coordinate(x, y)`
设置机器人目标坐标
- **参数**:
  - `x`: float - 目标X坐标
  - `y`: float - 目标Y坐标
- **返回值**: None

### 传感器数据

#### `get_imu()`
获取IMU数据
- **返回值**: IMU状态对象

#### `get_odom()`
获取里程计数据
- **返回值**: 里程计数据

#### `get_pose()`
获取机器人姿态
- **返回值**: 姿态数据

#### `get_pc()`
获取点云数据
- **返回值**: 点云数据

### 相机控制

#### `get_gopro_frame(num_frames=1)`
获取GoPro相机帧
- **参数**:
  - `num_frames`: int - 获取帧数（默认1）
- **返回值**: 相机帧数据

#### `get_camera_image(resolution=None)`
获取机器人机载相机图像（使用缓存机制以降低延迟）
- **参数**:
  - `resolution`: tuple 或 None - 目标分辨率 (width, height)，None表示原始分辨率
- **返回值**: numpy.ndarray 或 None - 指定分辨率的RGB图像数据或 None（如果获取失败）
- **说明**: 该方法使用后台缓存机制，延迟通常低于100ms，支持动态调整图像分辨率以适应网络传输需求

### 机械臂控制

#### `get_arm()`
获取当前机械臂关节位置和速度
- **返回值**: dict
  ```python
  {
      'pos': [float, float, float, float, float, float],  # 关节位置
      'vel': [float, float, float, float, float, float]   # 关节速度
  }
  ```

#### `set_arm(arm_joint_pos, wait=False, timeout=10.0)`
设置机械臂目标位置
- **参数**:
  - `arm_joint_pos`: list - 6个关节的目标位置
  - `wait`: bool - 是否等待运动完成（默认False）
  - `timeout`: float - 等待超时时间（秒，默认10.0）
- **返回值**: None

#### `set_arm_default(wait=False, timeout=10.0)`
设置机械臂到默认位置
- **参数**:
  - `wait`: bool - 是否等待运动完成（默认False）
  - `timeout`: float - 等待超时时间（秒，默认10.0）
- **返回值**: None

#### `get_arm_controller_type()`
获取当前机械臂控制器类型
- **返回值**: str - 控制器类型 ('pid', 'direct', 或 'unknown')

#### PID控制器相关方法（仅在PID模式下有效）

##### `get_arm_pid_gains()`
获取PID增益
- **返回值**: tuple - (kp, ki, kd)

##### `set_arm_pid_gains(kp=None, ki=None, kd=None)`
设置PID增益
- **参数**:
  - `kp`: float - 比例增益
  - `ki`: float - 积分增益
  - `kd`: float - 微分增益
- **返回值**: None

##### `get_arm_limits()`
获取PID控制器限制
- **返回值**: dict
  ```python
  {
      'kp': float,
      'ki': float,
      'kd': float
  }
  ```

##### `set_arm_output_limits(limits)`
设置输出限制
- **参数**:
  - `limits`: 限制值
- **返回值**: None

##### `set_arm_integral_limits(limits)`
设置积分限制
- **参数**:
  - `limits`: 限制值
- **返回值**: None

##### `reset_arm_pid()`
重置PID控制器状态
- **返回值**: None

### 夹爪控制

#### `open_gripper()`
张开夹爪
- **返回值**: None
- **异常**: 如果夹爪未初始化，抛出 RuntimeError

#### `close_gripper()`
闭合夹爪
- **返回值**: None
- **异常**: 如果夹爪未初始化，抛出 RuntimeError

#### `get_gripper()`
获取夹爪状态
- **返回值**: 夹爪状态
- **异常**: 如果夹爪未初始化，抛出 RuntimeError

#### `set_gripper(state)`
设置夹爪位置
- **参数**:
  - `state`: float - 夹爪位置（0.0-1.0）
- **返回值**: None
- **异常**: 如果夹爪未初始化，抛出 RuntimeError

### 状态管理

#### `get_state()`
获取机器人当前状态
- **返回值**: str - 状态 ('start', 'stop')

#### `start_policy()`
启动策略执行
- **返回值**: None

#### `stop_policy()`
停止策略执行
- **返回值**: None

### 其他方法

#### `echo(value=None)`
回显输入值（用于测试）
- **参数**:
  - `value`: any - 要回显的值
- **返回值**: any - 输入值

#### `remote_control_loop()`
启动远程控制循环
- **返回值**: None

## 使用示例

### 基本初始化
```python
# 初始化机器人（包含夹爪）
robot = Go2(ros_version='foxy', arm_controller_type='pid', init_gripper=True)

# 启动机器人
robot.start()

# 站立
robot.stand()

# 获取传感器数据
imu_data = robot.get_imu()
pose = robot.get_pose()

# 获取相机图像（原始分辨率）
camera_image = robot.get_camera_image()

# 获取相机图像（指定分辨率，用于网络传输）
low_res_image = robot.get_camera_image(resolution=(320, 240))
high_res_image = robot.get_camera_image(resolution=(1280, 720))

# 控制机械臂
robot.set_arm([0.0, -1.0, 2.0, 0.0, 0.0, 0.0])

# 控制夹爪
robot.open_gripper()
robot.close_gripper()

# 停止机器人
robot.stop()
```

### 不初始化夹爪
```python
robot = Go2(ros_version='foxy', init_gripper=False)
# 此时调用夹爪方法会抛出异常
```

## 注意事项

1. **安全**: 使用 'direct' 控制器时要特别小心，可能导致突然运动
2. **初始化**: 确保在调用方法前正确初始化机器人
3. **夹爪**: 如果未初始化夹爪，相关方法会抛出 RuntimeError
4. **ROS版本**: 根据实际环境选择正确的ROS版本
5. **同步**: 某些方法支持等待参数，确保运动完成后再执行后续操作

---

## 底层机器人接口 (GO2Interface)

`GO2Interface` 类是 Go2 机器人的底层 DDS 接口类，提供了直接的机器人控制和传感器数据访问功能。

### 类定义

```python
class GO2Interface:
    def __init__(self, ip_address)
```

### 初始化参数

| 参数 | 类型 | 描述 |
|------|------|------|
| `ip_address` | str | 机器人网络接口名称（如 'eth0'） |

### 运动控制方法

#### `move(vx, vy, vyaw)`
控制机器人移动速度
- **参数**:
  - `vx`: float - X方向速度，范围 [-2.5, 3.8] (m/s)
  - `vy`: float - Y方向速度，范围 [-1.0, 1.0] (m/s)
  - `vyaw`: float - 旋转角速度，范围 [-4, 4] (rad/s)
- **返回值**: None

#### `start(_index=0)`
启动机器人控制系统
- **参数**:
  - `_index`: int - 启动模式（0: 经典步行模式，1: 恢复站立模式）
- **返回值**: None

#### `stop()`
停止机器人控制系统
- **返回值**: None

#### `get_to_coordination_goal(x, y)`
使用PID控制机器人移动到指定坐标
- **参数**:
  - `x`: float - 目标X坐标
  - `y`: float - 目标Y坐标
- **返回值**: int - 0表示到达目标，1表示正在移动

#### `follow_coordination(x, y, yaw=624)`
让机器人跟随到指定坐标和朝向
- **参数**:
  - `x`: float - 目标X坐标
  - `y`: float - 目标Y坐标
  - `yaw`: float - 目标朝向角度（弧度），默认使用当前朝向
- **返回值**: None

#### `follow_trajactory(path)`
让机器人跟随指定的轨迹路径
- **参数**:
  - `path`: list - PathPoint_ 对象列表组成的轨迹
- **返回值**: None

### 关节控制方法

#### `joint_control(joint_pos)`
直接控制关节位置（低层控制）
- **参数**:
  - `joint_pos`: list - 12个关节的目标位置
- **返回值**: None

### 传感器数据获取方法

#### `get_odom()`
获取里程计数据
- **返回值**: Odometry_ 对象或 None（如果获取失败）

#### `get_pose()`
获取机器人姿态数据
- **返回值**: PoseStamped_ 对象或 None（如果获取失败）

#### `get_lowstate()`
获取机器人低层状态（包括所有关节状态）
- **返回值**: LowState_ 对象或 None（如果获取失败）

#### `get_pointcloud()`
获取激光雷达点云数据
- **返回值**: numpy.ndarray - 点云数据或 None（如果获取失败）

#### `get_sportstate()`
获取机器人运动状态
- **返回值**: SportModeState_ 对象或 None（如果获取失败）

### 相机控制方法

#### `get_camera_image(resolution=None)`
获取机器人相机图像（使用缓存机制以降低延迟）
- **参数**:
  - `resolution`: tuple 或 None - 目标分辨率 (width, height)，None表示原始分辨率
- **返回值**: numpy.ndarray - 指定分辨率的RGB图像数据或 None（如果获取失败）
- **说明**: 该方法使用后台线程持续获取图像并缓存最新帧，显著降低获取延迟，支持动态分辨率调整

### 底层命令初始化

#### `InitLowCmd()`
初始化低层控制命令结构
- **返回值**: None

### 订阅进程管理

#### `subscribe_process()`
启动后台订阅进程来接收传感器数据
- **返回值**: None

## GO2Interface 使用示例

```python
from .robot_interface import GO2Interface

# 初始化机器人接口
robot_interface = GO2Interface('eth0')

# 启动机器人
robot_interface.start()

# 控制移动
robot_interface.move(0.5, 0.0, 0.0)  # 向前移动

# 获取传感器数据
odom = robot_interface.get_odom()
pose = robot_interface.get_pose()
pointcloud = robot_interface.get_pointcloud()
sport_state = robot_interface.get_sportstate()

# 关节控制
joint_positions = [0.0] * 12  # 12个关节位置
robot_interface.joint_control(joint_positions)

# 移动到坐标
robot_interface.get_to_coordination_goal(1.0, 2.0)

# 获取相机图像
image = robot_interface.get_camera_image()

# 停止机器人
robot_interface.stop()
```

## GO2Interface 注意事项

1. **网络连接**: 确保机器人网络接口正确配置
2. **数据获取**: 传感器数据获取可能返回 None，需要适当的错误处理
3. **线程安全**: 内部使用多进程和队列，需要注意线程安全
4. **性能**: 低层控制方法执行频率较高，需要考虑实时性要求
5. **初始化顺序**: 必须先调用 start() 启动机器人，然后才能使用其他控制方法