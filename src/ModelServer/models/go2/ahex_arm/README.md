# AhexArm Control System

A modular and extensible robotic arm control system with advanced n-th order differential control algorithms.

## Architecture Overview

The system has been refactored into three main components for better modularity and extensibility:

### 1. DifferentialController (`differential_controller.py`)
- **Purpose**: Hardware-agnostic differential control algorithm
- **Features**: 
  - N-th order derivative constraints (position, velocity, acceleration, jerk, etc.)
  - Configurable saturation limits and control gains
  - Thread-safe real-time control loop
  - Works with any hardware implementing `HardwareInterface`

### 2. AhexArmHardware (`ahex_arm_hardware.py`)
- **Purpose**: Hardware-specific interface for Ahex robotic arm
- **Features**:
  - ROS communication (joint state feedback, position commands)
  - Automatic driver management
  - Hardware status monitoring
  - Implements `HardwareInterface` abstract base class

### 3. AhexArm (`ahex_arm.py`)
- **Purpose**: Complete integrated system (backward compatible)
- **Features**:
  - Combines hardware interface and differential controller
  - Plug-and-play initialization
  - Maintains API compatibility with original implementation

## Usage Examples

### Basic Usage (Backward Compatible)
```python
from ahex_arm import AhexArm

# Initialize with default settings (4th order control)
arm = AhexArm()

# Set target position (non-blocking)
arm.set_target_position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# Set target position with blocking wait
success = arm.set_target_position([1.0, 0.5, 0.0, 0.0, 0.0, 0.0], wait=True, timeout=10.0)

# Check if target is reached
if arm.is_target_reached():
    print("Target reached!")

# Get current position
current_pos = arm.get_current_position()
```

### Advanced Usage (Decoupled Architecture)
```python
from ahex_arm_hardware import AhexArmHardware
from differential_controller import DifferentialController

# Initialize hardware interface
hardware = AhexArmHardware()

# Initialize controller with custom parameters
controller = DifferentialController(
    hardware,
    control_frequency=100.0,  # 100 Hz control loop
    n_derivatives=3           # Position + velocity + acceleration control
)

# Configure control parameters
controller.set_derivative_limits([2.0, 1.0, 0.5])  # [position, velocity, acceleration]
controller.set_control_gains([3.0, 1.5])           # [position_error_gain, velocity_error_gain]

# Start control process
controller.start_control_process()

# Use the controller
controller.set_target_position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
```

### Using with Go2 Robot System

#### Integrated Mode (Default)
```python
from go2 import Go2

# Initialize with integrated arm control (backward compatible)
robot = Go2(arm_control_mode='integrated')

# Use arm as before
robot.set_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
current_pos = robot.get_arm()
```

#### Decoupled Mode (Advanced)
```python
from go2 import Go2

# Initialize with decoupled arm control
robot = Go2(arm_control_mode='decoupled')

# Basic arm control (same API)
robot.set_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# Advanced control parameters (only in decoupled mode)
robot.set_arm_control_parameters(
    derivative_limits=[2.0, 1.0, 0.5, 0.2],  # [position, velocity, acceleration, jerk]
    control_gains=[2.0, 1.0, 0.5]            # [pos_error_gain, vel_error_gain, acc_error_gain]
)

# Get control system information
info = robot.get_arm_control_info()
print(f"Control mode: {info['mode']}")
print(f"Derivatives: {info['derivative_names']}")

# Access underlying components for advanced usage
hardware = robot.get_arm_hardware()
controller = robot.get_arm_controller()
```

## Control Modes

The system supports different levels of control sophistication:

| n_derivatives | Controlled Variables | Control Type | Use Case |
|---------------|---------------------|--------------|----------|
| 0 | None | Direct tracking | Testing, emergency |
| 1 | Position | P control | Simple positioning |
| 2 | Position + Velocity | PD control | Basic motion control |
| 3 | Position + Velocity + Acceleration | Advanced control | Smooth motion |
| 4 | Position + Velocity + Acceleration + Jerk | Ultra-smooth control | High precision tasks |
| 5+ | Higher-order derivatives | Research applications | Specialized requirements |

## Extending to Other Hardware

To use the differential controller with different robotic hardware:

1. **Implement HardwareInterface**:
```python
from differential_controller import HardwareInterface
import numpy as np

class MyRobotHardware(HardwareInterface):
    def get_current_position(self):
        # Return current joint positions as np.array
        return np.array([...])
    
    def send_position_command(self, positions):
        # Send position command to your hardware
        pass
    
    def get_joint_count(self):
        # Return number of joints
        return 6
    
    def is_ready(self):
        # Return True when hardware is ready
        return True
```

2. **Use with DifferentialController**:
```python
hardware = MyRobotHardware()
controller = DifferentialController(hardware, n_derivatives=4)
controller.start_control_process()
controller.set_target_position([...])
```

## Configuration Parameters

### Derivative Limits
Controls maximum allowed values for each derivative:
- `position_max`: Maximum position change per step
- `velocity_max`: Maximum velocity
- `acceleration_max`: Maximum acceleration  
- `jerk_max`: Maximum jerk (rate of acceleration change)

### Control Gains
Controls responsiveness of each error term:
- `k_position`: Position error gain
- `k_velocity`: Velocity error gain
- `k_acceleration`: Acceleration error gain

### Control Frequency
- Higher frequency → smoother control, more CPU usage
- Lower frequency → less smooth control, less CPU usage
- Typical range: 10-200 Hz

## Migration Guide

### From Original AhexArm
The new `AhexArm` class maintains full backward compatibility:

```python
# Old code (still works)
from arm_control_thread import AhexArm
arm = AhexArm()
arm.set_target_position([...])

# New code (same API)
from ahex_arm import AhexArm  
arm = AhexArm()
arm.set_target_position([...])
```

### Accessing New Features
Use the decoupled architecture for advanced features:

```python
# Get underlying components
hardware = arm.get_hardware_interface()
controller = arm.get_controller()

# Configure advanced parameters
controller.set_derivative_limits([2.0, 1.0, 0.5, 0.2])
controller.set_control_gains([3.0, 2.0, 1.0])
```

This modular architecture provides flexibility for different use cases while maintaining compatibility with existing code.
