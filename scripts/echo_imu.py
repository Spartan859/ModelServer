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

def print_imu(msg: LowState_):
    print(msg.imu_state)


ip_address = 'eth0'
ChannelFactoryInitialize(0, ip_address)

lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
lowstate_subscriber.Init(print_imu, 10)


import time
time.sleep(10)