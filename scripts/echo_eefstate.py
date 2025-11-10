# from unitree_sdk2py.core.channel import (
#     ChannelFactoryInitialize,
#     ChannelPublisher,
#     ChannelSubscriber,
# )
# from unitree_sdk2py.idl.unitree_go.msg.dds_ import (
#     WirelessController_,
#     PathPoint_,
#     SportModeState_,
#     LowState_,
#     LowCmd_,
# )
# from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
# from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import TwistStamped_, TwistWithCovarianceStamped_



# ChannelFactoryInitialize(0, 'eth0')

# publisher = ChannelPublisher("rt/go2_ahex/eef_traj", TwistWithCovarianceStamped_)
# publisher.Init()

# def eef_state_cb(msg: TwistStamped_):
#     print(msg)
#     msg.eef_pose[0] = msg.eef_pose[0] - 0.0
#     traj_msg = TwistWithCovarianceStamped_([msg])
#     publisher.Write(traj_msg)

# eef_state_sub = ChannelSubscriber("rt/go2_ahex/eef_state", TwistStamped_)
# eef_state_sub.Init(eef_state_cb)
# breakpoint()

import rclpy
from rclpy.node import Node
import copy

from robot_state.msg import EEFState, EEFTraj


class EchoEEFState(Node):
    def __init__(self):
        super().__init__('echo_eef_state')
        self.eef_state_sub = self.create_subscription(
            EEFState,
            'go2_ahex/eef_state',
            self.eef_state_callback,
            10
        )

        self.eef_traj_pub = self.create_publisher(
            EEFTraj,
            'go2_ahex/eef_traj',
            10
        )

    def eef_state_callback(self, msg: EEFState):
        # msg.eef_pose[0] = msg.eef_pose[0] - 0.0
        echo_msg = EEFTraj()
        echo_msg.traj = [copy.deepcopy(msg) for i in range(50)]
        for i in range(len(echo_msg.traj)):
            echo_msg.traj[i].tick += i * 100
            echo_msg.traj[i].eef_pose[0] = echo_msg.traj[i].eef_pose[0] + i * 0.002
            
        print('tick:', msg.tick)
        print('eef_pose:', msg.eef_pose[:3])
        self.eef_traj_pub.publish(echo_msg)

if __name__ == '__main__':
    rclpy.init()
    node = EchoEEFState()
    rclpy.spin(node)
    rclpy.shutdown()
