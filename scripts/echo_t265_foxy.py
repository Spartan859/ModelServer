# #!/usr/bin/env python
# from unitree_sdk2py.core.channel import (
#     ChannelFactoryInitialize,
#     ChannelPublisher,
#     ChannelSubscriber,
# )
# from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
# from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_
# from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import (
#     PointCloud2_,
# )
# ChannelFactoryInitialize(0, 'eth0')
# odom_subscriber = ChannelSubscriber("rt/camera/odom/sample", Odometry_)
# odom_subscriber.Init(print)
# breakpoint()


import rclpy
from rclpy.node import Node


from nav_msgs.msg import Odometry



class EchoT265(Node):
    def __init__(self):
        super().__init__('echo_t265_foxy')
        self.t265_pose_sub = self.create_subscription(
            Odometry,
            'camera/odom/sample',
            self.t265_pose_callback,
            10
        )

        self.t265_pose_pub = self.create_publisher(
            Odometry,
            'camera/odom/sample_echo',
            10
        )

    def t265_pose_callback(self, msg: Odometry):
        print(msg)
        self.t265_pose_pub.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    node = EchoT265()
    rclpy.spin(node)
    rclpy.shutdown()
