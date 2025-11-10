from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import (
    PointCloud2_,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import (
    WirelessController_,
    PathPoint_,
    SportModeState_,
    LowState_,
    LowCmd_,
)
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
)
from unitree_sdk2py.idl.default import (
    unitree_go_msg_dds__LowCmd_
)

class T265Publisher:
    def __init__(self):
        self.t265_pose_publisher = ChannelPublisher("rt/t265/pose", PoseStamped_)
        self.t265_pose_publisher.Init()

    def publish_pose(self, pose):
        pose_stamped = PoseStamped_(
            header=Header_(
                stamp=Time_(
                    sec=int(timestamp),
                    nanosec=int((timestamp - int(timestamp)) * 1e9)
                ),
                frame_id='odom'
            ),
            pose=Pose_(
                position=Point_(
                    x=position[0],
                    y=position[1],
                    z=position[2]
                ),
                orientation=Quaternion_(
                    x=orientation[0],
                    y=orientation[1],
                    z=orientation[2],
                    w=orientation[3]
                )
            )
        )
        self.pose_publisher.Write(pose_stamped)
        self.t265_pose_publisher.Write(pose)