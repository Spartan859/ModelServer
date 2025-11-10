import pyrealsense2 as rs
import numpy as np
import cv2
import time
import threading
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


class T265():
    def __init__(self, serial_number=None):
        self.pose_calibration_scale = 1.06
        self.fps = 200
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        if serial_number:
            self.config.enable_device(serial_number)
        
        self.config.enable_stream(rs.stream.pose)

        # Start pipeline
        profile = self.pipeline.start(self.config)
        ChannelFactoryInitialize(0, 'eth0')
        self.pose_publisher = ChannelPublisher("rt/slam/t265/pose", PoseStamped_)
        self.pose_publisher.Init()

        # 创建一个子进程
        # self.pose_publisher_thread = threading.Thread(target=self.publish_pose)
        # self.pose_publisher_thread.start()

    def join_pose_publisher_thread(self):
        self.pose_publisher_thread.join()

    def get_fps(self):
        return self.fps

    def get_pose(self):
        try:
            frames = self.pipeline.wait_for_frames()
            pose_frame = frames.get_pose_frame()
            timestamp = frames.get_timestamp()
            timestamp = 1e-3 * timestamp

            if not pose_frame:
                return None
            
            pose_data = pose_frame.get_pose_data()
            
            # Extract position and orientation from the pose data
            position = np.array([pose_data.translation.x, 
                                 pose_data.translation.y, 
                                 pose_data.translation.z])
            orientation = np.array([pose_data.rotation.w, 
                                    pose_data.rotation.x, 
                                    pose_data.rotation.y, 
                                    pose_data.rotation.z])

            # position = position * self.pose_calibration_scale

            print(f"Position: {position}, Orientation: {orientation}, Timestamp: {timestamp}")
            return position, orientation, timestamp
        
        except Exception as e:
            print(f"Error getting pose: {e}")
            return None
    
    def publish_pose(self):
        position, orientation, timestamp = self.get_pose()
        if position is not None and orientation is not None:
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
        else:
            print("Error getting pose")

    def stop_pipeline(self):
        self.pipeline.stop()


if __name__ == '__main__':
    # Replace with your actual device serial number if necessary
    t265_camera = T265('119622110447')
    t265_camera.join_pose_publisher_thread()




