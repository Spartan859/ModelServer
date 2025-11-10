#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-05-08
################################################################

import copy
import json
import threading
import time
import numpy as np

import rospy
from sensor_msgs.msg import JointState
from xpkg_arm_msgs.msg import XmsgArmJointParam
from xpkg_arm_msgs.msg import XmsgArmJointParamList


class JointTrack:

    def __init__(self):
        ### ros node
        rospy.init_node("joint_track", anonymous=True)
        self.__rate = rospy.Rate(50.0)

        ### publisher
        self.__joint_ctrl_pub = rospy.Publisher(
            '/xtopic_arm/joints_cmd',
            XmsgArmJointParamList,
            queue_size=10,
        )

        ### subscriber
        self.__joint_state_sub = rospy.Subscriber(
            '/xtopic_arm/joint_states',
            JointState,
            self.__joint_state_callback,
        )
        self.__joint_state_sub

        ### variable
        self.__delta_lr_max = 0.10 * 1
        self.__delta_dm_max = 0.05 * 4
        # target
        self.__target_change_thresh = 0.2
        # deadzone - for each joint
        self.__deadzone_thresh = np.array([0.02, 0.02, 0.02, 0.02, 0.02, 0.02])

        self.__target_joints = [
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.array(
                [0.5, 0.523598775598, 2.09439265359, 1.57, -1.0472, 0.0]),
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.array(
                [-0.5, 0.523598775598, 2.09439265359, -1.57, -1.0472, 0.0]),
        ]

        self.__target_joints_idx = 0
        self.__joint_num = self.__target_joints[0].shape[0]
        # state
        self.__pos_lock = threading.Lock()
        self.__cur_pos = np.zeros(self.__joint_num)
        # control
        self.__ctrl_msg = XmsgArmJointParamList()
        for _ in range(self.__joint_num):
            param = XmsgArmJointParam()
            param.mode = "position_mode"
            # param.mode = "mit_mode"
            param.position = 0.0
            param.velocity = 0.0
            param.effort = 0.0
            param.extra_param = json.dumps({"braking_state": False, "mit_kp": 4.0, "mit_kd": 0.5})
            self.__ctrl_msg.joints.append(param)

        # finish log
        print("joint track node init finished")

    def __joint_state_callback(self, msg: JointState):
        # update state
        with self.__pos_lock:
            self.__cur_pos = np.array(msg.position)

    def work(self):
        while not rospy.is_shutdown():
            with self.__pos_lock:
                delta_pos = self.__target_joints[
                    self.__target_joints_idx] - self.__cur_pos

            # update target
            delta_pos_norm = np.linalg.norm(delta_pos)
            if delta_pos_norm < self.__target_change_thresh:
                self.__target_joints_idx = (self.__target_joints_idx +
                                            1) % len(self.__target_joints)

            # publish control message
            inc_pos = copy.deepcopy(delta_pos)

            # first 3 joints
            # inc_pos_norm = np.fabs(inc_pos[:3]).max()
            # if inc_pos_norm > 0:  # 避免除零错误
            #     inc_pos[:3] = inc_pos[:3] / inc_pos_norm * self.__delta_lr_max
            inc_pos[:3] = np.clip(
                inc_pos[:3],
                -self.__delta_lr_max,
                self.__delta_lr_max,
            )

            # last 3 joints
            inc_pos[3:] = np.clip(
                inc_pos[3:],
                -self.__delta_dm_max,
                self.__delta_dm_max,
            )
            control_pos = self.__cur_pos + inc_pos
            # deadzone
            for i in range(3):
                if abs(delta_pos[i]) < self.__deadzone_thresh[i]:
                    control_pos[i] = self.__target_joints[self.__target_joints_idx][i]
            # for i in range(6):
                # control_pos[i] = self.__target_joints[self.__target_joints_idx][i]
            print(f"control_pos: {control_pos}")

            for i in range(self.__joint_num):
                self.__ctrl_msg.joints[i].position = control_pos[i]
            self.__joint_ctrl_pub.publish(self.__ctrl_msg)

            # sleep
            self.__rate.sleep()


def main():
    joint_track = JointTrack()
    joint_track.work()


if __name__ == '__main__':
    main()

