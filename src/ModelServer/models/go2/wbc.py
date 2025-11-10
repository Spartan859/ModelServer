import rclpy
from rclpy.node import Node
from robot_state.msg import EEFTraj, EEFState
from ModelServer import go2_noetic, go2_foxy
from hex_utils import DynUtil
from hex_utils import HexArmState, HexCartPose
from transforms3d import affines, quaternions, euler, axangles
import numpy as np
from unitree_go.msg import (
    WirelessController,
    LowState,
    LowCmd,
    MotorCmd,
    SportModeState
)
from sensor_msgs.msg import JointState
from realtime_traj import RealtimeTraj
import time
from gripper.gripper_control import Gripper


def print_pose(pose):
    # 把4*4矩阵转为xyz, 欧拉角
    translation = pose[:3, 3]
    rotation_rpy = euler.mat2euler(pose[:3, :3])
    # 转为list之后，小数点前显示两位，统一保留小数点后四位
    print(f"(xyz): [{translation[0]:6.4f}, {translation[1]:6.4f}, {translation[2]:6.4f}] | (rpy): [{rotation_rpy[0]:6.4f}, {rotation_rpy[1]:6.4f}, {rotation_rpy[2]:6.4f}]")




class WBCNode(Node):
    def __init__(
        self,
        low_state_history_depth: int = 1,  # changed from 10, doesn't make much of a difference
        pose_estimator: str = "imu",
    ):
        super().__init__("wbc_node")
        self.pose_estimator = pose_estimator
        self.gripper = Gripper()

        self.arm2base = affines.compose(
            T=np.array([0.108, 0.0, 0.094]),
            R=np.identity(3),
            Z=np.ones(3),
        )
        self.arm2base_inv = np.linalg.inv(self.arm2base)

        self.center2base = affines.compose(
            T=np.array([0.4, 0.0, 0.5]),
            R=np.identity(3),
            Z=np.ones(3),
        )
        self.center2base_inv = np.linalg.inv(self.center2base)

        self.tcp2ee = affines.compose(
            T=np.zeros(3),
            R=np.array(
                [
                    # [0.0, 0.0, 1.0],
                    # [-1.0, 0.0, 0.0],
                    # [0.0, -1.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [-1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                ]
            ),
            Z=np.ones(3),
        )
        self.tcp2ee_inv = np.linalg.inv(self.tcp2ee)
        self.radius = 0.2

        self.robot_pose_time = -1.0
        self.realtime_target_traj = RealtimeTraj()
        self.robot_pose = np.identity(4, dtype=np.float32)
        self.arm_dof_pos = np.zeros(6)
        self.arm_dof_vel = np.zeros(6)
        self.arm_dof_tau = np.zeros(6)
        self.time_base = int(time.time() // 1e5 * 1e5)
        self.gripper_pos = 1.0
        self.arm_and_robot_move = True

        self.dyn_util = DynUtil('/home/unitree/projects/ModelServer/src/ModelServer/models/go2/ahex_arm/xpkg_urdf_saberd6x/urdf/xpkg_urdf_saberd6x.urdf', 'link_6')
        self.hexarmstate = HexArmState(pos=np.zeros(6), vel=np.zeros(6), eff=np.zeros(6))

        if self.pose_estimator == "imu":
            self.robot_pose_sub = self.create_subscription(
                SportModeState,
                "sportmodestate",
                self.robot_pose_cb,
                10,
            )
        # self.arm_state_sub = self.create_subscription(
        #     JointState, "xtopic_arm/joint_states", self.arm_state_cb, low_state_history_depth
        # )

        self.eef_traj_sub = self.create_subscription(EEFTraj, "go2_arm/eef_traj", self.eef_traj_callback, 10)
        self.eef_state_pub = self.create_publisher(EEFState, "go2_arm/eef_state", 10)
        self.create_timer(0.1, self.timer_callback)
        self.arm_dof_pos = go2_noetic.get_arm()['pos']
        self.last_time = time.time()
        self.this_time = time.time()
        self.max_wbc_time = 0
        self.max_t0_5 = 0
        self.max_t0_6 = 0
        self.max_t1 = 0
        self.max_t1_5 = 0
        self.max_t2 = 0
        self.max_t3 = 0
        self.max_t3_5 = 0
        self.max_t4 = 0
        self.max_t5 = 0
        self.max_t6 = 0
        self.max_t7 = 0

    def robot_pose_cb(self, msg: SportModeState):
        self.sport_mode_state_msg = msg
        self.robot_pose_time = msg.stamp.sec + msg.stamp.nanosec / 1e9 - self.time_base

        # self.robot_pose = affines.compose(
        #     T=np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]),
        #     R=quaternions.quat2mat(
        #         [
        #             msg.pose.orientation.w,
        #             msg.pose.orientation.x,
        #             msg.pose.orientation.y,
        #             msg.pose.orientation.z,
        #         ]
        #     ),
        #     Z=np.ones(3),
        # )
    
    # def arm_state_cb(self, msg: JointState):
    #     self.arm_dof_pos = np.array(msg.position)
    #     self.arm_dof_vel = np.array(msg.velocity)
    #     self.arm_dof_tau = np.array(msg.effort)

    # def eef_traj_callback(self, msg: EEFTraj):
    #     self.eef_traj = msg
    
    def eef_traj_callback(self, msg: EEFTraj):
        translations = []
        quaternions_wxyz = []
        gripper_pos = []
        timestamps = []
        for i in range(len(msg.traj)):
            translations.append(msg.traj[i].eef_pose[:3])
            quaternions_wxyz.append(msg.traj[i].eef_pose[3:])
            gripper_pos.append(msg.traj[i].gripper_pos)
            timestamps.append(float(msg.traj[i].tick) / 1000)
        translations = np.array(translations)
        quaternions_wxyz = np.array(quaternions_wxyz)
        gripper_pos = np.array(gripper_pos)
        timestamps = np.array(timestamps)
        self.realtime_target_traj.update(
            translations,
            quaternions_wxyz,
            gripper_pos,
            timestamps,
            # self.prev_obs_tick_s,
            self.robot_pose_time,
            adaptive_latency_matching=True,  # TODO: add to config
            smoothen_time=0.1  # TODO: add to config
        )

    def timer_callback(self):
        self.this_time = time.time()
        # print(f"time: {self.this_time - self.last_time:.6f}")
        self.last_time = self.this_time
        t0 = time.time()
        self.arm_dof_pos = go2_noetic.get_arm()['pos']
        self.current_tcp_pose = self.get_tcp_pose()
        t0_5 = time.time()
        self.current_tcp_pose_time = self.robot_pose_time

        msg = EEFState()
        # eef_pose = np.concatenate([self.target_translation, target_quat_wxyz])
        current_quat_wxyz = quaternions.mat2quat(self.current_tcp_pose[:3, :3])
        t0_6 = time.time()
        msg.eef_pose = np.concatenate([self.current_tcp_pose[:3, 3], current_quat_wxyz])
        msg.tick = int(self.current_tcp_pose_time * 1000)
        msg.gripper_pos = self.gripper_pos
        self.eef_state_pub.publish(msg)
        t1 = time.time()

        futuretime = self.robot_pose_time + 0.1
        if len(self.realtime_target_traj.timestamps) == 0:
            return
            future_tcp_pose = self.current_tcp_pose
        else:
            translation, quaternion_wxyz, gripper_pos = self.realtime_target_traj.interpolate(futuretime)  # time: 2ms
            t1_5 = time.time()
            future_tcp_pose = affines.compose(
                T=translation, R=quaternions.quat2mat(quaternion_wxyz), Z=np.ones(3)
            )  # time: 3ms
        t2 = time.time()
        # self.robot_pose @ self.arm2base @ future_ee2arm @ self.tcp2ee == future_tcp_pose
        # Solve for future_ee2arm: future_ee2arm = inv(arm2base) @ inv(robot_pose) @ future_tcp_pose @ inv(tcp2ee)
        future_arm_pose = future_tcp_pose.copy()


        # if gripper_pos > 0.95:
        #     self.center_pose = self.robot_pose @ self.center2base  # 球心
        #     # x = (future_tcp_pose[:3, 3] - self.center_pose[:3, 3]).copy()
        #     # x_arm = x / np.linalg.norm(x) * self.radius
        #     # x_arm[2] = x[2]
        #     # x[:2] = x[:2] - x_arm[:2]
        #     # x[2] = 0.0
        #     # go2_foxy.async_mode.move(x[0], x[1], 0.0)
        #     # future_arm_pose[:3, 3] = self.center_pose[:3, 3] + x_arm.copy()

        #     xy = (future_tcp_pose[:2, 3] - self.center_pose[:2, 3]).copy()
        #     xy_arm = xy / np.linalg.norm(xy) * self.radius
        #     xy_robot = xy - xy_arm
        #     go2_foxy.async_mode.move(xy_robot[0] * 2, xy_robot[1] * 2, 0.0)
        #     future_arm_pose[:2, 3] = self.center_pose[:2, 3] + xy_arm.copy()

        future_ee2arm = self.arm2base_inv @ self.robot_pose_inv @ future_arm_pose @ self.tcp2ee_inv
        t3 = time.time()

        ## 这里用inverse_kinematics解出机械臂关节角
        # 从future_ee2arm变换矩阵中提取位置和四元数
        future_ee_pos = future_ee2arm[:3, 3]
        future_ee_quat = quaternions.mat2quat(future_ee2arm[:3, :3])  # time: 5ms
        t3_5 = time.time()
        
        # 创建目标末端执行器姿态
        target_pose = HexCartPose(
            pos=future_ee_pos,
            quat=future_ee_quat,
        )
        
        t4 = time.time()
        # 求解逆运动学
        success, tar_state, err_norm = self.dyn_util.inverse_kinematics(
            target_pose, self.hexarmstate)  # most time-consuming: 15ms
        t5 = time.time()

        # 如果逆运动学求解成功，更新目标关节角度
        if success:
            target_joint_pos = tar_state.get_pos()
            # go2_noetic.async_mode.set_arm(target_joint_pos)  # most time-consuming: 25ms
            go2_noetic.set_arm(target_joint_pos)  # most time-consuming: 25ms
            # self.get_logger().info(f"IK solved successfully, error norm: {err_norm}")
        else:
            # 如果求解失败，保持当前关节位置
            target_joint_pos = self.arm_dof_pos
            self.get_logger().warn(f"IK failed, error norm: {err_norm}, keeping current joint positions")

        t6 = time.time()

        self.gripper.control_Pos_Vel(gripper_pos)
        self.gripper_pos = gripper_pos

        # vx, vy, vyaw = 0.0, 0.0, 0.0
        self.arm_dof_pos = target_joint_pos

        # go2_foxy.async_mode.move(vx, vy, vyaw)
        # go2_noetic.async_mode.set_arm(target_joint_pos)
        t7 = time.time()
        self.max_wbc_time = max(self.max_wbc_time, time.time() - self.this_time)
        self.max_t0_5 = max(self.max_t0_5, t0_5 - t0)
        self.max_t0_6 = max(self.max_t0_6, t0_6 - t0_5)
        self.max_t1_5 = max(self.max_t1_5, t1_5 - t1)
        self.max_t3_5 = max(self.max_t3_5, t3_5 - t3)
        self.max_t1 = max(self.max_t1, t1 - t0_6)
        self.max_t2 = max(self.max_t2, t2 - t1)
        self.max_t3 = max(self.max_t3, t3 - t2)
        self.max_t4 = max(self.max_t4, t4 - t3)
        self.max_t5 = max(self.max_t5, t5 - t4)
        self.max_t6 = max(self.max_t6, t6 - t5)
        self.max_t7 = max(self.max_t7, t7 - t6)

        # print(f"max wbc time: {self.max_wbc_time:.6f}")
        print(f"{self.max_t0_5:.6f},{self.max_t0_6:.6f},{self.max_t1:.6f},{self.max_t1_5:.6f},{self.max_t2:.6f}")
        # print(f"max t2: {self.max_t2:.6f}")
        # print(f"max t3: {self.max_t3:.6f}")
        # print(f"max t4: {self.max_t4:.6f}")
        # print(f"max t5: {self.max_t5:.6f}")
        # print(f"max t6: {self.max_t6:.6f}")
        # print(f"max t7: {self.max_t7:.6f}")

    def get_tcp_pose(self) -> np.ndarray:
        """
        In the iphone pose frame
        """
        # arx5_ee_pose = self.arx5_solver.forward_kinematics(arm_dof_pos)
        arm_dof_pos = self.arm_dof_pos
        self.hexarmstate.set_pos(arm_dof_pos)
        pose = self.dyn_util.forward_kinematics(self.hexarmstate)[-1]
        self.ee2arm = affines.compose(
            # T=arx5_ee_pose[:3], R=euler.euler2mat(*arx5_ee_pose[3:]), Z=np.ones(3)
            T=pose.get_pos(), R=quaternions.quat2mat(pose.get_quat()), Z=np.ones(3)
        )
        # 直接从position和quaternion计算robot_pose和其逆矩阵，避免矩阵求逆操作
        robot_position = np.array(self.sport_mode_state_msg.position)
        robot_rotation_mat = quaternions.quat2mat(self.sport_mode_state_msg.imu_state.quaternion)
        
        self.robot_pose = affines.compose(
            T=robot_position,
            R=robot_rotation_mat,
            Z=np.ones(3),
        )
        
        # 高效计算逆变换矩阵：对于刚体变换 T = [R t; 0 1]，其逆为 T^-1 = [R^T -R^T*t; 0 1]
        robot_rotation_mat_T = robot_rotation_mat.T  # R的转置
        robot_position_inv = -robot_rotation_mat_T @ robot_position  # -R^T * t
        
        self.robot_pose_inv = affines.compose(
            T=robot_position_inv,
            R=robot_rotation_mat_T,
            Z=np.ones(3),
        )
        # print_pose(self.robot_pose @ self.arm2base @ self.ee2arm @ self.tcp2ee)
        return self.robot_pose @ self.arm2base @ self.ee2arm @ self.tcp2ee
        # translation = tcp_pose[:3, 3]
        # rotation_rpy = euler.quat2euler(tcp_pose[:3, :3])
        # return np.concatenate([translation, rotation_rpy]), 0.0


def main(args=None):
    rclpy.init(args=args)
    node = WBCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()