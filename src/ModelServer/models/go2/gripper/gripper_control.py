import numpy as np
from enum import IntEnum
from struct import unpack
from struct import pack
import can
import subprocess
import time


# TODO: 修改self.slave_id为夹爪电机的id，调整control_Pos_Vel中期望速度V_desired为合适的值

class Gripper():
    def __init__(self, channel="vcan0", bustype="socketcan", slave_id=0x031, master_id=0x021, position_mode="relative"):
        # check if vcan0 exists, if not create it
        self._ensure_vcan_exists(channel)
        
        # start the subprocess for wscan-arm
        script_path = "/home/unitree/projects/ModelServer/src/ModelServer/models/go2/gripper/ws_can/script/wscan-arm"
        cmd = [script_path, channel, "connect", "ws://192.168.123.231:7998"]
        self.wscan_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # wait 0.1s before initializing CAN bus
        time.sleep(0.1)
        
        # init the CAN channel with filter for master_id only
        can_filters = [{"can_id": master_id, "can_mask": 0x7FF, "extended": False}]
        self.bus = can.interface.Bus(channel=channel, bustype=bustype, can_filters=can_filters)
        self.slave_id = slave_id
        self.master_id = master_id
        self.Limit_Param = [12.5, 30, 10] # max: q dq tau
        self.gripper_state = {'q':0, 'dq':0, 'tau':0, 'err':0, 'id':0}
        self.open_rad = 4.2
        # enable the gripper motor
        self.enable_motor()
        self.recv_until_specialId()
        # self.open_state = self.gripper_state['recv_q']
        # self.close_state = self.gripper_state['recv_q'] - self.open_rad
        self.open_state = 1.8
        self.close_state = -3.2
        # position mode: "relative" (0-1) or "absolute" (radians)
        if position_mode not in ["relative", "absolute"]:
            raise ValueError("position_mode must be 'relative' or 'absolute'")
        self.position_mode = position_mode

    def _ensure_vcan_exists(self, channel):
        """
        check if the specified CAN channel exists, if not create it
        """
        try:
            # check if the CAN interface exists
            result = subprocess.run(['ip', 'link', 'show', channel], 
                                  capture_output=True, text=True, check=False)
            
            if result.returncode != 0:
                # interface doesn't exist, create it
                print(f"CAN interface {channel} not found, creating it...")
                subprocess.run(['sudo', 'ip', 'link', 'add', channel, 'type', 'vcan'], 
                             check=True)
                subprocess.run(['sudo', 'ip', 'link', 'set', channel, 'up'], 
                             check=True)
                print(f"CAN interface {channel} created and brought up successfully")
            else:
                # interface exists, check if it's up
                # if 'state UP' not in result.stdout:
                #     print(f"CAN interface {channel} exists but is down, bringing it up...")
                #     subprocess.run(['sudo', 'ip', 'link', 'set', channel, 'up'], 
                #                  check=True)
                #     print(f"CAN interface {channel} brought up successfully")
                # else:
                print(f"CAN interface {channel} is already up and running")
                    
        except subprocess.CalledProcessError as e:
            print(f"Error managing CAN interface {channel}: {e}")
            raise
        except Exception as e:
            print(f"Unexpected error checking CAN interface {channel}: {e}")
            raise

    def __del__(self):
        """
        cleanup method to terminate the wscan process when the object is destroyed
        """
        if hasattr(self, 'wscan_process') and self.wscan_process.poll() is None:
            self.wscan_process.terminate()
            try:
                self.wscan_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.wscan_process.kill()

    def control_Pos_Vel(self, P_desired: float, V_desired: float = 16.0):
        """
        control the motor in position and velocity control mode 
        :param P_desired: desired position (relative: 0-1, absolute: radians)
        :param V_desired: desired velocity
        :return: None
        """
        if self.position_mode == "relative":
            # relative mode: 0 is closed, 1 is open
            if P_desired < 0:
                P_desired = 0
                print("P_desired is less than 0, set to 0")
            if P_desired > 1:
                P_desired = 1
                print("P_desired is greater than 1, set to 1")
            # map 0-1 to close_state-open_state
            P_desired_rad = self.close_state + (self.open_state - self.close_state) * P_desired
        else:
            # absolute mode: direct radians input, no range limitation
            P_desired_rad = P_desired

        motorid = 0x100 + self.slave_id
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        P_desired_uint8s = self.float_to_uint8s(P_desired_rad)
        V_desired_uint8s = self.float_to_uint8s(V_desired)
        data_buf[0:4] = P_desired_uint8s
        data_buf[4:8] = V_desired_uint8s
        self.send(motorid, data_buf)
        self.recv_until_specialId()  # receive the data from CAN bus

    def receive_data_parsing(self, message):
        """
        process the CAN message received
        """
        CANID = message.arbitration_id
        data = message.data
        
        if len(data) >= 6:
            self.gripper_state['id'] = data[0] & 0x0f
            self.gripper_state['err'] = (data[0] >> 4) & 0x0f
            q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
            dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
            tau_uint = np.uint16(((data[4] & 0xf) << 8) | data[5])
            Q_MAX = self.Limit_Param[0]
            DQ_MAX = self.Limit_Param[1]
            TAU_MAX = self.Limit_Param[2]
            self.gripper_state['recv_q'] = self.uint_to_float(q_uint, -Q_MAX, Q_MAX, 16)
            self.gripper_state['recv_dq'] = self.uint_to_float(dq_uint, -DQ_MAX, DQ_MAX, 12)
            self.gripper_state['recv_tau'] = self.uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)

    def send(self, arbitration_id, data, extended_id=False):
        """
        send a CAN message
        """
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=extended_id
        )
        try:
            self.bus.send(msg)
            # print(f"[SEND] {msg}")
        except can.CanError as e:
            # print(f"Send failed: {e}") 
            pass

    def recv_until_specialId(self):
        """
        receive the CAN message from gripper (filtered by master_id)
        """
        while True:
            msg = self.bus.recv()  # blocking, only receives master_id messages due to filter
            if msg is None:
                continue
            # print(f"[RECV] {msg}")
            # No need to check arbitration_id since filter ensures only master_id messages
            self.receive_data_parsing(msg)
            break

    def uint_to_float(self, x: np.uint16, min: float, max: float, bits):
        span = max - min
        data_norm = float(x) / ((1 << bits) - 1)
        temp = data_norm * span + min
        return np.float32(temp)

    def float_to_uint8s(self, value):
        # Pack the float into 4 bytes
        packed = pack('f', value)
        # Unpack the bytes into four uint8 values
        return unpack('4B', packed)
    
    def enable_motor(self):
        """
        enable the gripper motor by sending enable command
        """
        motorid = 0x100 + self.slave_id
        data_buf = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], np.uint8)
        self.send(motorid, data_buf)
    
    def disable_motor(self):
        """
        disable the gripper motor by sending disable command
        """
        motorid = 0x100 + self.slave_id
        data_buf = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD], np.uint8)
        self.send(motorid, data_buf)
    
    def set_zero_position(self):
        """
        set the zero position of the gripper motor
        """
        motorid = 0x100 + self.slave_id
        data_buf = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE], np.uint8)
        self.send(motorid, data_buf)
    
    def get_state(self):
        """
        get the state of the gripper motor
        :return: position in relative (0-1) or absolute (radians) mode
        """
        current_pos_rad = self.gripper_state['recv_q']
        
        if self.position_mode == "relative":
            # convert from radians to relative position (0-1)
            # map from [close_state, open_state] to [0, 1]
            relative_pos = (current_pos_rad - self.close_state) / (self.open_state - self.close_state)
            return float(np.clip(relative_pos, 0, 1))
        else:
            # absolute mode: return radians directly
            return float(current_pos_rad)