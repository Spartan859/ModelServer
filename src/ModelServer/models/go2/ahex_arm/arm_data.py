from dataclasses import dataclass
from cyclonedds.idl import IdlStruct
import cyclonedds.idl.types as types


# This class defines user data consisting of a float data and a string data
@dataclass
class JointState(IdlStruct, typename="JointState"):
    position: types.array[types.float64, 6]
    float_data: float