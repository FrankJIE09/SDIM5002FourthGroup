import yaml
import rtde_control
import rtde_io
import rtde_receive
from scipy.spatial.transform import Rotation as Rt
import numpy as np
import math

def from_yaml_get_data(label):
    file = open('../configs/UR10e.yaml', 'r', encoding='utf-8')
    read = file.read()
    cfg = yaml.load(read, Loader=yaml.FullLoader)

    return cfg[label]
    pass


# 姿态数据:由UR使用的rot_vec转换为rpy
def rot_vec2rpy(pose):
    r = Rt.from_rotvec(pose[3:6])
    rpy = r.as_euler('xyz', degrees=False)
    return rpy


# 姿态数据:由rpy转换为UR使用的rot_vec
def rpy2rot_vec(rpy):
    r = Rt.from_euler('xyz', rpy)
    rot_vec = r.as_rotvec()
    return rot_vec


class URControl:
    def __init__(self):
        IP = '192.168.1.10'
        self.control_c = rtde_control.RTDEControlInterface(IP)
        self.receive_r = rtde_receive.RTDEReceiveInterface(IP)
        self.io_control = rtde_io.RTDEIOInterface(IP)
        return

    def get_robot(self):
        return self


if __name__ == '__main__':
    self = URControl()
    print(self.receive_r.getTargetTCPPose())
    rpy = np.array([math.pi, 0, 0])
    rot_vec = rpy2rot_vec(rpy)
    # self.control_c.moveL([0.008878933, -0.8099861, 0.381, rot_vec[0], rot_vec[1], rot_vec[2]])
# self.control_c.moveL(
# 	[move_point_fix[0], move_point_fix[1], move_point_fix[2], rot_vec[0], rot_vec[1], rot_vec[2]],)
