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

    def submitTube(self):
        rpy_ = np.array([-3 * math.pi / 4, 0, -3 * math.pi / 4])
        rot_vec_ = rpy2rot_vec(rpy_)
        self.control_c.moveL([0.2, -0.85 + 0.005, 0.55, rot_vec_[0], rot_vec_[1], rot_vec_[2]], speed=0.1)

    def initPose(self):
        rpy_ = np.array([math.pi, 0, -3 * math.pi / 4])
        rot_vec_ = rpy2rot_vec(rpy_)
        self.control_c.moveL([0, -0.5, 0.55, rot_vec_[0], rot_vec_[1], rot_vec_[2]], speed=0.1)


if __name__ == '__main__':
    self = URControl()
    from SerialControl import ComSwitch
    import time

    self.grab = ComSwitch()
    print(self.receive_r.getTargetTCPPose())
    rpy = np.array([math.pi, 0, -3 * math.pi / 4])
    rotVec = rpy2rot_vec(rpy)
    self.control_c.moveL([0, -0.8, 0.25, rotVec[0], rotVec[1], rotVec[2]],
                         speed=0.2)
    self.grab.close()
    time.sleep(1)
    self.control_c.moveL([0, -0.8, 0.45, rotVec[0], rotVec[1], rotVec[2]],
                         speed=0.2)
    rpy = np.array([math.pi, 0, -3 * math.pi / 4])
    self.grab.open()
    time.sleep(1)
    # rot_vec = rpy2rot_vec(rpy)
    # self.control_c.moveL([0, -0.5, 0.55, rot_vec[0], rot_vec[1], rot_vec[2]], speed=0.1)
# self.control_c.moveL(
# 	[move_point_fix[0], move_point_fix[1], move_point_fix[2], rot_vec[0], rot_vec[1], rot_vec[2]],)
