# MIT License.
# Copyright (c) 2021 by BioicDL. All rights reserved.
# Created by LiuXb & JieYu on 2021/11/28
# -*- coding:utf-8 -*-

"""
@Modified:
@Description:
"""
# from build import Bionic
import time


class RobotClient(object):
    def __init__(self):
        self.robot = Bionic.tcpCommunication()
        self.robot.connect("127.0.0.1", 32000)

    def MoveL(self, vel, acc, pose):
        self.robot.MoveL(vel, acc, pose[0], pose[1], pose[2], pose[3])
        return self.robot.rbtFeedbackReadReady()

    def getEncPos(self):
        self.robot.getEncPos()
        return float(self.robot.rbtFeedbackReadReady())

    def setServoOnOff(self):
        self.robot.ServoONOff()
        return self.robot.rbtFeedbackReadReady()

    def getRobotTcpPos(self):
        self.robot.getRobotTcpPos()
        received_data = self.robot.rbtFeedbackReadReady()
        print(received_data)
        param = received_data.split('\n')
        for i in range(len(param)):
            param[i] = float(param[i])
        print(param)
        return param

    def convey_vel(self):
        t = 1
        pulse1 = self.getEncPos()
        time.sleep(t)
        pulse2 = self.getEncPos()
        pulse_minus = abs(pulse2 - pulse1)
        distance = pulse_minus / 8000 * 200  # 2*pi*R = 200mm  8000pulse/r
        conveyVel = distance / t
        return conveyVel
