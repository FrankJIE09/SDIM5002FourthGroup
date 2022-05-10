# MIT License.
# Copyright (c) 2021 by BioicDL. All rights reserved.
# Created by LiuXb on 2021/10/19
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description:图像处理和机器运动顺序执行
"""
import math
import sys
import os
import yaml
import time
import numpy as np
import cv2
import sympy
import threading

# from yolov5.YOLO5 import Yolo5
import yolov5.detect as detect
# from Client import RobotClient
from separation_2d_Calibration import Calibration2D
import pyrealsense2 as rs
# from SerialControl import ComSwitch
# import warnings
#
# warnings.filterwarnings(action='ignore')


def cal_time(v, acc, x, y, a, b, vb, t_sum):
    # (x,y)object's real coordinate under camera;(a,b)robot's coordinate before moving
    # t5 = Symbol('t5')
    # mmm = solve(
    #     (v * t5 - v ** 2 / acc) ** 2 - (y - vb * (t5 + t_sum) * sympy.sin(theta) - b) ** 2 - (
    #             x - vb * (t5 + t_sum) * sympy.cos(theta) - a) ** 2, t5)
    mmm = [0, 0]
    mmm[0] = (
                     1.24981827555266e+31 * a * acc * vb + 2.13138005662797e+29 * acc * b * vb + 1.25e+31 * acc * t_sum * vb ** 2 - 1.24981827555266e+31 * acc * vb * x - 2.13138005662797e+29 * acc * vb * y + 1.25e+31 * v ** 3 - 1.76776695296637e+31 * sympy.sqrt(
                 0.5 * a ** 2 * acc ** 2 * v ** 2 - 0.000145368990265327 * a ** 2 * acc ** 2 * vb ** 2 + 0.0170485615803015 * a * acc ** 2 * b * vb ** 2 + 0.999854620442127 * a * acc ** 2 * t_sum * v ** 2 * vb - 1.0 * a * acc ** 2 * v ** 2 * x + 0.000290737980530654 * a * acc ** 2 * vb ** 2 * x - 0.0170485615803015 * a * acc ** 2 * vb ** 2 * y + 0.999854620442127 * a * acc * v ** 3 * vb + 0.5 * acc ** 2 * b ** 2 * v ** 2 - 0.499854631009735 * acc ** 2 * b ** 2 * vb ** 2 + 0.0170510404530238 * acc ** 2 * b * t_sum * v ** 2 * vb - 1.0 * acc ** 2 * b * v ** 2 * y - 0.0170485615803015 * acc ** 2 * b * vb ** 2 * x + 0.99970926201947 * acc ** 2 * b * vb ** 2 * y + 0.5 * acc ** 2 * t_sum ** 2 * v ** 2 * vb ** 2 - 0.999854620442127 * acc ** 2 * t_sum * v ** 2 * vb * x - 0.0170510404530238 * acc ** 2 * t_sum * v ** 2 * vb * y + 0.5 * acc ** 2 * v ** 2 * x ** 2 + 0.5 * acc ** 2 * v ** 2 * y ** 2 - 0.000145368990265327 * acc ** 2 * vb ** 2 * x ** 2 + 0.0170485615803015 * acc ** 2 * vb ** 2 * x * y - 0.499854631009735 * acc ** 2 * vb ** 2 * y ** 2 + 0.0170510404530238 * acc * b * v ** 3 * vb + acc * t_sum * v ** 3 * vb ** 2 - 0.999854620442127 * acc * v ** 3 * vb * x - 0.0170510404530238 * acc * v ** 3 * vb * y + 0.5 * v ** 4 * vb ** 2)) / (
                     acc * (1.25e+31 * v ** 2 - 1.25e+31 * vb ** 2))
    mmm[1] = (
                     1.24981827555266e+31 * a * acc * vb + 2.13138005662797e+29 * acc * b * vb + 1.25e+31 * acc * t_sum * vb ** 2 - 1.24981827555266e+31 * acc * vb * x - 2.13138005662797e+29 * acc * vb * y + 1.25e+31 * v ** 3 + 1.76776695296637e+31 * sympy.sqrt(
                 0.5 * a ** 2 * acc ** 2 * v ** 2 - 0.000145368990265327 * a ** 2 * acc ** 2 * vb ** 2 + 0.0170485615803015 * a * acc ** 2 * b * vb ** 2 + 0.999854620442127 * a * acc ** 2 * t_sum * v ** 2 * vb - 1.0 * a * acc ** 2 * v ** 2 * x + 0.000290737980530654 * a * acc ** 2 * vb ** 2 * x - 0.0170485615803015 * a * acc ** 2 * vb ** 2 * y + 0.999854620442127 * a * acc * v ** 3 * vb + 0.5 * acc ** 2 * b ** 2 * v ** 2 - 0.499854631009735 * acc ** 2 * b ** 2 * vb ** 2 + 0.0170510404530238 * acc ** 2 * b * t_sum * v ** 2 * vb - 1.0 * acc ** 2 * b * v ** 2 * y - 0.0170485615803015 * acc ** 2 * b * vb ** 2 * x + 0.99970926201947 * acc ** 2 * b * vb ** 2 * y + 0.5 * acc ** 2 * t_sum ** 2 * v ** 2 * vb ** 2 - 0.999854620442127 * acc ** 2 * t_sum * v ** 2 * vb * x - 0.0170510404530238 * acc ** 2 * t_sum * v ** 2 * vb * y + 0.5 * acc ** 2 * v ** 2 * x ** 2 + 0.5 * acc ** 2 * v ** 2 * y ** 2 - 0.000145368990265327 * acc ** 2 * vb ** 2 * x ** 2 + 0.0170485615803015 * acc ** 2 * vb ** 2 * x * y - 0.499854631009735 * acc ** 2 * vb ** 2 * y ** 2 + 0.0170510404530238 * acc * b * v ** 3 * vb + acc * t_sum * v ** 3 * vb ** 2 - 0.999854620442127 * acc * v ** 3 * vb * x - 0.0170510404530238 * acc * v ** 3 * vb * y + 0.5 * v ** 4 * vb ** 2)) / (
                     acc * (1.25e+31 * v ** 2 - 1.25e+31 * vb ** 2))

    # print("t1 = ", mmm[0])
    # print("t2 = ", mmm[1])
    for num in range(2):
        if mmm[num] < 0:
            mmm[num] = 100000000000

    t5 = mmm[0] if mmm[0] < mmm[1] else mmm[1]
    tb = t5 + t_sum
    x2 = x - vb * tb * math.cos(theta)
    y2 = y - vb * tb * math.sin(theta)
    # print("mmm = ", tb)
    # print("x = ", x2)
    # print("y = ", y2)

    return [x2, y2, t5, tb]


class MOVE(object):
    def __init__(self):
        self.COM = ComSwitch()
        # 目标定上方高度
        self.height_offset = -50
        # 抓取高度
        self.z = -110
        # belt direct
        self.robotFuncLabel = False
        self.robot = RobotClient()
        self.robot.setServoOnOff()
        self.belt_velocity = self.robot.convey_vel()
        # print("belt_velocity = ", self.belt_velocity)
        self.belt_velocity = 250

        self.vel = 200
        self.accRobot = 2000
        self.tc = 0  # camera deal time
        self.t3 = 0.25  # send  time
        self.t4 = 0.07  # calculate moveL t time
        self.t2 = 0.63  # 从目标点上方往下运动的时间(s)

    def robotMove(self, struct):
        self.robotFuncLabel = True
        """set robot"""
        # deal the struct
        # objectPool.append([temp, tCameraBefore, angle, target_pose])
        angle = struct[2]
        target_pose = struct[3]
        self.tc = time.time() - struct[1]
        t_sum = self.tc + self.t3 + self.t4 + self.t2
        robotPos = self.robot.getRobotTcpPos()
        print("robotPos = ",robotPos)
        calBefore = time.time()
        temp = cal_time(self.vel, self.accRobot, struct[0][0], struct[0][1], robotPos[0], robotPos[1],
                        self.belt_velocity, t_sum)
        calEndure = time.time() - calBefore
        self.t4 = calEndure
        print("calEndure = ", calEndure)
        # limit line for x
        xLimitUpper = 449
        xLimitLower = -79
        if temp[0] > xLimitUpper:
            tm = (temp[0] - xLimitUpper) / (math.cos(theta) * self.belt_velocity)
            # print("waitTime =", tm)
            temp[0] = xLimitUpper
            temp[1] = temp[1] - self.belt_velocity * tm * math.sin(theta)
            # print("temp = ", temp)
            tempPoseHover = [temp[0], temp[1], self.height_offset, angle, 0, 0]
            tempPose = [temp[0], temp[1], self.z, angle, 0, 0]
            moveBefore = time.time()
            self.robot.MoveL(self.vel, self.accRobot, tempPoseHover)
            self.openGripper()
            moveEndure = time.time() - moveBefore
            t_delay = temp[3] + tm - moveEndure -1.3
            print("moveEndure = ", moveEndure)
            if t_delay > 0:
                print("delay_time = ",t_delay)
                time.sleep(t_delay)
            descendBefore = time.time()
            self.robot.MoveL(self.vel, self.accRobot, tempPose)
            descendEndure = time.time() - descendBefore
            print("descendTime = ", descendEndure)
            self.closeGripper()
            self.robot.MoveL(self.vel, self.accRobot, tempPoseHover)

        elif temp[0] < xLimitLower:
            print('##########################################')
            print("xLimitLower,can not grab object")
            print('##########################################')
            self.robotFuncLabel = False
            return
        else:
            tempPoseHover = [temp[0], temp[1], self.height_offset, angle , 0, 0]
            tempPose = [temp[0], temp[1], self.z, angle , 0, 0]
            # print("temp = ", temp)
            self.robot.MoveL(self.vel, self.accRobot, tempPoseHover)
            self.openGripper()
            descendBefore = time.time()
            self.robot.MoveL(self.vel, self.accRobot, tempPose)
            descendEndure = time.time() - descendBefore
            print("descendTime = ", descendEndure)
            self.closeGripper()
            self.robot.MoveL(self.vel, self.accRobot, tempPoseHover)
        self.t2 = descendEndure
        # Move to the waste box
        self.robot.MoveL(self.vel, self.accRobot,
                         [target_pose[0], target_pose[1], self.height_offset, angle, 0, 0])
        self.openGripper()
        self.robotFuncLabel = False
        return

        # break

    def threadCheckLabel(self):
        while True:
            while self.robotFuncLabel is True or objectPool == []:
                time.sleep(1)
            self.robotFuncLabel = True
            print("Before move,objectPool = ", objectPool)
            self.robotMove(objectPool[0])
            objectPool.remove(objectPool[0])
            print("After move,objectPool = ", objectPool)

    def openGripper(self):
        self.COM.open()

    def closeGripper(self):
        self.COM.close()
        time.sleep(0.5)

    def getBeltVel(self):
        while True:
            self.belt_velocity = self.robot.convey_vel()


class CAMERA(object):
    def __init__(self):
        points = rs.points()
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        profile = self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def get_image(self):
        frames = self.pipeline.wait_for_frames()
        time_getImage = time.time()
        aligned_frames = self.align.process(frames)
        aligned_color_frame = aligned_frames.get_color_frame()
        color_image = np.asanyarray(aligned_color_frame.get_data())
        return color_image, time_getImage

    def identify(self):
        """set camera"""
        # set ROI
        # row_min, row_max, col_min, col_max
        crop_bounding = [292, 525, 491, 1048]  # (400 100) (1000 700)
        # calibration
        cali_file = os.path.dirname(os.path.abspath(__file__)) + '/sepa_cali2D.yaml'
        hand_eye = Calibration2D(cali_file)

        # set detector
        # object_detector = Yolo5('./yolov5/yolov5_cfg.yaml')
        break_flag = False
        flag = True
        grasp_cnt = 0
        attempt_cnt = 0
        # waste bin position
        bin_1 = [0, -200, -75, 0, 0, 0]
        bin_2 = [350, -200, -75, 0, 0, 0]
        bin_3 = [0, 450, -75, 0, 0, 0]
        bin_4 = [350, 450, -75, 0, 0, 0]
        print("Begin")
        while flag:
            attempt_cnt += 1
            """ get image"""
            color_img, tCameraBefore = self.get_image()
            pictureName = '1.jpg'
            cv2.imwrite('picture/' + pictureName, color_img)
            # frame = camera.get_frame()
            # color_img = frame.color_image[0]
            # depth_img = frame.depth_image[0]
            """ object detection"""
            # crop_img = color_img[crop_bounding[0]:crop_bounding[1], crop_bounding[2]:crop_bounding[3]]
            # cv2.imshow("test_color", crop_img)
            # cv2.waitKey()
            # exit()
            # region_class = object_detector.detect('picture/'+pictureName)
            region_class = detect.run(weights='best.pt',source='picture/' + pictureName)
            # if no object
            if region_class.__len__() == 0:
                continue
            print(region_class)
            # object on the belt
            uv_roi = []
            cla_roi = []
            cfi_roi = []
            for i in range(len(region_class)):
                uv_temp = np.array(region_class[i][0:4], np.int16)
                cla_temp = int(region_class[i][5])
                cfi_temp = region_class[i][4]
                # col_min, row_min, col_max, row_max
                uv_mid_c = (uv_temp[0] + uv_temp[2]) / 2.0
                uv_mid_r = (uv_temp[1] + uv_temp[3]) / 2.0
                if uv_mid_c < crop_bounding[2] - 20 or uv_mid_c > crop_bounding[3] + 10 or uv_mid_r < crop_bounding[
                    0] - 20 or uv_mid_r > crop_bounding[1] + 10:
                    continue
                else:
                    uv_roi.append(uv_temp)
                    cla_roi.append(cla_temp)
                    cfi_roi.append(cfi_temp)
            # object on the belt
            if len(uv_roi) == 0:
                continue
            # 选取第一个，后续可能可以实现多点抓取
            uv = uv_roi[0]
            cla = cla_roi[0]
            cfi = cfi_roi[0]
            """ get place pose"""
            if cla == 0:
                target_pose = bin_1
            elif cla == 1:
                target_pose = bin_2
            elif cla == 2:
                target_pose = bin_3
            elif cla == 3:
                target_pose = bin_4
            else:
                print('\033[1;35m Error Category \033[0m!')
                target_pose = bin_1
                # continue
            # transfer to robot coordinate
            # col
            ux = (uv[0] + uv[2]) / 2.0
            # row
            vy = (uv[1] + uv[3]) / 2.0
            temp = hand_eye.cvt(ux, vy)
            # 处理时间，得到抓取点和目标点
            # angle
            if abs(uv[2] - uv[0]) >= abs(uv[3] - uv[1]):
                angle = 90
            else:
                angle = 0
            # limit line for x
            objectPool.append([temp, tCameraBefore, angle, target_pose])
            # print("IN identify,objectPool = ", objectPool)


theta = math.radians(0.977)
objectPool = []

if __name__ == "__main__":
    camera = CAMERA()
    # move = MOVE()
    GetRealPoint = threading.Thread(target=camera.identify)
    # LoopGetBelVel = threading.Thread(target=move.getBeltVel, daemon=False)
    try:
        GetRealPoint.start()
        # LoopGetBelVel.start()
    except Exception as e:
        print(e)
