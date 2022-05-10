# MIT License.
# Copyright (c) 2021 by BioicDL. All rights reserved.
# Created by LiuXb on 2021/10/19
# -*- coding:utf-8 -*-

"""
@Modified:
@Description: the view robot workspace and camera field of view are not overlapping
"""

import numpy as np
import cv2
import yaml
from tqdm import tqdm
import threading
import multiprocessing as mp

def correct(uv1, uv2, uv3, uv4, u, v, cfg_path):
    __cfg = yaml.load(open(cfg_path, 'r'), Loader=yaml.FullLoader)
    # [x, y]
    xy1 = __cfg['xy1']
    xy2 = __cfg['xy2']
    xy3 = __cfg['xy3']
    xy4 = __cfg['xy4']
    # perspective transformation
    pts1 = np.float32([uv1, uv2, uv3, uv4])
    pts2 = np.float32([xy1, xy2, xy3, xy4])
    image2baseMatrix = cv2.getPerspectiveTransform(pts1, pts2)
    pick_point = [u, v]
    grasp_point = np.array([[pick_point]], dtype=np.float32)
    gp_base = cv2.perspectiveTransform(grasp_point, image2baseMatrix)
    x = gp_base[0][0][0]
    y = gp_base[0][0][1]
    return x, y


def thread_(uv11):
    global sum_
    u = 610.5
    v = 344
    x_ = 117.84
    y_ = -782.05
    # [494, 423] [497, 430] [702, 270] [705, 275]

    # u = 709.5
    # v = 326.0
    # x_ = -4.594
    # y_ = -805.84

    for uv21 in tqdm(range(496 - minus, 496 + minus)):
        for uv31 in range(702 - minus, 702 + minus):
            for uv41 in range(704 - minus, 704 + minus):
                for uv12 in range(258 - minus, 258 + minus):
                    for uv22 in range(368 - minus, 368 + minus):
                        for uv32 in range(257 - minus, 257 + minus):
                            for uv42 in range(365 - minus, 365 + minus):
                                uv1 = [uv11, uv12]
                                uv2 = [uv21, uv22]
                                uv3 = [uv31, uv32]
                                uv4 = [uv41, uv42]
                                x, y = correct(uv1, uv2, uv3, uv4, u, v, path)
                                absSum = abs(x - x_) + abs(y - y_)
                                if absSum < sum_:
                                    print("#" * 10, absSum)
                                    print(x, y)
                                    sum_ = absSum
                                    print(uv1, uv2, uv3, uv4)


if __name__ == '__main__':
    path = './sepa_cali2D.yaml'
    minus = 5
    sum_ = 10000
    for uv11_ in range(496 - minus, 496 + minus):
        t = threading.Thread(target=thread_, args=(uv11_,))
        t.start()

# correct()
# path = './sepa_cali2D.yaml'
# hand_eye = Calibration2D(path)
# print(hand_eye.cvt(702, 424))
# print(hand_eye.image2baseMatrix)
# print(hand_eye.robot2img(-0.22718306, -0.96182245))
