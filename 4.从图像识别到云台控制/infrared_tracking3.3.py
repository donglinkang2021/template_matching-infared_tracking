#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/5/1 11:07
# @Author : Linkdom


import time
import cv2
import numpy as np
import math
import datetime
import threading
import smtplib
import re
import dlib
import pid
import PWMServo
from config import *

######################
servo1_color_track = 1500
servo2_color_track = 1500
servo1_pid1 = pid.PID(P=0.35, I=0.6, D=0.065)  # pid初始化 #上下
servo2_pid2 = pid.PID(P=0.33, I=0.6, D=0.066)  # pid初始化 #左右


def setServoInit():
    global servo1_color_track, servo1_color_track
    servo1_color_track = servo2_color_track = 1500


dis_ok_color = False
action_finish_color = True


# 执行动作
def track():
    global servo1_color_track, servo2_color_track
    global dis_ok_color, action_finish_color

    while True:
        # 云台跟踪
        if dis_ok_color:
            dis_ok_color = False
            action_finish_color = False
            PWMServo.setServo(1, servo1_color_track, 20)
            PWMServo.setServo(2, servo2_color_track, 20)
            time.sleep(0.02)
            action_finish_color = True
        else:
            time.sleep(0.01)


# 作为子线程开启
cv_template_track = threading.Thread(target=track)
cv_template_track.setDaemon(True)
cv_template_track.start()

tracking_path = []


# 不要把一次性太多功能塞进一个函数
def template_matching(img, template, threshold):  # 并没有确定读入的模板是否存在需要另外写
    img_rgb = img
    img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
    h, w = template.shape
    res = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)  # 返回最大值和最小值的索引
    xc = 0
    yc = 0
    flag = 0  # 判断是否检测目标，以相关系数的来判定
    if max_val > threshold:
        flag = 1
        cv2.rectangle(img_rgb, max_loc, (max_loc[0] + w, max_loc[1] + h), (0, 0, 255), 2)
        yc = int(max_loc[0] + w / 2)
        xc = int(max_loc[1] + h / 2)
        cv2.circle(img_rgb, (yc, xc), 5, (0, 255, 0), 2)

    return xc, yc, img_rgb, flag, max_loc, max_val


def change_template(flag, template, template0, img, loc):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w, = template0.shape
    next_template = template0
    if flag == 1:
        next_template = img_gray[loc[1]:loc[1] + h, loc[0]:loc[0] + w]
    return next_template


def cv_template_track(frame, template, template0):
    global servo1_color_track, servo2_color_track
    global dis_ok_color, action_finish_color, tracking_path
    tmp_h, tmp_w,_ = frame.shape
    frame_resize = frame
    img_center_x = tmp_w/2
    img_center_y = tmp_h/2
    img_center_x=int(img_center_x)
    img_center_y=int(img_center_y)
    img = frame_resize
    # 找到地标中心
    # 找到信标中心,返回结果图像，同时开启第二种模式：第二种模式往往更丝滑：因为可以和前一帧比较；
    xc, yc, result_img, flag, loc, val = template_matching(img, template0, 0.5)
    template = change_template(flag, template, template0, img, loc)

    cv2.imshow('template', template)
    cv2.line(result_img, (img_center_x - 40, img_center_y), (img_center_x + 40, img_center_y), (0, 255, 0), 2)
    cv2.line(result_img, (img_center_x, img_center_y - 40), (img_center_x, img_center_y + 40), (0, 255, 0), 2)
    cv2.circle(result_img, (img_center_x, img_center_y), radius=10, color=(0, 255, 0), thickness=2)

    if flag:  
        (centerX, centerY) = (xc, yc)
        # 绘制误差线
        cv2.line(result_img, (centerX, centerY), (centerX, img_center_y), (0, 200, 0), 2)
        cv2.line(result_img, (centerX, centerY), (img_center_x, centerY), (0, 0, 200), 2)
        ########pid处理#########
        # 以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入#
        # 调节上下移动舵机
        err = abs(img_center_y - centerY)
        if err < 10:
            servo1_pid1.SetPoint = centerY
        else:
            servo1_pid1.SetPoint = img_center_y
        servo1_pid1.update(centerY)
        tmp = int(servo1_color_track - servo1_pid1.output)
        tmp = 500 if tmp < 500 else tmp
        servo1_color_track = 1950 if tmp > 1950 else tmp
        # 调节左右横移舵机
        err = abs(img_center_x - centerX)
        if err < 10:
            servo2_pid2.SetPoint = centerX  # 设置为对称点让它反复横移逐渐稳定
        else:
            servo2_pid2.SetPoint = img_center_x
        servo2_pid2.update(centerX)  # 当前
        tmp = int(servo2_color_track + servo2_pid2.output)
        tmp = 500 if tmp < 500 else tmp
        servo2_color_track = 2500 if tmp > 2500 else tmp

        #  重新恢复设置画画
        dis_ok_color = True
    return result_img


def main():
    cap = cv2.VideoCapture(0)

    template0 = cv2.imread('true_beacon.png', 0)
    template = template0

    while True:
        ret, frame = cap.read()
        frame2=cv_template_track(frame,  template, template0)
        cv2.imshow('image',frame2)
        if cv2.waitKey(1) & 0xff == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
