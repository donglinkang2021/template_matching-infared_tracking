#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/4/30 10:31
# @Author : Linkdom
import numpy as np
import cv2.cv2 as cv

cap = cv.VideoCapture('stay.flv')
# # 视频的第一帧
# ret, frame = cap.read()
# # 设置窗口的初始位置
# x, y, = 900, 300  # simply hardcoded the values
# # 设置初始ROI来追踪
# roi = cv.imread('miss.jpg')
# h, w ,_ = roi.shape
# track_window = (x, y, w, h)

# 获取第一帧位置，并指定目标位置
ret, frame = cap.read()
x, y, h, w = 743, 100, 393, 397
track_window = (x, y, h, w)
# 指定感兴趣区域
roi = frame[x:x + h, y:y + w]

hsv_roi = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
roi_hist = cv.calcHist([hsv_roi], [0], None, [180], [0, 180])
cv.normalize(roi_hist, roi_hist, 0, 255, cv.NORM_MINMAX)

# 设置终止条件，可以是10次迭代，也可以至少移动1 pt
term_crit = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 1)
while (1):
    ret, frame = cap.read()
    if ret == True:
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        dst = cv.calcBackProject([hsv], [0], roi_hist, [0, 180], 1)
        # 应用meanshift来获取新位置
        ret, track_window = cv.meanShift(dst, track_window, term_crit)
        # 在图像上绘制
        x, y, w, h = track_window
        img2 = cv.rectangle(frame, (x, y), (x + w, y + h), 255, 2)
        cv.namedWindow('res.png', cv.WINDOW_NORMAL)
        cv.imshow('res.png', img2)
        k = cv.waitKey(30) & 0xff
        if k == 27:
            break
    else:
        break
