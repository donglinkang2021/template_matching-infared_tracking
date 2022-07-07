# -*- coding: UTF-8 -*-

import time
import cv2 as cv
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
from email.mime.text import MIMEText
from email.header import Header
from email.mime.multipart import MIMEMultipart 
from email.mime.image import MIMEImage
import dlib

# 找到地标中心函数
# 输入：原始图像
# 输出：中心点坐标
def Find_Center(img):
    thresh = []
    list_xc = []
    list_yc = []
    list_x = []
    list_y = []
    list_w = []
    list_h = []
    # 存储符合条件参数列表
    list_result_xc = []
    list_result_yc = []
    list_result_x = []
    list_result_y = []
    list_result_w = []
    list_result_h = []
    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    #contours, thresh = cv.threshold(img_gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    contours, thresh = cv.threshold(img_gray, 200, 255, cv.THRESH_BINARY)
    # 膨胀操作（部分线太细）
    kernel = np.ones((5, 5), np.int8)
    image1 = cv.erode(thresh, kernel, iterations=3)  # 先膨胀
    # 2.寻找轮廓
    contours, hierarchy = cv.findContours(thresh, cv.RETR_LIST, 2)
    # 3.绘制轮廓
    img_draw = img.copy()
    # cv.drawContours(img_draw, contours, -1, (0, 0, 255), 2)
    # 显示全部轮廓
    # cv.imshow('findContours', img_draw)
    # 4.矩形检测
    length = len(contours)
    img_approx = img.copy()
    img_bounding = img.copy()
    for i in range(length):
        cnt = contours[i]
        epsilon = 0.001 * cv.arcLength(cnt, True)
        approx = cv.approxPolyDP(cnt, epsilon, True)
        if len(approx) >10:
            #print(approx)
            cv.polylines(img_approx, [approx], True, (0, 255, 0), 2)
            (x, y, w, h) = cv.boundingRect(approx)  # 检测轮廓
            cv.rectangle(img_bounding, (x, y), (x + w, y + h), (255, 0, 0), 2)  # 绘制检测结果
            # 计算矩形中心
            x_c = int(x + w / 2)
            y_c = int(y + h / 2)
            # 将中心标注出来
            cv.circle(img_bounding, (x_c, y_c), 1, color=(0, 0, 255), thickness=3)
            # 将所有的矩形元素储存到列表中
            list_xc.append(x_c)
            list_yc.append(y_c)
            list_x.append(x)
            list_y.append(y)
            list_w.append(w)
            list_h.append(h)

    for i in range(len(list_xc)):
        if list_w[i] > 5 and list_h[i] > 5:
            if list_result_xc==[]:
                list_result_xc.append(list_xc[i])
                list_result_yc.append(list_yc[i])
                list_result_x.append(list_x[i])
                list_result_y.append(list_y[i])
                list_result_w.append(list_w[i])
                list_result_h.append(list_h[i])
            break
    # print(len(list_result_x))
    if len(list_result_w) != 0:
        scale=0.5/max(list_result_w)
    else:
        scale = 0

    img_rectangle = img.copy()
    # 绘制矩形框
    for i in range(len(list_result_x)):
        # 绘制检测结果
        cv.rectangle(img_rectangle, (list_result_x[i], list_result_y[i]),
                     (list_result_x[i] + list_result_w[i], list_result_y[i] + list_result_h[i]), (255, 0, 0), 2)
        # 将中心标注出来
        cv.circle(img_rectangle, (list_result_xc[i], list_result_yc[i]), 1, color=(0, 0, 255), thickness=3)
    # 函数返回中心值
    if len(list_result_x) != 0:
        return list_result_xc[0], list_result_yc[0], img_rectangle, scale, True, thresh
    else:
        return 0, 0, img_rectangle, scale, False, thresh


# 初始化云台参数
servo1_color_track = 1500
servo2_color_track = 1500

servo1_face_track = 1200
servo2_face_track = 1500

servo1_pid1 = pid.PID(P=0.35, I=0.6, D=0.065)#pid初始化 #上下
servo2_pid2 = pid.PID(P=0.33, I=0.6, D=0.066)#pid初始化 #左右

def setServoInit():
    global servo1_color_track, servo1_color_track

    servo1_color_track = servo2_color_track= 1500

dis_ok_color = False
action_finish_color = True


#执行动作
def track():
    global servo1_color_track, servo2_color_track
    global servo1_face_track, servo2_face_track
    global dis_ok_color, action_finish_color

    while True:
    #云台跟踪
        if dis_ok_color:
            dis_ok_color = False
            action_finish_color = False
            PWMServo.setServo(1, servo1_color_track, 20)
            PWMServo.setServo(2, servo2_color_track, 20)
            time.sleep(0.02)
            action_finish_color = True
        elif dis_ok_face:
            dis_ok_face = False
            PWMServo.setServo(1, servo1_face_track, 20)
            PWMServo.setServo(2, servo2_face_track, 20)
            time.sleep(0.02)
        else:
            time.sleep(0.01)

 #作为子线程开启
cv_color_track = threading.Thread(target=track)
cv_color_track.setDaemon(True)
cv_color_track.start()  

tracking_path = []
def cv_color_track(frame, rw, rh, target_color = 'green'):
    global servo1_color_track, servo2_color_track
    global dis_ok_color, action_finish_color, tracking_path

    frame_resize = cv.resize(frame, (rw, rh), interpolation = cv.INTER_NEAREST)
    img_center_x = frame_resize.shape[:2][1]/2#获取缩小图像的宽度值的一半
    img_center_y = frame_resize.shape[:2][0]/2
    #frame_GaussianBlur = cv.GaussianBlur(frame_resize, (3, 3), 0)
    #frame_lab = cv.cvtColor(frame_GaussianBlur, cv.COLOR_BGR2LAB) #将图像转换到LAB空间
    frame_lab = cv.cvtColor(frame_resize, cv.COLOR_BGR2LAB) #将图像转换到LAB空间

    frame_mask = cv.inRange(frame_lab, color_range[target_color][0], color_range[target_color][1]) #根据hsv值对图片进行二值化 
    #opened = cv.morphologyEx(frame_mask, cv.MORPH_OPEN, np.ones((3,3),np.uint8))#开运算
    #closed = cv.morphologyEx(opened, cv.MORPH_CLOSE, np.ones((3,3),np.uint8))#闭运算
    #contours = cv.findContours(closed, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2] #找出所有外轮廓
    contours = cv.findContours(frame_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2] #找出所有外轮廓
    areaMaxContour = getAreaMaxContour(contours)[0] #找到最大的轮廓
    
    centerX = 0
    centerY = 0
    radius = 0
    
    if areaMaxContour is not None:  #有找到最大面积
        (centerX, centerY), radius = cv.minEnclosingCircle(areaMaxContour) #获取最小外接圆
        if radius >= 8:                        
            ########pid处理#########
            #以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入#
            err = abs(img_center_y - centerY)
            if err < 10:
                servo1_pid1.SetPoint = centerY
            else:
                servo1_pid1.SetPoint = img_center_y
            servo1_pid1.update(centerY)
            tmp = int(servo1_color_track -  servo1_pid1.output)
            tmp = 500 if tmp < 500 else tmp
            servo1_color_track = 1950 if tmp > 1950 else tmp

            err = abs(img_center_x - centerX)
            if err < 15:
                servo2_pid2.SetPoint = 2*img_center_x - centerX
            else:
                servo2_pid2.SetPoint = img_center_x

            servo2_pid2.update(2*img_center_x - centerX)#当前
            tmp = int(servo2_color_track + servo2_pid2.output)
            tmp = 500 if tmp < 500 else tmp
            servo2_color_track = 2500 if tmp > 2500 else tmp

            centerX = int(leMap(centerX, 0, rw, 0, 640))
            centerY = int(leMap(centerY, 0, rh, 0, 480))
            radius = int(leMap(radius, 0, rw, 0, 640))        

            cv.circle(frame, (int(centerX), int(centerY)), int(radius), range_rgb[target_color], 2)
            dis_ok_color = True
    #cv.putText(frame, "Color: " + target_color, (10, frame.shape[0] - 10), cv.FONT_HERSHEY_SIMPLEX, 0.65, range_rgb[target_color], 2)
    return frame

def revolving_stage_control(xc, yc, w, h, pError_x, pError_y):
    # PID参数，P，I，D
    pid_x = [0.05, 0.05, 0]
    pid_y = [0.05, 0.05 ,0]
    # 设置误差
    error_x = xc - w / 2
    error_y = h / 2 - yc
    speed_x = pid_x[0] * error_y + pid_x[1] * (error_y - pError_y)
    speed_x = int(speed_x*100)/100
    speed_y = pid_y[0] * error_x + pid_y[1] * (error_x - pError_x)
    speed_y = int(speed_y*100)/100
    #speed_x = int(np.clip(speed_x, -5, 5))
    #speed_y = int(np.clip(speed_y, -5, 5))
    # print('Vx', speed_x)
    # print('Vy', speed_y)
    # print('error_x', error_x)
    # print('error_y', error_y)
    return error_x, error_y, speed_x, speed_y


def main():
    # cap = cv.VideoCapture(0)
    cap = cv.VideoCapture(0)
    cap.set(3, 1920)
    cap.set(4, 1080)
    cap.set(5, 30)
    ret, img = cap.read()
    h, w, _ = img.shape
    print("h=", h)
    print("w=", w)
    # 初始x,y误差
    pError_x = 0
    pError_y = 0

    # 计数常量
    count = 0

    # 每0.1S计算一次帧率
    t = 0.1
    counter = 0
    fps = 0
    start_time = time.time()

   # 记录视频初始设置
    fourcc = cv.VideoWriter_fourcc(*'XVID')
    time_stamp = datetime.datetime.now()
    time_stamp_str = time_stamp.strftime("%Y_%m_%d__%H_%M_%S")
    print(time_stamp_str)
    out = cv.VideoWriter('test_1080p'+time_stamp_str+'.avi', fourcc, 30.0, (1920, 1080*2))
    # out1 = cv.VideoWriter('test_thresh_1080p'+time_stamp_str+'.avi', fourcc, 30.0, (640, 480))

    # 起飞 H = 5m
    # arm_and_takeoff(1.5)


    # print('正在飞往目标')
    while True:
        ret, img = cap.read()
        # cv.imshow("output", img)

        # 找到地标中心
        xc, yc, result_img, scale, flag, thresh = Find_Center(img)
        # print(xc, yc)

        # 计算帧率
        counter += 1
        if (time.time() - start_time) > t:
            fps = counter / (time.time() - start_time)
            # 将帧率数据转换为字符串
            # fps = str(fps)
            # 将帧率先转换为整型，再转换为字符串
            fps = str(int(fps))
            counter = 0
            start_time = time.time()
            print("当前帧率为：", fps)
        # 显示帧率
        cv.putText(result_img, "FPS={0}".format(fps), (10, 10), 3, 0.5, (200, 0, 0), 1)

        # 显示是否发现目标
        # cv.putText(result_img, "Find Target?{0}".format(str(flag)), (10, 150), 3, 0.5, (200, 0, 0), 1)

        # 绘制瞄准线
        h, w, _ = result_img.shape
        cv.line(result_img, (int(w / 2)-40, int(h / 2)), (int(w / 2)+40, int(h / 2)), (0, 255, 0), 2)
        cv.line(result_img, (int(w / 2), int(h / 2)-40), (int(w / 2), int(h / 2)+40), (0, 255, 0), 2)
        cv.circle(result_img, (int(w / 2), int(h / 2)), radius=10, color=(0, 255, 0), thickness=2)

        # 显示飞机基本信息
        # Fly_info_show(result_img)

        # 当有效检测时进行绘制，无效检测/无目标时中心值为0
        if flag:
            # 绘制误差线
            cv.line(result_img, (xc, yc), (xc, int(h / 2)), (0, 200, 0), 2)
            cv.line(result_img, (xc, yc), (int(w / 2), yc), (0, 0, 200), 2)
            # 飞行控制函数
            pError_x, pError_y, speed_x, speed_y = revolving_stage_control(xc, yc, w, h, pError_x, pError_y)


        thresh = cv.resize(thresh, (1920,1080), interpolation=cv.INTER_CUBIC)
        result_img = cv.resize(result_img, (1920,1080), interpolation=cv.INTER_CUBIC)

        thresh_img = np.zeros((1080,1920,3))
        thresh_img[:,:,0] = thresh
        thresh_img[:,:,1] = thresh
        thresh_img[:,:,2] = thresh
        thresh_img = thresh_img.astype(np.uint8)

        # 将实时图像和二值化图像上下拼接以同时显示和储存
        output = np.vstack((result_img, thresh_img))

        out.write(output)

        x, y = output.shape[0:2]
        output_show = cv.resize(output, (int(y/4), int(x/4)))
        cv.imshow('result', output_show)

        # 判断是否在中心，并进行计数，计数到达一定值后降落
        if flag and abs(pError_x) < 1 and abs(pError_y) < 1:
            count = count+1
            print("发现目标", count)
            # if count < 4:
            #     dis = math.sqrt(pError_y * pError_y + pError_x * pError_x)  # 默认高度为1米
            #     dis = dis*scale
            #     filename = 'Group11_A_Figure' + str(count) + '_' + str(dis)+'px.jpg'#文件名不能有小数点，所以距离取整
            #     cv.imwrite(filename, result_img)
            #     print(filename)
            if count > 100:
                break

        if cv.waitKey(1) & 0xff == ord("q"):
            break
        
    count = 0

    cap.release()
    cv.destroyAllWindows()

if __name__ == '__main__':
    # Connect to the Vehicle硬件连接
    # vehicle = connect('/dev/ttyUSB0', wait_ready=True, baud=57600)

    main()