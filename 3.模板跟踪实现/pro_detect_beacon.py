import cv2.cv2 as cv
import numpy as np
import operator
'''
此次实验应用了两种方法来实现对信标的实时跟踪
第一种：
    方法：
    把process_video()中的两行template = change_template(flag, template, template0, img, loc)全部注释掉；
    即可实现模板只是先验模板(自己提前选取好的模板)
    优缺点：
    稳定性好，但运动过快时会丢失追踪。
第二种：
    方法：
    不要注释掉第一种所说的那两行；
    优缺点：
    信标更加丝滑，可以观测到显示出的模板就是上一帧的获取模板，实时更新，这也就实现连续跟踪；而且丢失跟踪时模板恢复成默认配置。
'''

# 不要把一次性太多功能塞进一个函数
def template_matching(img, template, threshold):  # 并没有确定读入的模板是否存在需要另外写
    img_rgb = img
    img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)
    h, w = template.shape
    res = cv.matchTemplate(img_gray, template, cv.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)  # 返回最大值和最小值的索引
    xc = 0
    yc = 0
    flag = 0  # 判断是否检测目标，以相关系数的来判定
    if max_val > threshold:
        flag = 1
        cv.rectangle(img_rgb, max_loc, (max_loc[0] + w, max_loc[1] + h), (0, 0, 255), 2)
        yc = int(max_loc[0] + w / 2)
        xc = int(max_loc[1] + h / 2)
        cv.circle(img_rgb, (yc,xc), 5, (0, 255, 0), 2)

    return xc, yc, img_rgb, flag, max_loc, max_val


def change_template(flag, template, template0, img, loc):
    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    h, w = template0.shape
    next_template = template0
    if flag == 1:
        next_template = img_gray[loc[1]:loc[1] + h, loc[0]:loc[0] + w]
    return next_template


def process_video():
    cap = cv.VideoCapture(0)
    template0 = cv.imread('true_beacon.png', 0)
    template = template0
    cnt = 1
    while True:
        isframe, img = cap.read()
        # 找到信标中心,返回结果图像，同时开启第二种模式：第二种模式往往更丝滑：因为可以和前一帧比较；
        if cnt:  # 下面这里只执行一次，由于是第一次识别，我们把阈值只设置成0.5
            xc, yc, result_img, flag, loc, val = template_matching(img, template0, 0.5)
            template = change_template(flag, template, template0, img, loc)
            cnt = 0
            print("第一次执行完毕")
        else:  # 这里由于是前一帧与后一帧的区别不大，所以我们把阈值调高到0.8
            xc, yc, result_img, flag, loc, val = template_matching(img, template, 0.8)
            template = change_template(flag, template, template0, img, loc)
        # 输出看一下最大的相关系数是多少
        print(val)

        cv.imshow('template', template)
        # 绘制瞄准线
        h, w, l = result_img.shape
        cv.line(result_img, (int(w / 2) - 40, int(h / 2)), (int(w / 2) + 40, int(h / 2)), (0, 255, 0), 1)
        cv.line(result_img, (int(w / 2), int(h / 2) - 40), (int(w / 2), int(h / 2) + 40), (0, 255, 0), 1)
        cv.circle(result_img, (int(w / 2), int(h / 2)), radius=15, color=(0, 255, 0), thickness=1)
        # 显示
        cv.imshow('result_img', result_img)
        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break
    cap.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    process_video()
