import cv2.cv2 as cv
import numpy as np
import operator

def template_matching(imagepath):
    img_rgb = cv.imread(imagepath)
    img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)
    template = cv.imread('true_beacon.png', 0)
    h, w = template.shape
    res = cv.matchTemplate(img_gray, template, cv.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)  #  返回最大值和最小值的索引
    print(max_val)
    cv.rectangle(img_rgb, max_loc, (max_loc[0]+w, max_loc[1]+h), (0, 0, 255), 2)  #  画矩形的长宽顺序与我们默认的顺序不一样

    yc = int(max_loc[0] + w / 2)
    xc = int(max_loc[1] + h / 2)
    cv.circle(img_rgb, (yc, xc), 5, (0, 255, 0), 2)
    cv.namedWindow('res.png', cv.WINDOW_NORMAL)
    cv.imshow('res.png', img_rgb)
    cv.waitKey(0)
    cv.destroyAllWindows()


if __name__ == '__main__':
    template_matching("1000.jpg")