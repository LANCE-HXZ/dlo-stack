# -*- coding: utf-8 -*-
import cv2
from datetime import datetime
import matplotlib.pyplot as plt


# 细化函数，输入需要细化的图片（经过二值化处理的图片）和映射矩阵array
# 这个函数将根据算法，运算出中心点的对应值
def VThin(image, array, END):
    h, w = image.shape
    iThin = image
    NEXT = 1
    for i in range(h):
        for j in range(w):
            if NEXT == 0:
                NEXT = 1
            else:
                M = image[i, j - 1] + image[i, j] + image[i, j + 1] if 0 < j < w - 1 else 1
                if iThin[i, j] == 0 and M != 0:
                    a = [0] * 9
                    for k in range(3):
                        for l in range(3):
                            # 如果3*3矩阵的点不在边界且这些值为零，也就是黑色的点
                            if -1 < (i - 1 + k) < h and -1 < (j - 1 + l) < w and iThin[i - 1 + k, j - 1 + l] == 255:
                                a[k * 3 + l] = 1
                    sum = a[0] * 1 + a[1] * 2 + a[2] * 4 + a[3] * 8 + a[5] * 16 + a[6] * 32 + a[7] * 64 + a[8] * 128
                    # 然后根据array表，对ithin的那一点进行赋值。
                    iThin[i, j] = array[sum] * 255
                    if array[sum] == 1:
                        NEXT = 0
                        END = 1
    return iThin, END


def HThin(image, array, END):
    h, w = image.shape
    iThin = image
    NEXT = 1
    for j in range(w):
        for i in range(h):
            if NEXT == 0:
                NEXT = 1
            else:
                M = image[i - 1, j] + image[i, j] + image[i + 1, j] if 0 < i < h - 1 else 1
                if iThin[i, j] == 0 and M != 0:
                    a = [0] * 9
                    for k in range(3):
                        for l in range(3):
                            # 如果3*3矩阵的点不在边界且这些值为零，也就是黑色的点
                            if -1 < (i - 1 + k) < h and -1 < (j - 1 + l) < w and iThin[i - 1 + k, j - 1 + l] == 255:
                                a[k * 3 + l] = 1
                    sum = a[0] * 1 + a[1] * 2 + a[2] * 4 + a[3] * 8 + a[5] * 16 + a[6] * 32 + a[7] * 64 + a[8] * 128
                    # 然后根据array表，对ithin的那一点进行赋值。
                    iThin[i, j] = array[sum] * 255
                    if array[sum] == 1:
                        NEXT = 0
                        END = 1
    return iThin, END


# 映射表
array = [0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1,
         1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
         0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1,
         1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
         1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1,
         1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
         0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1,
         1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0,
         1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0]

# 读取灰度图片，并显示
prev_time = datetime.now()
img = cv2.imread('test/test.bmp', 0)  # 直接读为灰度图像

# OpenCV定义的结构元素
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
# 膨胀图像
img = cv2.dilate(img, kernel)


# 获取自适应二值化的细化图，并显示
END = 1
V_END = 1
H_END = 1
while END:
    END = 0
    if V_END:
        img, END = VThin(img, array, END)
        V_END = END
    if H_END:
        img, END = HThin(img, array, END)
        H_END = END
cur_time = datetime.now()
print(cur_time - prev_time)

plt.subplot(1, 1, 1)
plt.imshow(img, 'gray')
plt.pause(3000)
# plt.waitforbuttonpress(0)
# cv2.imshow('img', img)
# cv2.waitKey(0)
#
# cv2.destroyAllWindows()