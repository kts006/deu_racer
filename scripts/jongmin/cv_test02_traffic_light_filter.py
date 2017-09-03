#-*- coding: utf-8 -*-
import cv2
import numpy as np
import logging
import os

__author__ = 'Prof.Jong Min Lee'

logger = logging.getLogger(os.path.basename(__file__))
logging.basicConfig(level=logging.INFO)

want_max_size = False
rgb_filtering = False
isFirst = True

rgb_fig_choice = 0      # 0 for R, 1 for Y,  2 for G
rgb_filter_choice = 0   # 0 for R, 1 for Y,  2 for G
light_on = True
colors = {0:'빨간색', 1:'노란색', 2:'녹색'}


if rgb_fig_choice == 0:
    img = cv2.imread('figs/RGB_RED.png')
    logger.warn('빨간색 신호등이 켜져 있는지 찾습니다.')
elif rgb_fig_choice == 1:
    img = cv2.imread('figs/RGB_YELLOW.png')
    logger.info('노란색 신호등이 켜져 있는지 찾습니다.')
elif rgb_fig_choice == 2:
    img = cv2.imread('figs/RGB_GREEN.png')
    logger.info('녹색 신호등이 켜져 있는지 찾습니다.')

if not light_on:
    img = cv2.imread('figs/RGB_all_off.png', 1)

# blurring: (15,15) --> 홀수로 해야 함
# 이미지를 흐리게 하면 노이즈가 감소되는 효과 있음
img = cv2.GaussianBlur(img, (15,15), 0)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# bilateral filtering
# frame = cv2.bilateralFilter(frame, 9, 75, 75)s

if isFirst:
    logger.info('shape = %s' % (img.shape, ))
    logger.debug('size = %d' % (img.size, ))
    isFirst = False

# filtering: old value
# red_lower = np.array([0, 0, 200])
# red_upper = np.array([10, 50, 255])
# yellow_lower = np.array([11, 0, 180])
# yellow_upper = np.array([34, 70, 255])
# green_lower = np.array([35, 0, 170])
# green_upper = np.array([80, 150, 255])

# filtering: new value
red_lower = np.array([0, 0, 237])
red_upper = np.array([10, 100, 255])
yellow_lower = np.array([24, 0, 220])
yellow_upper = np.array([38, 150, 255])
green_lower = np.array([38, 0, 200])
green_upper = np.array([71, 176, 255])

if rgb_filter_choice == 0:
    filter_lower = red_lower
    filter_upper = red_upper
elif rgb_filter_choice == 1:
    filter_lower = yellow_lower
    filter_upper = yellow_upper
else:
    filter_lower = green_lower
    filter_upper = green_upper


mask = cv2.inRange(hsv, filter_lower, filter_upper)
res = cv2.bitwise_and(img, img, mask=mask)

cv2.imshow('img', img)
# cv2.imshow('gray', gray)
cv2.imshow('hsv', hsv)
cv2.imshow('res', res)

# 필터링된 비트 수 찾기
indices = np.where(mask > 0)
indices_len = len(indices[0])
logger.info('탐지된 픽셀 수 = %d' % (indices_len,))
logger.debug('인덱스 수는 탐지된 신호등의 픽셀 수를 의미함.')

if indices_len > 300 and 0 <= rgb_fig_choice <=2:
    logger.info('%s 신호들이 탐지되었습니다!!!' % (colors[rgb_fig_choice],))

cv2.waitKey(-1)

cv2.destroyAllWindows()
