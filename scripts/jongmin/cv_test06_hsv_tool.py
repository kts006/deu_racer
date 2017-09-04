#-*- coding: utf-8 -*-
import cv2
import numpy as np
import logging
import os

__author__ = 'Prof.Jong Min Lee'

SEARCH_RED_LIGHT = 0
SEARCH_YELLOW_LIGHT = 1
SEARCH_GREEN_LIGHT = 2


def get_image_file(mode):
    if mode == SEARCH_RED_LIGHT:
        img = cv2.imread('figs/RGB_RED.png')
    elif mode == SEARCH_YELLOW_LIGHT:
        img = cv2.imread('figs/RGB_YELLOW.png')
    elif mode == SEARCH_GREEN_LIGHT:
        img = cv2.imread('figs/RGB_GREEN.png')
    else:
        img = cv2.imread('figs/RGB_all_off.png')
    return img


def nothing(x):
    pass


def create_trackbar_window():
    cv2.namedWindow('Trackbar', cv2.WINDOW_NORMAL)
    cv2.createTrackbar('Low H', 'Trackbar', 24, 179, nothing)
    cv2.createTrackbar('High H', 'Trackbar', 38, 179, nothing)
    cv2.createTrackbar('Low S', 'Trackbar', 0, 255, nothing)
    cv2.createTrackbar('High S', 'Trackbar', 150, 255, nothing)
    cv2.createTrackbar('Low V', 'Trackbar', 220, 255, nothing)
    cv2.createTrackbar('High V', 'Trackbar', 255, 255, nothing)

if __name__ == '__main__':
    logger = logging.getLogger(os.path.basename(__file__))
    logging.basicConfig(level=logging.INFO)

    light_on = True
    img = get_image_file(SEARCH_YELLOW_LIGHT)
    cv2.imshow('img', img)
    create_trackbar_window()
    while True:
        # rgb -> hsv 변환
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # trackbar 값 읽기
        low_h = cv2.getTrackbarPos('Low H', 'Trackbar')
        high_h = cv2.getTrackbarPos('High H', 'Trackbar')
        low_s = cv2.getTrackbarPos('Low S', 'Trackbar')
        high_s = cv2.getTrackbarPos('High S', 'Trackbar')
        low_v = cv2.getTrackbarPos('Low V', 'Trackbar')
        high_v = cv2.getTrackbarPos('High V', 'Trackbar')
        logger.debug('low_h = %s' % (low_h,))

        # masking
        filter_lower = np.array([low_h, low_s, low_v])
        filter_upper = np.array([high_h, high_s, high_v])
        mask = cv2.inRange(hsv, filter_lower, filter_upper)

        cv2.imshow('mask', mask)

        # 그리기
        # 'q' 입력시 종료
        if cv2.waitKey(100) & 0xFF == ord('q'): break

    cv2.destroyAllWindows()



