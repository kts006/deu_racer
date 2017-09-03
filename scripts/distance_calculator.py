#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import math
import logging


logger = logging.getLogger(__file__)
logging.basicConfig(level=logging.INFO)


class DistanceCalculator:
    def __init__(self, type):
        '''
        크기가 4.4cm (__W)인 표지를 30cm (__D)에 위치시키고
        웹캠으로 사진을 찍어 픽셀 수 (__P)가 95인 상황으로 초기화됨.
        필요시 수정 필요
        '''
        # (private) initial_data는 dict 객체
        # key: 표지판 이름, value: [__P, __D, __W]
        self.__initial_data = {'parking': [95, 30.0, 4.4]}
        self.__P = 0
        self.__D = 0
        self.__W = 0
        self.__initialize(type)

    def __initialize(self, type):
        self.__P = self.__initial_data[type][0]  # unit: pixels
        self.__D = self.__initial_data[type][1]  # unit: cm
        self.__W = self.__initial_data[type][2]  # unit: cm
        self.__F = self.__P * self.__D / self.__W

    def set_type(self, type):
        self.__initialize(type)

    def get_distance(self, p1, p2):
        x1, y1 = p1[0]
        x2, y2 = p2[0]
        pixels = int(math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2))
        # Divide by zero 예외 처리 위해 크기 보정
        if pixels < 50:
            pixels = 50
        logger.debug('pixels = %d' % pixels)
        return self.__F * self.__W / float(pixels)

