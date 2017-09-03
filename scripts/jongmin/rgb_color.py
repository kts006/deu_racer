#!/usr/bin/env python3
# cf. matplotlib.colors.cnames  - dict object


class RGBColor:
    RED = (0, 0, 255)
    GREEN = (0, 255, 0)
    BLUE = (255, 0, 0)
    AQUA = (255, 255, 0)
    BEIGE = (0xDC, 0xF5, 0xF5)
    CORAL = (0x50, 0x7F, 0xFF)
    CYAN = (0xFF, 0xFF, 0)
    YELLOW = (0, 0xFF, 0xFF)
    TOMATO = (0x47, 0x63, 0xFF)
    SIENNA = (0x2D, 0x52, 0xA0)
    SEASHELL = (0xEE, 0xF5, 0xFF)
    PURPLE = (0x80, 0, 0x80)
    ROYALBLUE = (0xE1, 0x69, 0x41)


if __name__ == '__main__':
    print('RED = ', RGBColor.RED)
    print('repr(RED) = ', repr(RGBColor.RED))