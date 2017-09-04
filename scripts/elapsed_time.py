import cv2


class ElapsedTime:
    def __init__(self):
        self.start_t = -1
        self.end_t = -1

    def start(self):
        self.start_t = cv2.getTickCount()

    def end(self):
        self.end_t = cv2.getTickCount()

    def getElapsedTime(self):
        if self.start_t != -1 and self.end_t != -1:
            return (self.end_t - self.start_t) / cv2.getTickFrequency()
        else:
            return -1

    def getTime(self):
        return self.getElapsedTime()

