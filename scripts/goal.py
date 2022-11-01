import math
import numpy as np

global PARKING_SPOT
global STOP_LINE 
PARKING_SPOT = 0
STOP_LINE = 1


class Goal:
    def __init__(self, mode=PARKING_SPOT, x=None, y=None, yaw=None, flag=0):
        
        self.mode = mode
        self.x = x
        self.y = y
        self.yaw = yaw
        self.flag = flag

        # self.x_tol = x_tol
        # self.y_tol = y_tol
        # self.yaw_tol = yaw_tol

    def tolarance(self):
        tolarance = []
        if self.mode == PARKING_SPOT:
            tolarance[0] = 1 #x_tol
            tolarance[1] = 1 #y_tol 
            tolarance[2] = math.radians(30) #yaw_tol

        elif self.mode == STOP_LINE:
            tolarance[0] = 1 #distnace
            tolarance[1] = None #empty
            tolarance[2] = None #empty

        return tolarance

    def rotation(self):
        A = np.array([[math.cos(self.yaw), math.sin(self.yaw)], [-math.sin(self.yaw), math.cos(self.yaw)]])
        return A