import math
import numpy as np

global PARKING_SPOT
global STOP_LINE 
PARKING_SPOT = 0
STOP_LINE = 1


class Goal:
    def __init__(self, mode=PARKING_SPOT, position=np.array(None, None), yaw=None, flag=0):
        
        self.mode = mode
        self.position = position
        self.yaw = yaw
        self.unit_vector = np.array([math.cos(yaw), math.sin(yaw)])
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