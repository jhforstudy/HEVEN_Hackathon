#!/usr/bin/env python

import math
import numpy as np

global PARKING_SPOT
global STOP_LINE 
PARKING_SPOT = 0
STOP_LINE = 1


class Goal:
    def __init__(self, mode=PARKING_SPOT, position=np.array(None, None), yaw=None, flag=0, number=1):
        
        self.mode = mode
        self.position = position
        self.yaw = yaw
        self.unit_vector = np.array([math.cos(yaw), math.sin(yaw)])
        self.flag = flag
        self.tolarance = [0,0,0]
        self.number = number

        self.init_tolarance()

    def init_tolarance(self):
        # tolerence is TBD -> according to the size of the map
        if self.mode == PARKING_SPOT:
            self.tolarance[0] = 1 #x_tol
            self.tolarance[1] = 1 #y_tol 
            self.tolarance[2] = math.radians(15) #yaw_tol

        elif self.mode == STOP_LINE:
            self.tolarance[0] = 0.6 #distnace
            self.tolarance[1] = None #empty
            self.tolarance[2] = None #empty

    def rotation(self):
        A = np.array([[math.cos(self.yaw), math.sin(self.yaw)], [-math.sin(self.yaw), math.cos(self.yaw)]])
        return A