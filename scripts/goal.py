import math

class Goal:
    def __init__(self, x, y, yaw, x_tol, y_tol, yaw_tol, x_diff, y_diff, yaw_diff, mode=0):
        self.x = x
        self.y = y
        self.yaw = yaw

        self.x_tol = x_tol
        self.y_tol = y_tol
        self.yaw_tol = yaw_tol

        self.x_diff = x_diff
        self.y_diff = y_diff
        self.yaw_diff = yaw_diff

    def rotation(self):
        A = [[math.cos(self.yaw), math.sin(self.yaw)], [-math.sin(self.yaw), math.cos(self.yaw)]]
        return A