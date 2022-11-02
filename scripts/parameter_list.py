from visualization_msgs.msg import Marker, MarkerArray

class Param():
    def __init__(self):
        # Information of a car
        self.car_angular_velocity = 1
        self.car_acceleration = 1
        self.car_jerk = 0
        self.WHEELBASE = 0.425
        self.REAR_LIDAR = 0.325
        self.WIDTH = 0.145
        self.SIZE_OF_TROPHY = 0.5

        # Endpoint of Map 1
        self.END_POINT_X_1 = 13.34
        self.END_POINT_Y_1 = -9.31

        # Endpoint of Map 2
        self.END_POINT_X_2 = -3.96
        self.END_POINT_Y_2 = -4.97

        # Endpoint of Map 3
        self.END_POINT_X_3 = 13.34
        self.END_POINT_Y_3 = -9.31

        # Goal Marker
        self.m = Marker()
        self.m.header.frame_id = "map"
        self.m.ns = "goal_marker"
        self.m.id = 1
        self.m.type = Marker.POINTS
        self.m.action = Marker.ADD
        self.m.color.r, self.m.color.g, self.m.color.b = 1, 1, 0
        self.m.color.a = 1
        self.m.scale.x = 0.2
        self.m.scale.y = 0.2
        self.m.scale.z = 0.0

        # Rate of each thread
        self.thread_rate = 10