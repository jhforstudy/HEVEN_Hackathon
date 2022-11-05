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
        self.END_POINT_X_2 = -3.919
        self.END_POINT_Y_2 = -1.2804
        # Endpoint of Map 3
        self.END_POINT_X_3 = 26.21
        self.END_POINT_Y_3 = -62.39
        
        # Direction of traffic in MAP 2
        self.map_2_traffic_dir = "RIGHT"

        # Direction of first parking mission (1 or 2)
        self.map_3_parking_first_dir = 1
        # Direction of first parking mission (3 or 4)
        self.map_3_parking_second_dir = 3

        # Spawn lists (x, y, yaw - degree)
        self.MAP_1_SPAWN_POINT = [(0,0,0)]
        if self.map_2_traffic_dir == "LEFT":
            self.MAP_2_SPAWN_POINT = [(0,0,0),(-7.08,-15.835,180)]
        elif self.map_2_traffic_dir == "RIGHT":
            self.MAP_2_SPAWN_POINT = [(0,0,0),(-0.669,-15.835,0)]
        self.MAP_3_SPAWN_POINT = [(0,0,0)]
        if self.map_3_parking_first_dir == 1:
            self.MAP_3_SPAWN_POINT.append((1.947,-15.683,0))
        elif self.map_3_parking_first_dir == 2:
            self.MAP_3_SPAWN_POINT.append((13.837,-15.683,0))
        if self.map_3_parking_second_dir == 3:
            self.MAP_3_SPAWN_POINT.append((11.526,-53.659,180))
        elif self.map_3_parking_second_dir == 4:
            self.MAP_3_SPAWN_POINT.append((2.47,-53.659,0))

        # Goal Marker
        self.m = Marker()
        self.m.header.frame_id = "map"
        self.m.ns = "goal_marker"
        self.m.id = 1
        self.m.type = Marker.CYLINDER
        self.m.action = Marker.ADD
        self.m.color.r, self.m.color.g, self.m.color.b = 1, 1, 0
        self.m.color.a = 1
        self.m.scale.x = 0.5
        self.m.scale.y = 0.5
        self.m.scale.z = 0

        # Rate of each thread
        self.thread_rate = 10
