from visualization_msgs.msg import Marker, MarkerArray

# (Map 2) You can change the direction of traffic
TRAFFIC_DIR = "LEFT"       # "LEFT", "RIGHT"
# (Map 3) You can change the position of parking lot
PARK_NUM_1 = 2      # 1, 2, 3, 4
PARK_NUM_2 = 6      # 5, 6, 7, 8

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
        
        self.STOP_LINE_TIME = 5
        # Direction of traffic in MAP 2 ("LEFT" or "RIGHT")
        self.map_2_traffic_dir = TRAFFIC_DIR

        # Center of stop line
        self.MAP_2_STOP_LINE_X = -4
        self.MAP_2_STOP_LINE_Y = -16.45

        # Check point of left or right road
        self.MAP_2_CHECK_LEFT_X = -6.02
        self.MAP_2_CHECK_LEFT_Y = -15.87
        self.MAP_2_CHECK_RIGHT_X = -1.89
        self.MAP_2_CHECK_RIGHT_Y = -15.87

        # Direction of first parking mission in MAP 3 (1 or 2 or 3 or 4)
        self.map_3_parking_first_dir = PARK_NUM_1
        # Direction of second parking mission in MAP 3 (5 or 6 or 7 or 8)
        self.map_3_parking_second_dir = PARK_NUM_2

        # Size of parking lot
        self.PARKING_LOT_WIDTH = 0.55
        self.PARKING_LOT_HEIGHT = 0.8

        # Tilt degree of parking lot
        self.PARKING_LOT_TILT_DEGREE = 45

        # Center point of parking lot (First parking mission)
        self.PARKING_LOT_X_1 = 0.497
        self.PARKING_LOT_Y_1 = -20.683
        self.PARKING_LOT_YAW_1 = -90

        self.PARKING_LOT_X_2 = 1.647
        self.PARKING_LOT_Y_2 = -20.683
        self.PARKING_LOT_YAW_2 = -90

        self.PARKING_LOT_X_3 = 12.026
        self.PARKING_LOT_Y_3 = -20.683
        self.PARKING_LOT_YAW_3 = -90

        self.PARKING_LOT_X_4 = 13.226
        self.PARKING_LOT_Y_4 = -20.683
        self.PARKING_LOT_YAW_4 = -90

        # Center point of parking lot (Second parking mission)
        self.PARKING_LOT_X_5 = 13.226
        self.PARKING_LOT_Y_5 = -48.55
        self.PARKING_LOT_YAW_5 = 90 + self.PARKING_LOT_TILT_DEGREE

        self.PARKING_LOT_X_6 = 12.026
        self.PARKING_LOT_Y_6 = -48.55
        self.PARKING_LOT_YAW_6 = 90 + self.PARKING_LOT_TILT_DEGREE

        self.PARKING_LOT_X_7 = 1.647
        self.PARKING_LOT_Y_7 = -48.55
        self.PARKING_LOT_YAW_7 = 90 + self.PARKING_LOT_TILT_DEGREE

        self.PARKING_LOT_X_8 = 0.497
        self.PARKING_LOT_Y_8 = -48.55
        self.PARKING_LOT_YAW_8 = 90 + self.PARKING_LOT_TILT_DEGREE

        self.MAP_3_STOP_LINE_X_1 = 8.278
        self.MAP_3_STOP_LINE_Y_1 = -12.154
        self.MAP_3_STOP_LINE_YAW_1 = -90
        self.MAP_3_STOP_LINE_X_2 = 26.236
        self.MAP_3_STOP_LINE_Y_2 = -51.142
        self.MAP_3_STOP_LINE_YAW_2 = -90

        if self.map_3_parking_first_dir == 1:
            self.parking_x_1 = self.PARKING_LOT_X_1
            self.parking_y_1 = self.PARKING_LOT_Y_1
            self.parking_yaw_1 = self.PARKING_LOT_YAW_1
        elif self.map_3_parking_first_dir == 2:
            self.parking_x_1 = self.PARKING_LOT_X_2
            self.parking_y_1 = self.PARKING_LOT_Y_2
            self.parking_yaw_1 = self.PARKING_LOT_YAW_2
        elif self.map_3_parking_first_dir == 3:
            self.parking_x_1 = self.PARKING_LOT_X_3
            self.parking_y_1 = self.PARKING_LOT_Y_3
            self.parking_yaw_1 = self.PARKING_LOT_YAW_3
        elif self.map_3_parking_first_dir == 4:
            self.parking_x_1 = self.PARKING_LOT_X_4
            self.parking_y_1 = self.PARKING_LOT_Y_4
            self.parking_yaw_1 = self.PARKING_LOT_YAW_4

        if self.map_3_parking_second_dir == 5:
            self.parking_x_2 = self.PARKING_LOT_X_5
            self.parking_y_2 = self.PARKING_LOT_Y_5
            self.parking_yaw_2 = self.PARKING_LOT_YAW_5
        elif self.map_3_parking_second_dir == 6:
            self.parking_x_2 = self.PARKING_LOT_X_6
            self.parking_y_2 = self.PARKING_LOT_Y_6
            self.parking_yaw_2 = self.PARKING_LOT_YAW_6
        elif self.map_3_parking_second_dir == 7:
            self.parking_x_2 = self.PARKING_LOT_X_7
            self.parking_y_2 = self.PARKING_LOT_Y_7
            self.parking_yaw_2 = self.PARKING_LOT_YAW_7
        elif self.map_3_parking_second_dir == 8:
            self.parking_x_2 = self.PARKING_LOT_X_8
            self.parking_y_2 = self.PARKING_LOT_Y_8
            self.parking_yaw_2 = self.PARKING_LOT_YAW_8

        # Spawn lists (x, y, yaw - degree)
        self.MAP_1_SPAWN_POINT = [(0,0,0)]
        if self.map_2_traffic_dir == "LEFT":
            self.MAP_2_SPAWN_POINT = [(0,0,0),(-4,-18.45,90),(-7.08,-15.835,180)]
        elif self.map_2_traffic_dir == "RIGHT":
            self.MAP_2_SPAWN_POINT = [(0,0,0),(-4,-18.45,90),(-0.669,-15.835,0)]
        self.MAP_3_SPAWN_POINT = [(0,0,0)]
        self.MAP_3_SPAWN_POINT.append(((8.288, -13.565, -90)))
        # Spawn points (First parking mission)
        if self.map_3_parking_first_dir == 1 or self.map_3_parking_first_dir == 2:
            self.MAP_3_SPAWN_POINT.append((1.947,-15.683,0))
        elif self.map_3_parking_first_dir == 3 or self.map_3_parking_first_dir == 4:
            self.MAP_3_SPAWN_POINT.append((13.837,-15.683,0))
        self.MAP_3_SPAWN_POINT.append(((25.280, -53.591, 180)))
        # Spawn points (Second parking mission)
        if self.map_3_parking_second_dir == 5 or self.map_3_parking_second_dir == 6:
            self.MAP_3_SPAWN_POINT.append((11.526,-53.659,180))
        elif self.map_3_parking_second_dir == 7 or self.map_3_parking_second_dir == 8:
            self.MAP_3_SPAWN_POINT.append((2.47,-53.659,0))

        # Goal Marker
        self.m = Marker()
        self.m.header.frame_id = "map"
        self.m.ns = "goal_marker"
        self.m.type = Marker.CYLINDER
        self.m.action = Marker.ADD
        self.m.color.r, self.m.color.g, self.m.color.b = 1, 1, 0
        self.m.color.a = 1
        self.m.scale.x = 0.5
        self.m.scale.y = 0.5
        self.m.scale.z = 0

        # Rate of each thread
        self.thread_rate = 10
