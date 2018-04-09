# turn 360, detect the angle seeker and obstacles
# find the nearest obstacle
# find the blind spot of the robot of that obstacle (at left or right side) (-1 or 1 direction)
# turn to face the nearest obstacle using the recorded angle_nearest_obstalce
# tune2face to the center of the obstacle
# move forward to the obstacle with center of obstacle as refer point (update refer point after each move)
# turn "-direction" until seeker is at center of the screen
# reverse away (50cm) from the seeker with seeker as the refer point
# move "direction" until obstacle blocks seeker, seeker as refer point
# hidding done

import math
import time

import detection
from rovio import rovio


class HidingAlgorithm(object):
    def __init__(self, host):
        self.rovio_detection = detection.RovioDetection()
        self.obstacle_detection = detection.ObstacleDetection()
        self.screen_width = 640
        self.screen_height = 480
        self.center_bound = 10
        self.center = [self.screen_width / 2, self.screen_height / 2]
        # self.rovio_api = rovio_api.RovioApi(host)
        # self.rovio_api_v2 = rovio_api.RovioApiV2(host)
        self.rovio = rovio.Rovio(host)

        self.obstacle_points = None
        self.rovio_point = None
        self.frame = None

        url = 'http://%s/GetData.cgi?8935' % host
        # url = "http://%s:%s/Jpeg/CamImg0000.jpg" % (host, 80)

    def distance(self, x1, y1, x2, y2):
        return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))

    def get_frame(self):
        print ('get frame?')
        ret, self.frame = self.rovio.camera.get_frame()
        print ('rovio point')
        self.rovio_point = self.rovio_detection.get_refer_point(self.frame)
        print ('obstacle')
        self.obstacle_points = self.obstacle_detection.get_refer_point(self.frame)

        # cv2.imshow('show', self.frame)
        # cv2.waitKey(1)

        print (self.rovio_point)
        print (self.obstacle_points)

    def detection_surrounding(self):
        angle_obstacles = []
        distance_obstacles = []

        angle_rovio = -1
        max_seeker = None
        turn_angle = 20
        for x in range(1, 16):
            self.get_frame()
            print ('turning x ', x)

            points = self.obstacle_points

            if points != 'No Obstacle Found':
                list_ob_points = [l['Refer_point'] for l in points]
                list_ob_areas = [l['Area'] for l in points]

                if len(list_ob_points) != 0:
                    angle_obstacles.append(x * turn_angle)
                    distance_obstacles.append(1 / max(list_ob_areas))

            seeker_point = self.rovio_point  # EXPERIMENT HOW MANY ROVIO ANGLE TO REGISTER

            if seeker_point != 'No Rovio Found':
                if max_seeker is None or seeker_point['Area'] > max_seeker['Area']:
                    max_seeker = seeker_point

                angle_rovio = x * turn_angle

            self.rovio.rotate_left(speed=0.5)  # turn 20 degree right
            # time.sleep(0.5)

        return distance_obstacles, angle_obstacles, angle_rovio

    def find_blind_spot(self, distance_obstacles, angle_obstacles, angle_rovio):
        blind_spot = 0
        angle_nearest_obstacle = -1

        if len(distance_obstacles) != 0:
            angle_nearest_obstacle = angle_obstacles[distance_obstacles.index(min(distance_obstacles))]
        else:
            print ("No Obstacle Found...")
            return 0, 0

        if angle_rovio != -1:
            diff_angle = abs(angle_nearest_obstacle - angle_rovio)
            if diff_angle > 20 or diff_angle < 160:
                blind_spot = 1
            elif diff_angle > 200 or diff_angle < 340:
                blind_spot = -1
            elif diff_angle <= 20 or diff_angle >= 340:
                blind_spot = 2
            else:
                blind_spot = -2
        else:
            return 3, angle_nearest_obstacle

        return blind_spot, angle_nearest_obstacle

    def check_center(self, bounding_box_x1, bounding_box_x2):
        return self.screen_width - bounding_box_x2 - self.center_bound < bounding_box_x1 < self.screen_width - bounding_box_x2 + self.center_bound

    def look_check_center(self, bx1, bx2, obj):
        i = 100
        while (not self.check_center(bx1, bx2)) and i >= 0:
            print (i, 'running check center')
            self.get_frame()
            bx1, bx2 = self.get_bxs(obj)
            i -= 1

        print ('stoppppppppppp')
        self.rovio.stop()

    def get_closest_ob(self, list1):
        print ('here?')
        ob_Areas = [x['Area'] for x in list1]
        print ('here????')
        i = ob_Areas.index(max(ob_Areas))
        print ('yes hereee', i)
        return i

    def get_bxs(self, obj):
        print ('why stop here?')
        bx1 = -1
        bx2 = -1
        if obj == 'obstacle':
            list1 = self.obstacle_points
            if list1 != 'No Obstacle Found':
                ind_closest = self.get_closest_ob(list1)
                bx1 = list1[ind_closest]['Bottom_corner1'][0]
                bx2 = list1[ind_closest]['Bottom_corner2'][0]
        else:
            list1 = self.rovio_point
            if list1 != 'No Rovio Found':
                bx1 = list1['Bottom_corner1'][0]
                bx2 = list1['Bottom_corner2'][0]

        return bx1, bx2

    def turn2face(self, obj, turn_direction):  # turn to face seeker
        print ('start turn 2 face')
        # turn direction depends on blind spot
        bx1, bx2 = self.get_bxs(obj)
        # EXPERIMENT ASSUME CAN BE STOPPED WHILE TURNING
        self.rovio.rotate_right(0.3, 360)
        self.look_check_center(bx1, bx2, obj)
        print ('doneeee')

    def get_refer_point(self, obj):
        if obj == 'obstacle':
            list1 = self.obstacle_points
            ind_closest = self.get_closest_ob(list1)
            return list1[ind_closest]['Refer_point']
        else:
            list1 = self.rovio_point
            return list1['Refer_point']

    def move_forward(self, nearest_distance, obj):
        self.get_frame()
        initial_refer_point = self.get_refer_point(obj)
        forward_dis = self.forward_dis(initial_refer_point)
        refer_point = initial_refer_point

        while forward_dis > nearest_distance:  # EXPERIMENT WITH THE NEAREST DISTANCE
            screen_dis = self.screen_dis(refer_point)

            self.tune2face(forward_dis, screen_dis, refer_point)
            self.rovio.forward(2)  # rovio move forward one step

            self.get_frame()
            refer_point = self.get_refer_point(obj)  # EXPERIMENT UPDATE REFER POINT WHILE MOVING
            forward_dis = self.forward_dis(refer_point)

    def move_backward(self, backward_distance, obj):
        self.get_frame()
        initial_refer_point = self.get_refer_point(obj)
        forward_dis = self.forward_dis(initial_refer_point)
        refer_point = initial_refer_point

        while forward_dis < backward_distance:  # EXPERIMENT WITH THE BACKWARD DISTANCE
            screen_dis = self.screen_dis(refer_point)

            self.tune2face(forward_dis, screen_dis, refer_point)
            self.rovio.backward(2)  # rovio move backward one step

            self.get_frame()
            refer_point = self.get_refer_point(obj)  # EXPERIMENT UPDATE REFER POINT WHILE MOVING
            forward_dis = self.forward_dis(refer_point)

    def forward_dis(self, refer_point):
        # return the distance from bottom center of screen to refer point of the object
        return self.distance(refer_point[0], refer_point[1], self.center[0], self.center[1])

    def screen_dis(self, refer_point):
        # return the distance from screen center line to refer_point of the object
        return refer_point[0] - self.center[0]

    def tune2face(self, forward_dis, screen_dis, refer_point):
        turn_degree = math.asin(abs(screen_dis) / forward_dis)
        if screen_dis > 10:  # refer_point at right side of screen
            # while screen_dis > 0:
            # rovio turn right 5 degree self.rovio.api.manual_drive(6, 0.3, 5)
            # alternatie method: turn right math.asin(screen_dis/forward_dis) - 3 error degree   EXPERIMENT TURN SCREEN ANGLE = REAL LIFE ANGLE
            self.rovio.rotate_right(speed=0.3, angle=turn_degree)
        elif screen_dis < -10:
            # while screen_dis < 0:
            # rovio turn left 5 degree self.rovio.api.manual_drive(5, 0.3, 5)
            # alternatie method: turn left math.asin(screen_dis/forward_dis) - 3 error degree	EXPERIMENT TURN SCREEN ANGLE = REAL LIFE ANGLE
            self.rovio.rotate_left(speed=0.3, angle=turn_degree)

    def hide_behind(self, move_str_direction):
        self.get_frame()
        refer_point_seeker = self.get_refer_point('rovio')

        while refer_point_seeker != 'No Rovio Found':  # roviodetection.get_refer_point(frame) != 'No Rovio Found':            #EXPERIMENT TO NOT UPDATE REFER POINT WHILE HIDING
            forward_dis = self.forward_dis(refer_point_seeker)
            screen_dis = self.screen_dis(refer_point_seeker)
            self.tune2face(forward_dis, screen_dis, refer_point_seeker)
            # move_direction depends on blind spot for one step
            if move_str_direction == 1:
                self.rovio.right(1)
            else:
                self.rovio.left(1)

            self.get_frame()
            refer_point_seeker = self.get_refer_point('rovio')

        if move_str_direction == 1:
            self.rovio.right(2)  # EXPERIMENT HOW MUCH MORE STEP TO MOVE TO COMPLETELY HIDE
        else:
            self.rovio.left(2)

    def loop(self):
        # STEP 1: turn 360, detect the angle seeker and obstacles
        print ('detect surrouding')
        distance_obstacles, angle_obstacles, angle_rovio = self.detection_surrounding()
        # find the nearest obstacle
        # find the blind spot of the robot of that obstacle (at left or right side) (-1 or 1 direction)
        print ('blind spot')
        blind_spot, angle_nearest_obstacle = self.find_blind_spot(distance_obstacles, angle_obstacles, angle_rovio)
        print (blind_spot)
        time.sleep(0.5)
        if blind_spot != -1 and blind_spot != 1:
            # if blind_spot == 2:
            # 	#turn 30 degree right, move forward until seeker is not seen
            # elif blind_spot == -2:
            # 	#turn 90 degree right, move forward few steps
            # elif blind_spot == 3:
            # print ("no seeker, has obstacle, case 3 starts")

            # elif blind_spot == 0:
            # 	if angle_rovio != -1:
            # 		print ("no obstacle, has rovio, case 2 starts, help ... ")
            # 		#move away from rovio
            # 	else:
            # 		while True:
            # 			print ("no seeker, no obstacle, case 1 starts")
            # 			self.rovio.api.manual_drive(18, 0.3,angle_nearest_obstalce)
            pass
        else:
            try:
                print ('manual drive')
                # STEP 2: turn to face the nearest obstacle using the recorded angle_nearest_obstalce
                self.turn2face('rovio', -1)

                self.rovio.rotate_right(speed=0.5, angle=angle_nearest_obstacle)
                time.sleep(0.5)
                print ('nearest distance')
                nearest_distance = 25  # to be determined
                # STEP 3: move forward to the obstacle with center of obstacle as refer point (update refer point after each move)
                self.move_forward(nearest_distance, "obstacle")
                time.sleep(0.5)
                print ('turn 2 face')
                # STEP 4: turn "-direction" until seeker is at center of the screen #seeker = recognize target
                self.turn2face("rovio", -blind_spot)
                time.sleep(0.5)
                print ('move backward')
                # STEP 5: reverse away (50cm) from the seeker with seeker as the refer point
                backward_distance = 25  # to be determined
                self.move_backward(backward_distance, "rovio")

                print ('hide behind')
                # STEP 6: move "direction" until obstacle blocks seeker, seeker as refer point
                # calculate the refer point of the seeker
                self.hide_behind(blind_spot)
            except:
                print ('Error liaoooooooo')
                raise
