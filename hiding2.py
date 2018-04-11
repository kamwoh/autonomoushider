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
from .rovio import rovio


class HidingAlgorithm(object):
    def __init__(self, host):
        self.rovio_detection = detection.RovioDetection()
        self.obstacle_detection = detection.ObstacleDetection()
        self.screen_width = 640
        self.screen_height = 480
        self.center_bound = 20
        self.center = [self.screen_width / 2, self.screen_height / 2]
        self.delay_time = 0.5  # delay time for 20degree with 0.5 speed (fine tune this)
        self.rovio = rovio.Rovio(host)

        self.obstacle_points = None
        self.rovio_point = None
        self.frame = None

    def distance(self, x1, y1, x2, y2):
        return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))

    def get_frame(self):
        print ('get frame?')

        self.frame = None
        while self.frame is None:
            ret, self.frame = self.rovio.camera.get_frame()

        print ('rovio point')
        self.rovio_point = self.rovio_detection.get_refer_point(self.frame)
        print ('obstacle')
        self.obstacle_points = self.obstacle_detection.get_refer_point(self.frame)
        print ('done')
        # cv2.imshow('show', self.frame)
        print ('after show')
        # cv2.waitKey(1)

        print (self.rovio_point)
        print (self.obstacle_points)

    def detection_surrounding(self):
        angle_obstacles = []
        distance_obstacles = []

        angle_rovio = -1
        max_seeker = None
        turn_angle = 20
        for x in range(1, 19):
            self.get_frame()
            print ('Current angle:', x * 20)

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
                    bxs_rovio = self.get_bxs('rovio')

            self.rovio.rotate_right(speed=0.5, angle=20)  # turn 20 degree right
            print ('Turning 20 degree to the right')
            time.sleep(self.delay_time)  # EXPERIEMNT !!!!! FOR EVERY 20 DEGREE WITH 0.5 SPEED, NEED 0.5 SEC TO TURN
            print (
                'Finished turning')  # IF PRINT FINISH TURNING BEFORE REAL LIFE ROVIO FINISHED TURNING, ALL MORE SLEEP TIME
            # DO THE FINE TURN OF THE WAIT TIME HERE
        return distance_obstacles, angle_obstacles, angle_rovio, bxs_rovio

    def find_blind_spot(self, distance_obstacles, angle_obstacles, angle_rovio):
        blind_spot = 0
        angle_nearest_obstacle = -1

        if len(distance_obstacles) != 0:
            angle_nearest_obstacle = angle_obstacles[distance_obstacles.index(min(distance_obstacles))]
        else:
            print ("No Obstacle Found...")
            return 0, 0

        if angle_rovio != -1:
            diff_angle = angle_nearest_obstacle - angle_rovio
            diff_angle_abs = abs(diff_angle)
            if diff_angle_abs > 20 or diff_angle_abs < 160:
                if diff_angle > 0:
                    blind_spot = 1
                else:
                    blind_spot = -1
            elif diff_angle_abs > 200 or diff_angle_abs < 340:
                if diff_angle > 0:
                    blind_spot = -1
                else:
                    blind_spot = 1
            elif diff_angle_abs <= 20 or diff_angle_abs >= 340:
                blind_spot = 2
            else:
                blind_spot = -2
        else:
            return 3, angle_nearest_obstacle

        return blind_spot, diff_angle

    def check_center(self, bounding_box_x1, bounding_box_x2):
        return self.screen_width - bounding_box_x2 - self.center_bound < bounding_box_x1 < self.screen_width - bounding_box_x2 + self.center_bound

    def look_check_center(self, bx1, bx2, obj):
        i = 100
        while (not self.check_center(bx1, bx2)) and i >= 0:
            print (i, 'running check center')
            self.get_frame()
            bx1, bx2 = self.get_bxs(obj)
            i -= 1

        print ('Stopping rovio, since obj is found at the center of screen')
        self.rovio.stop()
        return bx1, bx2

    def get_closest_ob(self, list1):
        print ('Looking for closest obj')
        ob_Areas = [x['Area'] for x in list1]
        i = ob_Areas.index(max(ob_Areas))
        print ('index of max area', i)
        return i

    def get_bxs(self, obj):
        print ('Getting bxs')
        bx1 = -1
        bx2 = -1
        # return bx1 and bx2 equal to -1 if no rovio is found
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

    def turn2face_rovio(self, angle_rovio=None, bxs=None):
        if angle_rovio != None:
            bx1, bx2 = self.get_bxs('rovio')
            # turn to face the previously recognized rovio
            if 40 < angle_rovio < 180:
                # assume less than 40 degree dift after detecting surrounding
                self.rovio.rotate_right(speed=0.2, angle=360)
            else:
                self.rovio.rotate_left(speed=0.2, angle=360)
        else:
            print ('finding any rovio by turning 360')
            self.rovio.rotate_right(speed=0.2, angle=360)
        bx1_compare, bx2_compare = self.look_check_center(bx1, bx2, 'rovio')
        print('Done turning to face rovio')
        print('bx1_found:', bx1_compare)
        print('bx2_found:', bx2_compare)
        print('bxs_recognized:', bxs)
        # ASSUMING THERE WILL HAVE NO OTHER ROVIO IN THE AREA
        # ELSE EXECUTE THE FOLLOWING CODES
        # if not ((bxs[1]-bxs[0])*0.8 < bx2_compare-bx1_compare < (bxs[1]-bxs[0])*1.2):
        #     #if the rovio found is not the one previously recognized
        #     print('Repeating the function turn2face_rovio')
        #     self.turn2face_rovio(angle_rovio, bxs)

    def turn2face_ob_from_rovio(self, diff_angle):
        # turn to face the previously recognized obstacle start from facing a rovio
        diff_angle_abs = abs(diff_angle)
        if diff_angle_abs > 20 or diff_angle_abs < 160:
            if diff_angle > 0:
                turn_direction = 1
                self.rovio.rotate_right_lag(tm=self.delay_time * diff_angle_abs / 20, speed=0.5, angle=diff_angle_abs)
            else:
                turn_direction = -1
                self.rovio.rotate_left_lag(tm=self.delay_time * diff_angle_abs / 20, speed=0.5, angle=diff_angle_abs)
        else:
            if diff_angle > 0:
                turn_direction = -1
                self.rovio.rotate_left_lag(tm=self.delay_time * diff_angle_abs / 20, speed=0.5,
                                           angle=360 - diff_angle_abs)
            else:
                turn_direction = 1
                self.rovio.rotate_right_lag(tm=self.delay_time * diff_angle_abs / 20, speed=0.5,
                                            angle=360 - diff_angle_abs)

        print('Correcting the angle after drifting error ... ')
        bx1, bx2 = self.get_bxs('obstacle')
        if turn_direction == 1:
            # turn to face the previously recognized nearest obstacle
            self.rovio.rotate_left(speed=0.2, angle=360)
        else:
            self.rovio.rotate_right(speed=0.2, angle=360)

        self.look_check_center(bx1, bx2, 'obstacle')

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

            self.tune2face(forward_dis, screen_dis)
            self.rovio.forward(2)  # rovio move forward one step
            time.sleep(1)
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

            self.tune2face(forward_dis, screen_dis)
            self.rovio.backward(2)  # rovio move backward one step
            time.sleep(1)
            self.get_frame()
            refer_point = self.get_refer_point(obj)  # EXPERIMENT UPDATE REFER POINT WHILE MOVING
            forward_dis = self.forward_dis(refer_point)

    def forward_dis(self, refer_point):
        # return the distance from bottom center of screen to refer point of the object
        return self.distance(refer_point[0], refer_point[1], self.center[0], self.center[1])

    def screen_dis(self, refer_point):
        # return the distance from screen center line to refer_point of the object
        return refer_point[0] - self.center[0]

    def tune2face(self, forward_dis, screen_dis):
        turn_degree = math.asin(abs(screen_dis) / forward_dis) - 3
        # - 3 as error degree
        if screen_dis > 10:  # refer_point at right side of screen
            # while screen_dis > 0:
            # rovio turn right 5 degree self.rovio.api.manual_drive(6, 0.3, 5)
            # alternatie method: turn right math.asin(screen_dis/forward_dis) - 3 error degree   EXPERIMENT TURN SCREEN ANGLE = REAL LIFE ANGLE
            self.rovio.rotate_right_lag(tm=self.delay_time * turn_degree / 20, speed=0.5, angle=turn_degree)
        elif screen_dis < -10:
            # while screen_dis < 0:
            # rovio turn left 5 degree self.rovio.api.manual_drive(5, 0.3, 5)
            # alternatie method: turn left math.asin(screen_dis/forward_dis) - 3 error degree	EXPERIMENT TURN SCREEN ANGLE = REAL LIFE ANGLE
            self.rovio.rotate_left_lag(tm=self.delay_time * turn_degree / 20, speed=0.5, angle=turn_degree)

    def hide_behind(self, move_str_direction):
        self.get_frame()
        refer_point_seeker = self.get_refer_point('rovio')

        while refer_point_seeker != 'No Rovio Found':  # EXPERIMENT TO UPDATE REFER POINT WHILE HIDING
            forward_dis = self.forward_dis(refer_point_seeker)
            screen_dis = self.screen_dis(refer_point_seeker)
            self.tune2face(forward_dis, screen_dis)
            # move_direction depends on blind spot for one step
            if move_str_direction == 1:
                self.rovio.right(1)
                time.sleep(1)
            else:
                self.rovio.left(1)
                time.sleep(1)

            self.get_frame()
            refer_point_seeker = self.get_refer_point('rovio')

        if move_str_direction == 1:
            self.rovio.right(2)  # EXPERIMENT HOW MUCH MORE STEP TO MOVE TO COMPLETELY HIDE
            time.sleep(1)
        else:
            self.rovio.left(2)
            time.sleep(1)

    def loop(self):

        print ('STEP 1: Detecting surrounding ... ')
        distance_obstacles, angle_obstacles, angle_rovio, bxs_rovio = self.detection_surrounding()
        print ('-- Finished STEP 1')
        # find the nearest obstacle
        # find the blind spot of the robot of that obstacle (at left or right side) (-1 or 1 direction)
        print ('-- Calculating blind spot ... ')
        blind_spot, diff_angle = self.find_blind_spot(distance_obstacles, angle_obstacles, angle_rovio)
        print ('-- Blind_spot:', blind_spot)
        print ('-- Angle diff btw closest rovio to closest obs:', diff_angle)

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
                print ('STEP 2: Turning to face rovio ...')
                self.turn2face_rovio(angle_rovio, bxs_rovio)  # MIGHT FACING THE WRONG ROVIO
                print ('-- Finished STEP 2, now hider is facing rovio')

                print ('STEP 3: Turning to face obstacle ...')
                self.turn2face_ob_from_rovio(diff_angle)  # ERROR CAUSE BY DIRFTING
                print ('-- Finished STEP 3, now hider is facing obstacle it will use to hide')

                nearest_distance = 25  # to be determined
                print ('nearest distance:', nearest_distance)

                print ('STEP 4: Moving towards the obstacle ... ')
                self.move_forward(nearest_distance, "obstacle")
                print ('-- Finished STEP 4, now hider is infront of the obstacle')

                print ('STEP 5: Turn to face the seeker ... ')
                self.turn2face_rovio()
                print ('-- Finished STEP 5, now hider is facing seeker')

                print ('STEP 6: Moving away from the seeker ... ')
                backward_distance = 25  # to be determined
                self.move_backward(backward_distance, "rovio")
                print ('-- Finished STEP 6, now hider is away from the seeker')

                print ('STEP 7: Hiding behind the obstacle ... ')
                self.hide_behind(blind_spot)
                print ('-- Finished STEP 7, hiding is done')
            except:
                print ('Error liaoooooooo')
                raise
