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
import cv2
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
        self.center_bound = 100   #50~100
        self.center = [self.screen_width / 2, self.screen_height / 2]
        self.delay_time = 0.5  # delay time for 
        self.rovio = rovio.Rovio(host)

        self.obstacle_points = None
        self.rovio_point = None
        self.frame = None

    def distance(self, x1, y1, x2, y2):
        return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))

    def get_frame(self):
        print('====================')
        print('Getting frame')
        self.frame = None
        while self.frame is None:
            ret, self.frame = self.rovio.camera.get_frame()
        self.rovio_point = self.rovio_detection.get_refer_point(self.frame)
        self.obstacle_points = self.obstacle_detection.get_refer_point(self.frame)
        if self.frame is not None:
            cv2.imshow('obstacle', self.frame)
        if self.rovio_detection.disp_img is not None:
            cv2.imshow('rovio', self.rovio_detection.disp_img)
        cv2.waitKey(1)

        print('rovio_point:', self.rovio_point)
        print('obstacle_point', self.obstacle_points)
        print('====================')

    def detection_surrounding(self):
        angle_obstacles = []
        distance_obstacles = []
        bxs_rovio = -1, -1
        angle_rovio = -1
        max_seeker = None
        turn_angle = 36
        for x in range(10):
            self.get_frame()
            print ('Current angle:', x * turn_angle)

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

            self.rovio.rotate_right(speed=4)  # turn 36 degree right
            print ('Turning 36 degree to the right')
            time.sleep(self.delay_time+0.5)  # EXPERIEMNT !!!!! FOR EVERY 36 DEGREE WITH 4 SPEED, NEED 0.5 SEC TO TURN
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

    def check_center(self, bounding_box_x1, bounding_box_x2, obj):
        #return self.screen_width - bounding_box_x2 - self.center_bound < bounding_box_x1 < self.screen_width - bounding_box_x2 + self.center_bound
        if obj == 'rovio':
            return self.get_refer_point('rovio') != 'No Rovio Found'
        else:
            return self.get_refer_point('obstacle') != 'No Obstacle Found'

    def look_check_center(self, bx1, bx2, obj):
        i = 100
        saw_obj = False
        while (not self.check_center(bx1, bx2, obj)) and i >= 0:
            print (i, 'running check center')
            self.get_frame()
            bx1, bx2 = self.get_bxs(obj)
            if bx1 <= 10 and bx2 >= self.screen_width-10:
                print ('------------------ OBJECT IS TOO CLOSE, ASSUMING IT IT AT THE CENTER OF SCREEN')
                break
            i -= 1
            if i == 0 and not (self.check_center(bx1, bx2, obj)):
                print('>>>>>>>>>>>>>>>>>>>>>>>>>>> Exiting program ... ERROR: rovio not found after 360')
                exit()

        print('--------------- Stopping rovio, since obj is found at the center of screen')
        saw_obj = True
        self.rovio.stop()
        return bx1, bx2, saw_obj

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

    def turn2face_rovio(self, angle_rovio=None, bxs=None, turn_direction=None, num_correction=0):
        if turn_direction is None:
            turn_direction = 0
        bx1, bx2 = -1, -1
        if not (self.check_center(bx1, bx2, 'rovio')):
            if angle_rovio is not None and turn_direction == 0:
                bx1, bx2 = self.get_bxs('rovio')
                # turn to face the previously recognized rovio
                if 40 < angle_rovio < 180:
                    # assume less than 40 degree dift after detecting surrounding
                    self.rovio.rotate_right(speed=0.2, angle=360)
                    turn_direction = 1
                else:
                    self.rovio.rotate_left(speed=0.2, angle=360)
                    turn_direction = -1
            else:
                if turn_direction == -1:
                    self.rovio.rotate_right(speed=0.2, angle=360)
                else:
                    self.rovio.rotate_left(speed=0.2, angle=360)
        bx1_compare, bx2_compare, saw_obj = self.look_check_center(bx1, bx2, 'rovio')

        self.rovio.stop()
        time.sleep(0.5)
        print('>>>>>>>>>>>>>>>>>>>>>>SAW OBJECT =', saw_obj)

        if saw_obj is False and angle_rovio >= 0:
            num_correction += 1
            print('>>>>>>>>>>>>>>>>>>CORRECTING WITH Number of correction<<<<<<<<<<<<<<<<<<<<<<<<<<', num_correction)
            self.turn2face_rovio(-turn_direction, num_correction=num_correction)
        else:
            if num_correction >= 5:
                print('TOO MUCH CORRECTION !!!')
            exit()

        self.get_frame()
        rpr = self.get_refer_point('rovio')
        if rpr == 'No Rovio Found':
            print('------ TARGET LOST AFTER TURN2FACE --- ')
            if saw_obj is True and num_correction < 5:
                num_correction += 1
                print('>>>>>>>>>>>>>>>>>>CORRECTING WITH Number of correction<<<<<<<<<<<<<<<<<<<<<<<<<<', num_correction)
                self.turn2face_rovio(-turn_direction, num_correction=num_correction)
            else:
                if num_correction >= 5:
                    print('TOO MUCH CORRECTION !!!')
                exit()


        return turn_direction
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
                self.rovio.rotate_right_lag(tm=self.delay_time * diff_angle_abs / 36, speed=0.2, angle=diff_angle_abs)
            else:
                turn_direction = -1
                self.rovio.rotate_left_lag(tm=self.delay_time * diff_angle_abs / 36, speed=0.2, angle=diff_angle_abs)
        else:
            if diff_angle > 0:
                turn_direction = -1
                self.rovio.rotate_left_lag(tm=self.delay_time * diff_angle_abs / 36, speed=0.2,
                                           angle=360 - diff_angle_abs)
            else:
                turn_direction = 1
                self.rovio.rotate_right_lag(tm=self.delay_time * diff_angle_abs / 36, speed=0.2,
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
            if list1 == str:
                print ('......................... REFER POINT LOST ........................')
                exit()
            ind_closest = self.get_closest_ob(list1)
            return list1[ind_closest]['Refer_point']

        else:
            list1 = self.rovio_point
            if list1 == 'No Rovio Found':
                return list1
                #print('......................... REFER POINT LOST ........................')
                #exit()
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
            time.sleep(0.5)
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
            time.sleep(0.5)
            self.get_frame()
            refer_point = self.get_refer_point(obj)  # EXPERIMENT UPDATE REFER POINT WHILE MOVING
            forward_dis = self.forward_dis(refer_point)

    def forward_dis(self, refer_point):
        # return the distance from bottom center of screen to refer point of the object
        print('refer_point_rovio:', refer_point)
        return self.distance(refer_point[0], refer_point[1], self.center[0], 0)

    def screen_dis(self, refer_point):
        # return the distance from screen center line to refer_point of the object
        print('center', self.center)
        return refer_point[0] - self.center[0]

    def tune2face(self, forward_dis, screen_dis):
        print('---------------------Starting tune 2 face')
        print('forward_dis:', forward_dis)
        print('screen_dis:', screen_dis)
        turn_degree = math.degrees(math.asin(abs(screen_dis) / forward_dis))
        print ('----- tune degree', turn_degree)
        act_turn_degree = turn_degree * 0.3
        print('------ actual tune degree', act_turn_degree)
        # - 3 as error degree
        if act_turn_degree > 10:
            if screen_dis > 10:  # refer_point at right side of screen
                # while screen_dis > 0:
                # rovio turn right 5 degree self.rovio.api.manual_drive(6, 0.3, 5)
                # alternatie method: turn right math.asin(screen_dis/forward_dis) - 3 error degree   EXPERIMENT TURN SCREEN ANGLE = REAL LIFE ANGLE
                print('tuning right ... ')
                self.rovio.rotate_right_lag(tm=self.delay_time * turn_degree / 36, speed=0.2, angle=act_turn_degree)

            elif screen_dis < -10:
                # while screen_dis < 0:
                # rovio turn left 5 degree self.rovio.api.manual_drive(5, 0.3, 5)
                # alternatie method: turn left math.asin(screen_dis/forward_dis) - 3 error degree	EXPERIMENT TURN SCREEN ANGLE = REAL LIFE ANGLE
                print('tuning left ... ')
                self.rovio.rotate_left_lag(tm=self.delay_time * turn_degree / 36, speed=0.2, angle=act_turn_degree)
        else:
            print('angle < 10 no need to tune ...')

        self.rovio.stop()
        self.get_frame()
        rpr = self.get_refer_point('rovio')

        print('refer_point_rovio:', rpr)

        print ('----------------------tune 2 face ended')    

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
                time.sleep(self.delay_time)
            else:
                self.rovio.left(1)
                time.sleep(self.delay_time)

            self.get_frame()
            refer_point_seeker = self.get_refer_point('rovio')

        if move_str_direction == 1:
            self.rovio.right(2)  # EXPERIMENT HOW MUCH MORE STEP TO MOVE TO COMPLETELY HIDE
            time.sleep(self.delay_time)
        else:
            self.rovio.left(self.delay_time)
            time.sleep(1)

    def loop(self):

        print ('STEP 1: Detecting surrounding ... ')
        distance_obstacles, angle_obstacles, angle_rovio, bxs_rovio = self.detection_surrounding()
        # find the nearest obstacle
        # find the blind spot of the robot of that obstacle (at left or right side) (-1 or 1 direction)
        print ('------------------------- Calculating blind spot ... ')
        blind_spot, diff_angle = self.find_blind_spot(distance_obstacles, angle_obstacles, angle_rovio)
        print ('-------------------------- Blind_spot:', blind_spot)
        print ('angle rovio:', angle_rovio)
        print ('angle obstacle:', angle_obstacles)
        print ('---------------------------Angle diff btw closest rovio to closest obs:', diff_angle)
        print ('------------------------- Finished STEP 1')
        #exit()
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
            print ('------------------------------------ BLIND SPOT != 1 OR -1 ----------------------------------')
            exit()
        else:
            try:
                print ('STEP 2: Turning to face rovio ...')
                turned = self.turn2face_rovio(angle_rovio, bxs_rovio)  # MIGHT FACING THE WRONG ROVIO
                print ('------------------- STOP TURNING')
                print ('------------------------THERE SHOULD BE A ROVIO IN THE SCREEN, TO BE TUNED LATER')
                time.sleep(5)
                
                print ('------------------- START TUNING 2 FACE ROVIO')
                self.get_frame()
                rpr = self.get_refer_point('Rovio')
                if rpr == 'No Rovio Found':
                    print('------ TARGET LOST AFTER TURN2FACE --- ')
                    exit()
                else:
                    self.tune2face(self.forward_dis(rpr), self.screen_dis(rpr))
                print ('-- Finished STEP 2, now hider is facing rovio')
                exit()

                print ('STEP 3: Turning to face obstacle ...')
                self.turn2face_ob_from_rovio(diff_angle)  # ERROR CAUSE BY DIRFTING
                print ('------------------- START TUNING 2 FACE OBSTACLE')
                rpr = self.get_refer_point('obstacle')
                self.tune2face(self.forward_dis(rpr),self.screen_dis(rpr))
                print ('-- Finished STEP 3, now hider is facing obstacle it will use to hide')
                exit()
                
                nearest_distance = 25  # to be determined
                print ('nearest distance:', nearest_distance)

                print ('STEP 4: Moving towards the obstacle ... ')
                self.move_forward(nearest_distance, "obstacle")
                print ('-- Finished STEP 4, now hider is infront of the obstacle')

                print ('STEP 5: Turn to face the seeker ... ')
                self.turn2face_rovio(turn_direction=-blind_spot)
                print ('-- Finished STEP 5, now hider is facing seeker')

                print ('STEP 6: Moving away from the seeker ... ')
                backward_distance = 25  # to be determined
                self.move_backward(backward_distance, "rovio")
                print ('-- Finished STEP 6, now hider is away from the seeker')

                print ('STEP 7: Hiding behind the obstacle ... ')
                self.hide_behind(blind_spot)
                print ('-- Finished STEP 7, hiding is done')
            except Exception as e:
                print ('Error liaooooooo00000000000000o')
                print (e)
                raise
