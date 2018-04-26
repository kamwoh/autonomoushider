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
from datetime import datetime, date


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
        self.drifting_angle = 90

        self.obstacle_points = None
        self.rovio_point = None
        self.frame = None

    def distance(self, x1, y1, x2, y2):
        return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))

    def get_frame(self):
        #print('====================')
        print('===Getting frame')
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

        #print('rovio_point:', self.rovio_point)
        #print('obstacle_point', self.obstacle_points)
        #print('====================')

    def check_center(self, bounding_box_x1, bounding_box_x2, obj):
        #return self.screen_width - bounding_box_x2 - self.center_bound < bounding_box_x1 < self.screen_width - bounding_box_x2 + self.center_bound
        if obj == 'rovio':
            return self.get_refer_point('rovio') != 'No Rovio Found'
        else:
            return self.get_refer_point('obstacle') != 'No Obstacle Found'

    def get_closest_ob(self, list1):
        #print ('Looking for closest obj')
        ob_Areas = [x['Area'] for x in list1]
        i = ob_Areas.index(max(ob_Areas))
        #print ('index of max area', i)
        return i

    def get_refer_point(self, obj):
        if obj == 'obstacle':
            list1 = self.obstacle_points
            if list1 == 'No Obstacle Found':
                return list1

            ind_closest = self.get_closest_ob(list1)
            return list1[ind_closest]['Refer_point']

        else:
            list1 = self.rovio_point
            if list1 == 'No Rovio Found':
                return list1

            return list1['Refer_point']

    def move_forward(self, nearest_distance, obj):
        self.get_frame()
        initial_refer_point = self.get_refer_point(obj)
        forward_dis = self.forward_dis(initial_refer_point) #ASSUMING THAT OBJECT WILL NOT GO OUT OF SCREEN 
        refer_point = initial_refer_point

        while forward_dis > nearest_distance:  # EXPERIMENT WITH THE NEAREST DISTANCE
            screen_dis = self.screen_dis(refer_point)
            print('>>>>forward distance =', forward_dis)
            print('>>>>forward distance - nearest distance =', forward_dis - nearest_distance)
            self.tune2face(forward_dis, screen_dis)
            self.rovio.stop()
            time.sleep(0.1)
            for x in range(3):
                self.rovio.forward(1)  # rovio move forward one step
                time.sleep(0.1)
            time.sleep(0.5)
            self.get_frame()
            refer_point = self.get_refer_point(obj)  # EXPERIMENT UPDATE REFER POINT WHILE MOVING
            forward_dis = self.forward_dis(refer_point)

    def move_backward(self, backward_steps, obj):
        if backward_steps == 0:
            return

        self.get_frame()
        initial_refer_point = self.get_refer_point(obj)
        forward_dis = self.forward_dis(initial_refer_point)
        refer_point = initial_refer_point
        original_distance = forward_dis
        steps = 0
        steps_limit = backward_steps
        if steps_limit == 4:
            steps_limit = 3

        print('>>>>Original distance:', original_distance)
        print('>>>>forward distance =', forward_dis)
        print('>>>>forward distance - original distance =', forward_dis - original_distance)
        
        while steps < steps_limit:  # EXPERIMENT WITH THE BACKWARD DISTANCE
            screen_dis = self.screen_dis(refer_point)
            self.tune2face(forward_dis, screen_dis)
            for x in range(6):
                self.rovio.backward(1)  # rovio move backward one step
                time.sleep(0.1)
            time.sleep(0.5)
            steps += 1
            self.get_frame()
            refer_point = self.get_refer_point(obj)  # EXPERIMENT UPDATE REFER POINT WHILE MOVING
            forward_dis = self.forward_dis(refer_point)

    def forward_dis(self, refer_point):
        # return the distance from bottom center of screen to refer point of the object
        print('refer_point_rovio:', refer_point)
        return self.distance(refer_point[0], refer_point[1], self.center[0], self.screen_height-1)

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
        no_adjust = False
        if refer_point_seeker == 'No Rovio Found':
            no_adjust = True

        while refer_point_seeker != 'No Rovio Found':  # EXPERIMENT TO UPDATE REFER POINT WHILE HIDING
            forward_dis = self.forward_dis(refer_point_seeker)
            screen_dis = self.screen_dis(refer_point_seeker)
            self.tune2face(forward_dis, screen_dis)
            # move_direction depends on blind spot for one step
            print('>>>>>>>>>>>>>Move one step straight')
            if move_str_direction == 1:
                print('>>>>>>>>>>>>>right')
                for x in range(1):
                    self.rovio.right(1)
                time.sleep(0.25)
            else:
                print('>>>>>>>>>>>>>left')
                for x in range(1):
                    self.rovio.left(1)
                time.sleep(0.25)

            time.sleep(0.5)
            self.get_frame()
            refer_point_seeker = self.get_refer_point('rovio')

        if no_adjust is False:
            print('>>>>>>>>>>>>>>Lost vision of rovio')
            print('>>>>>>> Hide the other half of the body by going 2 steps to the ')
            for y in range(2):
                if move_str_direction == 1:
                    for x in range(2):
                        print('>>>>>>>>>>>>>right')
                        self.rovio.right(4)
                        time.sleep(0.25)
                    self.rovio.rotate_right_lag(0.5, 4, 20)
                else:
                    for x in range(2):
                        print('>>>>>>>>>>>>>left')
                        self.rovio.left(4)
                        time.sleep(0.25)
                    self.rovio.rotate_left_lag(0.5, 4, 20)

    def turn2face_rovio2(self, turn_dir):
        # ASSUMING THERE IS ONLY ONE ROVIO
        rovio_angle = -1
        self.get_frame()
        if not (self.check_center(-1, -1, 'rovio')):
            for x in range(1, 11):
                if turn_dir == 1:
                    self.rovio.rotate_right(speed=4)
                else:
                    self.rovio.rotate_left(speed=4)
                time.sleep(self.delay_time * 2)
                self.get_frame()

                if self.check_center(-1, -1, 'rovio'):
                    rovio_angle = x * 36
                    break
        else:
            return 0

        return rovio_angle

    def tune2face2(self, obj):
        print('---------------------Starting tune 2 face')
        self.get_frame()
        rpo = self.get_refer_point(obj)
        forward_dis = self.forward_dis(rpo)
        screen_dis = self.screen_dis(rpo)
        print('forward_dis:', forward_dis)
        print('screen_dis:', screen_dis)
        turn_degree = math.degrees(math.asin(abs(screen_dis) / forward_dis))
        print('----- tune degree', turn_degree)
        act_turn_degree = turn_degree * 0.3
        print('------ actual tune degree', act_turn_degree)
        if act_turn_degree > 10:
            if screen_dis > 10:
                print('tuning right ... ')
                self.rovio.rotate_right_lag(tm=self.delay_time * turn_degree / 36, speed=0.2, angle=act_turn_degree)
            elif screen_dis < -10:
                print('tuning left ... ')
                self.rovio.rotate_left_lag(tm=self.delay_time * turn_degree / 36, speed=0.2, angle=act_turn_degree)
        else:
            print('angle < 10 no need to tune ...')

        self.rovio.stop()
        self.get_frame()
        rpr = self.get_refer_point(obj)
        print('new refer_point', rpr)
        print('----------------------tune 2 face ended')

    def find_blind_spot2(self, rovio_angle):
        if 35 < rovio_angle < 144:
            return 1
        elif 216 < rovio_angle < 325:
            return -1
        elif rovio_angle == 144 or rovio_angle == 180 or rovio_angle == 216:
            return 2
        else:
            return 0

    def adjust_for_180(self, blind_spot, rovio_angle):
        if blind_spot == 1:
            if rovio_angle/36 > 1:
                backward_steps = 2
            else:
                backward_steps = 1
        elif blind_spot == -1:
            if rovio_angle/36 < 9:
                backward_steps = 2
            else:
                backward_steps = 1
        elif blind_spot == 0:
            backward_steps = 0
        else:
            backward_steps = 3

        if blind_spot == 2:
            print('Blind spot = 2, adjusting_for_180')
            for y in range(3):
                if rovio_angle == 180 or rovio_angle == 216:
                    for x in range(3):
                        print('>>>>>>>>>>>>>right')
                        self.rovio.right(4)
                        time.sleep(0.25)
                    self.rovio.rotate_right_lag(0.5, 4, 20)
                    self.turn2face_rovio2(-1)
                    blind_spot = -1
                else:
                    for x in range(3):
                        print('>>>>>>>>>>>>>left')
                        self.rovio.left(4)
                    time.sleep(0.25)
                    self.rovio.rotate_left_lag(0.5, 4, 20)
                    self.turn2face_rovio2(1)
                    blind_spot = 1

            if rovio_angle != -1:
                self.tune2face2('rovio')

        return backward_steps, blind_spot

    def adjust_hiding(self, nearest_distance):
        self.tune2face2('obstacle')
        moved = False

        self.get_frame()
        initial_refer_point = self.get_refer_point('obstacle')
        forward_dis = self.forward_dis(initial_refer_point)  # ASSUMING THAT OBJECT WILL NOT GO OUT OF SCREEN
        refer_point = initial_refer_point

        while forward_dis > nearest_distance:  # EXPERIMENT WITH THE NEAREST DISTANCE
            moved = True
            screen_dis = self.screen_dis(refer_point)
            print('>>>>forward distance =', forward_dis)
            print('>>>>forward distance - nearest distance =', forward_dis - nearest_distance)
            self.tune2face(forward_dis, screen_dis)
            self.rovio.stop()
            time.sleep(0.1)
            for x in range(3):
                self.rovio.forward(1)  # rovio move forward one step
                time.sleep(0.1)
            time.sleep(0.5)
            self.get_frame()
            refer_point = self.get_refer_point('obstacle')  # EXPERIMENT UPDATE REFER POINT WHILE MOVING
            forward_dis = self.forward_dis(refer_point)

        if moved is False:
            backward_steps = 1

            self.get_frame()
            initial_refer_point = self.get_refer_point('obstacle')
            forward_dis = self.forward_dis(initial_refer_point)
            refer_point = initial_refer_point
            original_distance = forward_dis
            steps = 0
            steps_limit = backward_steps

            print('>>>>Original distance:', original_distance)
            print('>>>>forward distance =', forward_dis)
            print('>>>>forward distance - original distance =', forward_dis - original_distance)

            while steps < steps_limit:  # EXPERIMENT WITH THE BACKWARD DISTANCE
                screen_dis = self.screen_dis(refer_point)
                self.tune2face(forward_dis, screen_dis)
                for x in range(3):
                    self.rovio.backward(1)  # rovio move backward one step
                    time.sleep(0.1)
                time.sleep(0.5)
                steps += 1
                self.get_frame()
                refer_point = self.get_refer_point('obstacle')  # EXPERIMENT UPDATE REFER POINT WHILE MOVING
                forward_dis = self.forward_dis(refer_point)
        self.move_forward(110, 'obstacle')

    def loop(self):
        print('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')

        time_start = datetime.now().time()
        print('TIME START: ', time_start)
        print('---- STEP 1: TURN TO FACE AN OBSTACLE------ ')
        #STEP1: TURN TO FACE AN OBSTACLE
        #IF OBSTALCE IS NOT IN ONE FRAME WITH ROVIO
        #ELSE FIND ANOTHER OBSTACLE BY CONTINUE TURNING
        self.get_frame()
        while not (self.check_center(-1, -1, 'obstacle')):
            not_found = True
            for x in range(1, 11):
                self.rovio.rotate_right(speed=4)
                time.sleep(self.delay_time * 2)

                self.get_frame()
                if self.check_center(-1, -1, 'obstacle'):
                    print('>>>>>>> OBSTACLE FOUND!!!! <<<<<<<')
                    not_found = False
                    break

            if not_found is True:
                print('>>>>>>>>>>>>>> Moving to find obstacle <<<<<<<<<<<<')
                for i in range(7):
                    for y in range(5):
                        self.rovio.forward(5)
                    time.sleep(0.5)

        time_step1 = datetime.now().time()
        print('+++++++++ STEP 1 FINISHED +++++++++')            

        print('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')

        print('---- STEP 2: MOVE TOWARDS OBSTACLE ----')
        #STEP 2: MOVE TOWARDS OBSTACLE
        self.move_forward(100, "obstacle")
        time_step2 = datetime.now().time()
        print('+++++++++ STEP 2 FINISHED +++++++++')

        print('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')

        while True:
            #STEP 3: TURN TO FIND ROVIO
            print('---- STEP 3: TURN TO FIND ROVIO ----')
            rovio_angle = -1
            if rovio_angle == -1:
                rovio_angle = self.turn2face_rovio2(-1)

            if rovio_angle != -1:
                self.tune2face2('rovio')
            time_step3 = datetime.now().time()
            print('+++++++++ STEP 3 FINISHED +++++++++')

            print('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')


            print('---- STEP 4: CALCULATE BLIND SPOT ----')
            #STEP 4: CALCULATE BLIND SPOT
            print('==== ROVIO Angle:', rovio_angle)
            blind_spot = self.find_blind_spot2(rovio_angle)
            print('==== BLIND SPOT:', blind_spot)
            time_step4 = datetime.now().time()
            print('+++++++++ STEP 4 FINISHED +++++++++')

            print('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')


            print('---- STEP 5: MOVE BACKWARD FROM ROVIO ----')
            #STEP 5: MOVE BACKWARD
            backward_steps, blind_spot = self.adjust_for_180(blind_spot, rovio_angle)
            print('==== backward steps:', backward_steps)
            self.move_backward(backward_steps, 'rovio')
            time_step5 = datetime.now().time()
            print('+++++++++ STEP 5 FINISHED +++++++++')

            print('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')


            print('---- STEP 6: HIDE AT BLIND SPOT ----')
            #STEP 6: HIDE AT BLIND SPOT
            self.hide_behind(blind_spot)
            self.adjust_hiding(100)
            self.rovio.stop()
            time_step6 = datetime.now().time()
            print('+++++++++ STEP 6 FINISHED +++++++++')

            print('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')

            for x in range(5):
                print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Pausing left: ', 5-x)
                time.sleep(1)

        # time_end = datetime.now().time()
        # print('TIME END:', time_end)
        # print('TOTAL TIME FOR HIDING:', datetime.combine(date.min, time_end) - datetime.combine(date.min, time_start))
        # print('Time_step1 (find ob):', datetime.combine(date.min, time_step1) - datetime.combine(date.min, time_start))
        # print('Time_step2 (move2ob):', datetime.combine(date.min, time_step2) - datetime.combine(date.min, time_step1))
        # print('Time_step3 (find r):', datetime.combine(date.min, time_step3) - datetime.combine(date.min, time_step2))
        # print('Time_step4 (cal bs):', datetime.combine(date.min, time_step4) - datetime.combine(date.min, time_step3))
        # print('Time_step5 (backFromR):', datetime.combine(date.min, time_step5) - datetime.combine(date.min, time_step4))
        # print('Time_step6 (hideBehind):', datetime.combine(date.min, time_step6) - datetime.combine(date.min, time_step5))
