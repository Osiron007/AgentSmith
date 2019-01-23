#! /usr/bin/env python

########################################################################################################################
# This file contains functions for reward calculation
#   public functions:
#   __init__
#   reset_calculation
#   get_reward
#
#
#   helper functions:
#   get_distance_to_goal
#
########################################################################################################################
import math
from time import *
import numpy as np

class reward_calculator(object):

    def __init__(self, max_distence, timeout, reward_factor, use_crash_prevention, goal_x_area, goal_y_area, goal_area_angle):

        #init np random
        np.random.seed(1337)


        self.reward = 0.0
        self.episode_done = False
        self.timeout_occurred = False


        self.last_distance_to_goal = 0.0
        self.nearest_distance_to_goal = 0.0
        self.max_dist = max_distence
        self.distance_traveled = 0.0

        #goal angle
        self.last_angel_diff = 0.0
        self.reward_factor_angle = 0.2

        self.reward_factor = reward_factor

        #timing
        self.use_timeout = True
        self.timeout = timeout
        self.time_first_reward = time()
        self.time_last_reward = time()
        self.time_since_last_reward = 0.0
        self.reward_time_factor = 1.0 #value which is subtracted from reward depended on time
                                      #so robot has to find the quickest path

        #translatation tolerance
        self.goal_area_x = goal_x_area
        self.goal_area_y = goal_y_area

        #TODO goal_angle_tolerance
        #orientation tolerance quaternion (since v2)
        #use goal_area_angle
        self.goal_orientation_tolerance_x = 0.05
        self.goal_orientation_tolerance_y = 0.05
        self.goal_orientation_tolerance_z = 0.05
        self.goal_orientation_tolerance_w = 0.05


        #Crash Prevention
        self.use_crash_prevention = use_crash_prevention
        self.crash_prevention_occurred = False

        #Backward penalty
        self.use_backward_penalty = True

        #Misc
        self.goal_area_reached_cnt = 0
        self.first_reward = True
        self.init = True

    def reset_calculation(self):
        print("REWARD CALCULATION: reset calculation")
        #reset all values
        self.first_reward = True
        self.episode_done = False
        self.goal_area_reached_cnt = 0
        self.reward = 0.0
        self.last_distance_to_goal = 0.0
        self.nearest_distance_to_goal = 0.0
        self.distance_traveled = 0.0

        self.time_since_last_reward = 0.0

    def get_distance_to_goal(self, robot_position_map_frame, robot_goal_map_frame):
        #print("Calc dist to goal")
        dist_x = robot_goal_map_frame.position.x - robot_position_map_frame.pose.position.x
        dist_y = robot_goal_map_frame.position.y - robot_position_map_frame.pose.position.y

        distance = math.sqrt(math.pow(dist_x,2)+math.pow(dist_y,2))

        return distance

    def calc_starting_distance_for_new_goal(self, robot_position_map_frame, robot_goal_map_frame):
        self.last_distance_to_goal = self.get_distance_to_goal(robot_position_map_frame,robot_goal_map_frame)
        self.first_reward = False
        self.time_first_reward = time()


    def get_reward(self, robot_position_map_frame, robot_goal_map_frame, s_t1):
        # This Function handles the reward for all possibilities

        # undo normalization process
        angle_diff = s_t1[4] * 180
        wheel_left = s_t1[0] * 4
        wheel_right = s_t1[1] * 4

        distance_to_goal = 0

        if self.first_reward:
            # store time for last reward
            self.time_first_reward = time()
            # calc distance to goal
            distance_to_goal = self.get_distance_to_goal(robot_position_map_frame, robot_goal_map_frame)
            self.last_distance_to_goal = distance_to_goal

            self.last_angel_diff = angle_diff

            self.reward = -0.1
            self.episode_done = False
            self.first_reward = False

        else:
            # reset reward
            self.reward = 0.0
            self.episode_done = False

            distance_to_goal = self.get_distance_to_goal(robot_position_map_frame, robot_goal_map_frame)

            # calc distance traveled. This can be negative if robot has moved away from goal
            self.distance_traveled = self.last_distance_to_goal - distance_to_goal

            # add reward for distance traveled
            self.reward = self.reward + self.reward_factor * self.distance_traveled

            ###################################
            #        Crash Prevention         #
            ###################################
            if self.use_crash_prevention:
                print("REWARD CALCULATION: using crash prevention")
                # check if robot is too close to the border
                if math.fabs(robot_position_map_frame.pose.position.x) > 3.0 or math.fabs(
                        robot_position_map_frame.pose.position.y) > 3.0:
                    self.episode_done = True
                    self.reward = self.reward + 999
                    print("REWARD CALCULATION: crash prevention active")

            ###################################
            #            TimeOut              #
            ###################################
            if self.use_timeout:
                # check if timeout
                if (time() - self.time_first_reward) > self.timeout:
                    print("REWARD CALCULATION: TimeOut!")
                    self.episode_done = True
                    self.reward = self.reward + 999

            ###################################
            #             GOAL                #
            ###################################
            # check if robot is in goal area
            #   => goal area is with in self.goal_area cm around the goal
            if distance_to_goal < self.goal_area_x:
                # print("REWARD CALCULATION: robot is in goal area!")

                self.reward = self.reward + 0.2

                # in goal area robot should rotate to reach goal position
                angle_rotated = 0.0

                # first check if goal angle is reached
                if math.fabs(angle_diff) < 5:

                    #check wheel vel command
                    #wheel_left = s_t1[0]   #[-1.1]
                    #wheel_right = s_t1[1]  #[-1.1]

                    #Left Wheel
                    if s_t1[0] > 0:
                        diffL = s_t1[0]
                    else:
                        diffL = s_t1[0] * (-1)

                    # Right Wheel
                    if s_t1[1] > 0:
                        diffR = s_t1[1]
                    else:
                        diffR = s_t1[1] * (-1)

                    #print("REWARD CALCULATION: diffL:" + str(diffL) + " diffR:" + str(diffR))

                    discount_factor = ((diffL + diffR)/2)*0.2

                    reward_wheels = (0.2 - discount_factor) * 10

                    #print("REWARD CALCULATION: RWheels: "+ str(reward_wheels))

                    self.reward = self.reward + reward_wheels


                    # robot has to stay in goal position for at least 3 cycles
                    self.goal_area_reached_cnt = self.goal_area_reached_cnt + 1
                    print("REWARD CALCULATION: goal area reached cnt " + str(self.goal_area_reached_cnt))

                    if self.goal_area_reached_cnt >= 10:
                        self.reward = self.reward + 100
                        self.episode_done = True
                        print("REWARD CALCULATION: GOAL REACHED!")

                else:
                    # check how angle should be rewarded:
                    if angle_diff > 0 and self.last_angel_diff > 0:
                        angle_rotated = self.last_angel_diff - angle_diff

                    if angle_diff < 0 and self.last_angel_diff < 0:
                        angle_rotated = (math.fabs(self.last_angel_diff) - math.fabs(angle_diff))

                    if angle_diff > 0 and self.last_angel_diff < 0:
                        if angle_diff > 90:
                            angle_rotated = 180 - angle_diff
                        else:
                            angle_rotated = angle_diff

                    if angle_diff < 0 and self.last_angel_diff > 0:
                        if self.last_angel_diff > 90:
                            angle_rotated = 180 - self.last_angel_diff
                        else:
                            angle_rotated = self.last_angel_diff

                    # add reward for angle rotated
                    self.reward = self.reward + self.reward_factor_angle * angle_rotated

                    if self.goal_area_reached_cnt > 0:
                        print("REWARD CALCULATION: Robot has moved out of goal area => penalty -0.2")
                        # robot was in goal area and has move away
                        # substract penalty
                        self.reward = self.reward - 2
                    # robot is not in goal area => reset cnt
                    self.goal_area_reached_cnt = 0


            # store current distance for next reward calculation
            self.last_distance_to_goal = distance_to_goal

            self.last_angel_diff = angle_diff

            ###################################
            #        Backward Penalty         #
            ###################################
            if self.use_backward_penalty:
                # substract backward cost if robot is moving backwards
                if wheel_left < 0 and wheel_right > 0:
                    print("REWARD CALCULATION: Robot is moving backwards => penalty -0.1")
                    self.reward = self.reward - 1

            # substract action cost
            self.reward = self.reward - 1


        if self.episode_done:
            print("REWARD CALCULATION: Episode done")

        self.time_last_reward = time()


        return [self.reward, self.episode_done, distance_to_goal]