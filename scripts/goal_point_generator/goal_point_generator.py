#! /usr/bin/env python

########################################################################################################################
# This file contains functions for goal point/trajectory generation
#   public functions:
#   __init__
#   get_goal_point
#   use_goal_point_list
#
#
#   helper functions:
#   gpl1
#   gpl2
#   gpl3
#
########################################################################################################################
import math
import rospy
from time import *
import numpy as np

import random
from geometry_msgs.msg import PoseStamped as pose_stamped_msg_type
from geometry_msgs.msg import Pose as pose_msg_type


class goal_point_generator(object):

    def __init__(self, index):

        self.goal_point_list = []
        self.current_goal_point_index = 0

        self.trajectory_class = False

        self.current_gpl_index = 0

        #use std goal point list at beginning
        self.use_goal_point_list(index)


        self.end_of_trajectory = False

        self.init = True

    def get_goal_point(self, random_goal_point, random_orientation):
        #print("Goal Point Generator: Generating Goal")

        self.goal_pose = pose_msg_type()

        if random_goal_point:
            rnd_index = random.randint(1,(len(self.goal_point_list)-1))
            point = self.goal_point_list[rnd_index]
        else:
            point = self.goal_point_list[self.current_goal_point_index]

        if random_orientation:
            rot_index = random.randint(1, 8)
        else:
            rot_index = 1

        x_cord_cm = point[0]
        y_cord_cm = point[1]

        x_chord_m = float(x_cord_cm) / 100
        y_chord_m = float(y_cord_cm) / 100

        self.goal_pose.position.x = x_chord_m

        self.goal_pose.position.y = y_chord_m

        self.goal_pose.position.z = 0.0

        if rot_index == 1:
            self.goal_pose.orientation.x = 0.0
            self.goal_pose.orientation.y = 0.0
            self.goal_pose.orientation.z = 0.0
            self.goal_pose.orientation.w = 1.0
        elif rot_index == 2:
            self.goal_pose.orientation.x = 0.0
            self.goal_pose.orientation.y = 0.0
            self.goal_pose.orientation.z = 0.383
            self.goal_pose.orientation.w = 0.924
        elif rot_index == 3:
            self.goal_pose.orientation.x = 0.0
            self.goal_pose.orientation.y = 0.0
            self.goal_pose.orientation.z = 0.707
            self.goal_pose.orientation.w = 0.707
        elif rot_index == 4:
            self.goal_pose.orientation.x = 0.0
            self.goal_pose.orientation.y = 0.0
            self.goal_pose.orientation.z = 0.924
            self.goal_pose.orientation.w = 0.383
        elif rot_index == 5:
            self.goal_pose.orientation.x = 0.0
            self.goal_pose.orientation.y = 0.0
            self.goal_pose.orientation.z = 1.0
            self.goal_pose.orientation.w = 0.0
        elif rot_index == 6:
            self.goal_pose.orientation.x = 0.0
            self.goal_pose.orientation.y = 0.0
            self.goal_pose.orientation.z = 0.924
            self.goal_pose.orientation.w = -0.383
        elif rot_index == 7:
            self.goal_pose.orientation.x = 0.0
            self.goal_pose.orientation.y = 0.0
            self.goal_pose.orientation.z = 0.707
            self.goal_pose.orientation.w = -0.707
        elif rot_index == 8:
            self.goal_pose.orientation.x = 0.0
            self.goal_pose.orientation.y = 0.0
            self.goal_pose.orientation.z = -0.383
            self.goal_pose.orientation.w = 0.924

        print("Goal Point Generator: X: ",self.goal_pose.position.x,"Y: ",self.goal_pose.position.y)

        self.current_goal_point_index = self.current_goal_point_index + 1

        if self.current_goal_point_index >= len(self.goal_point_list):
            print("Goal Point Generator: resetting goal point index!")
            self.current_goal_point_index = 0
            self.end_of_trajectory = True

        return self.goal_pose

    def use_goal_point_list(self, index):
        print("Goal Point Generator: using goal point list " + str(index))

        if index == 1:
            #print("gpl1")
            self.__gpl1__()

        elif index == 2:
            #print("gpl2")
            self.__gpl2__()

        elif index == 3:
            #print("gpl3")
            self.__gpl3__()

        elif index == 4:
            # print("gpl4")
            self.__gpl4__()

        elif index == 5:
            # print("gpl5")
            self.__gpl5__()

        elif index == 6:
            # print("gpl6")
            self.__gpl6__()

        elif index == 7:
            # print("gpl7")
            self.__gpl7__()

        elif index == 11:
            # print("gpl11")
            self.__gpl11__()

        elif index == 12:
            # print("gpl12")
            self.__gpl12__()

        elif index == 14:
            # print("gpl12")
            self.__gpl14__()

        else:
            print("GPL ERROR: unknown index!")

        self.current_gpl_index = index

    def is_point_end_of_trajectory(self):
        return self.end_of_trajectory

    def is_trajectory(self):
        return self.trajectory_class

    def __gpl1__(self):

        self.trajectory_class = False

        #only create points if newly selected
        if not self.current_gpl_index == 1:
            self.goal_point_list = []

            #print("Goal Point Generator: creating goal point list 1")
            # X and Y positive
            for x in range(10, 70, 1):
                for y in range(10, 70, 1):
                    self.goal_point_list.append([x, y])
                    self.goal_point_list.append([-x, y])
                    self.goal_point_list.append([x, -y])
                    self.goal_point_list.append([-x, -y])
                print("Goal Point Generator: created " + str(len(self.goal_point_list)) + " goal points")

        self.current_goal_point_index = 0

    def __gpl2__(self):

        # only create points if newly selected
        if not self.current_gpl_index == 2:
            self.goal_point_list = []
            # X and Y positive
            for x in range(150, 500, 10):
                    self.goal_point_list.append([x, 0])

            print("Goal Point Generator: created " + str(len(self.goal_point_list)) + " goal points")

        self.trajectory_class = True
        self.current_goal_point_index = 0
        self.end_of_trajectory = False

    def __gpl3__(self):

        # only create points if newly selected
        if not self.current_gpl_index == 3:
            self.goal_point_list = []
            # X and Y positive
            for x in range(150, 300, 10):
                    self.goal_point_list.append([x, 0])
            for y in range(0, 600, 10):
                self.goal_point_list.append([300, -y])
            for x in range(300, 0, -10):
                    self.goal_point_list.append([x, -600])
            for y in range(600, 0, -10):
                self.goal_point_list.append([0, -y])

            print("Goal Point Generator: created " + str(len(self.goal_point_list)) + " goal points")

        self.trajectory_class = True
        self.current_goal_point_index = 0
        self.end_of_trajectory = False

    def __gpl4__(self):

        # Goal list for part 1
        # top left part of local area

        self.goal_point_list = []

        #print("Goal Point Generator: creating goal point list 1")
        # X and Y positive
        if not self.current_gpl_index == 4:
            for x in range(10, 200, 10):
                for y in range(10, 200, 10):
                    self.goal_point_list.append([x, y])

            print("Goal Point Generator: created " + str(len(self.goal_point_list)) + " goal points")

        self.trajectory_class = False
        self.current_goal_point_index = 0

    def __gpl5__(self):

        # Goal list for part 2
        # top right part of local area

        if not self.current_gpl_index == 5:
            self.goal_point_list = []

            # print("Goal Point Generator: creating goal point list 1")
            # X and Y positive
            for x in range(140, 160, 10):
                for y in range(140, 160, 10):
                    self.goal_point_list.append([x, -y])

            print("Goal Point Generator: created " + str(len(self.goal_point_list)) + " goal points")

        self.trajectory_class = False
        self.current_goal_point_index = 0

    def __gpl6__(self):

        # Goal list for part 3
        # bottom left part of local area

        if not self.current_gpl_index == 6:
            self.goal_point_list = []

            # print("Goal Point Generator: creating goal point list 1")
            # X and Y positive
            for x in range(10, 200, 10):
                for y in range(10, 200, 10):
                    self.goal_point_list.append([-x, y])

            print("Goal Point Generator: created " + str(len(self.goal_point_list)) + " goal points")

        self.trajectory_class = False
        self.current_goal_point_index = 0

    def __gpl7__(self):

        # Goal list for part 4
        # bottom right part of local area

        if not self.current_gpl_index == 7:
            self.goal_point_list = []

            # print("Goal Point Generator: creating goal point list 1")
            # X and Y positive
            for x in range(10, 200, 10):
                for y in range(10, 200, 10):
                    self.goal_point_list.append([-x, -y])

            print("Goal Point Generator: created " + str(len(self.goal_point_list)) + " goal points")

        self.trajectory_class = False
        self.current_goal_point_index = 0

    def __gpl11__(self):

        # Goal list for Learning Area 1

        if not self.current_gpl_index == 11:
            self.goal_point_list = []

            # print("Goal Point Generator: creating goal point list 1")
            # X and Y positive
            for x in range(10, 90, 5):
                for y in range(100, 180, 5):
                    self.goal_point_list.append([x, y])

            for x in range(10, 90, 5):
                for y in range(100, 180, 5):
                    self.goal_point_list.append([-x, -y])

            print("Goal Point Generator: created " + str(len(self.goal_point_list)) + " goal points")

        self.trajectory_class = False
        self.current_goal_point_index = 0

    def __gpl12__(self):

        # Goal list for Learning Area 2

        if not self.current_gpl_index == 12:
            self.goal_point_list = []

            # print("Goal Point Generator: creating goal point list 1")
            # X and Y positive
            for x in range(10, 80, 5):
                for y in range(150, 170, 5):
                    self.goal_point_list.append([-x, -y])

            print("Goal Point Generator: created " + str(len(self.goal_point_list)) + " goal points")

        self.trajectory_class = False
        self.current_goal_point_index = 0

    def __gpl14__(self):

        # Goal list for Learning Area 4

        if not self.current_gpl_index == 14:
            self.goal_point_list = []

            # print("Goal Point Generator: creating goal point list 1")
            # X and Y positive
            for x in range(160, 180, 5):
                for y in range(-10, 30, 5):
                    self.goal_point_list.append([x, y])

            print("Goal Point Generator: created " + str(len(self.goal_point_list)) + " goal points")

        self.trajectory_class = False
        self.current_goal_point_index = 0
