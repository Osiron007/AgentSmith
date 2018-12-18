#! /usr/bin/env python

########################################################################################################################
# This file contains functions for goal point/trajectory generation
#   public functions:
#   __init__
#   spawn_obstacles
#   delete_obstacles
#
#
#   helper functions:
#
#
#
#
########################################################################################################################
import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
import random
from obstacle_spawner import obstacle_spawner


class obstacle_controler(object):

    def __init__(self):


        self.obstacle_handler = obstacle_spawner()

        self.obstacle_name_list = [] #list for all obstacle names


    def spawn_obstacles(self, list_id):
        print("spawning")
        if list_id == 1:
            self.__ol1__()
        elif list_id == 2:
            self.__ol2__()




    def delete_obstacles(self):
        print("deleting")

        for obstacle_name in self.obstacle_name_list:
            self.obstacle_handler.delete_obstacle(obstacle_name)

    def __ol1__(self):
        print("ol1 - for gpl4")

        self.obstacle_name_list = []
        self.obstacle_name_list.append("BS1")
        self.obstacle_name_list.append("BS2")
        self.obstacle_name_list.append("BS3")
        self.obstacle_name_list.append("BS4")
        self.obstacle_name_list.append("BS5")
        # self.obstacle_name_list.append("Person4")
        # self.obstacle_name_list.append("Person5")
        # self.obstacle_name_list.append("Person6")
        # self.obstacle_name_list.append("Person7")

        self.delete_obstacles()

        self.obstacle_handler.spawn_obstacle(id="BS1", type=1, x_world=random.uniform(2, 0.8),
                                             y_world=random.uniform(2, 1),
                                             orientation_index=random.randint(1, 8))
        self.obstacle_handler.spawn_obstacle(id="BS2", type=1, x_world=random.uniform(2, 0.8),
                                             y_world=random.uniform(1.2, 0),
                                             orientation_index=random.randint(1, 8))
        self.obstacle_handler.spawn_obstacle(id="BS3", type=1, x_world=random.uniform(2, 0.8),
                                             y_world=random.uniform(0, -2),
                                             orientation_index=random.randint(1, 8))
        self.obstacle_handler.spawn_obstacle(id="BS4", type=1, x_world=random.uniform(-0.8, -1.5),
                                             y_world=random.uniform(-2, -0.8),
                                             orientation_index=random.randint(1, 8))
        self.obstacle_handler.spawn_obstacle(id="BS5", type=1, x_world=random.uniform(-1.5, -2),
                                             y_world=random.uniform(-2, -0.8),
                                             orientation_index=random.randint(1, 8))
        # self.obstacle_handler.spawn_obstacle(id="BS4", type=2, x_world=random.randint(-2, 4),
        #                                      y_world=random.randint(-6, 1),
        #                                      orientation_index=random.randint(1, 8))
        # self.obstacle_handler.spawn_obstacle(id="BS5", type=2, x_world=random.randint(-2, 4),
        #                                      y_world=random.randint(-6, 1),
        #                                      orientation_index=random.randint(1, 8))
        # self.obstacle_handler.spawn_obstacle(id="BS6", type=2, x_world=random.randint(-2, 4),
        #                                      y_world=random.randint(-6, 1),
        #                                      orientation_index=random.randint(1, 8))
        # self.obstacle_handler.spawn_obstacle(id="BS7", type=2, x_world=random.randint(-2, 4),
        #                                      y_world=random.randint(-6, 1),
        #                                      orientation_index=random.randint(1, 8))



    def __ol2__(self):
        print("ol2 - for gpl5")

        self.obstacle_handler.spawn_obstacle(id="BS1", type=1, x_world=random.randint(-2, 4),
                                             y_world=random.randint(-6, 1),
                                             orientation_index=random.randint(1, 8))
        self.obstacle_handler.spawn_obstacle(id="BS2", type=1, x_world=random.randint(-2, 4),
                                             y_world=random.randint(-6, 1),
                                             orientation_index=random.randint(1, 8))
        self.obstacle_handler.spawn_obstacle(id="BS3", type=1, x_world=random.randint(-2, 4),
                                             y_world=random.randint(-6, 1),
                                             orientation_index=random.randint(1, 8))
        self.obstacle_handler.spawn_obstacle(id="BS4", type=1, x_world=random.randint(-2, 4),
                                             y_world=random.randint(-6, 1),
                                             orientation_index=random.randint(1, 8))
        self.obstacle_handler.spawn_obstacle(id="BS5", type=1, x_world=random.randint(-2, 4),
                                             y_world=random.randint(-6, 1),
                                             orientation_index=random.randint(1, 8))
        self.obstacle_handler.spawn_obstacle(id="BS6", type=1, x_world=random.randint(-2, 4),
                                             y_world=random.randint(-6, 1),
                                             orientation_index=random.randint(1, 8))
        self.obstacle_handler.spawn_obstacle(id="BS7", type=1, x_world=random.randint(-2, 4),
                                             y_world=random.randint(-6, 1),
                                             orientation_index=random.randint(1, 8))

        self.obstacle_name_list = []
        self.obstacle_name_list.append("BS1")
        self.obstacle_name_list.append("BS2")
        self.obstacle_name_list.append("BS3")
        self.obstacle_name_list.append("BS4")
        self.obstacle_name_list.append("BS5")
        self.obstacle_name_list.append("BS6")
        self.obstacle_name_list.append("BS7")