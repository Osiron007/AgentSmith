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
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState
from geometry_msgs.msg import *


class obstacle_spawner(object):

    def __init__(self):

        print("Waiting for gazebo services...")
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        rospy.wait_for_service("gazebo/set_model_state")
        print("Services available")

        #create service proxys
        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.set_model_state = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)

        #read sdf files for cubes and zylinder
        with open("/home/nikas/.gazebo/models/bookshelf/model.sdf", "r") as f_bookshelf:
            self.bookshelf_xml = f_bookshelf.read()
        with open("/home/nikas/.gazebo/models/person_standing/model.sdf", "r") as f_person:
            self.person_xml = f_person.read()

    def spawn_obstacle(self, id, type, x_world, y_world, orientation_index):
        print("spawning")

        obstacle_name = id

        if type == 1:
            xml = self.bookshelf_xml
        elif type == 2:
            xml = self.person_xml
        #orientation = Quaternion(tf.transformations.quaternion_from_euler(0, 0, 0))
        if orientation_index == 1:
            obstacle_pose = Pose(Point(x=x_world,y=y_world,z=0.0),Quaternion(x=0,y=0,z=0,w=1))
        elif orientation_index == 2:
            obstacle_pose = Pose(Point(x=x_world,y=y_world,z=0.0),Quaternion(x=0,y=0,z=0.383,w=0.924))
        elif orientation_index == 3:
            obstacle_pose = Pose(Point(x=x_world,y=y_world,z=0.0),Quaternion(x=0,y=0,z=0.707,w=0.707))
        elif orientation_index == 4:
            obstacle_pose = Pose(Point(x=x_world,y=y_world,z=0.0),Quaternion(x=0,y=0,z=0.924,w=0.383))
        elif orientation_index == 5:
            obstacle_pose = Pose(Point(x=x_world,y=y_world,z=0.0),Quaternion(x=0,y=0,z=1,w=0))
        elif orientation_index == 6:
            obstacle_pose = Pose(Point(x=x_world,y=y_world,z=0.0),Quaternion(x=0,y=0,z=0.924,w=-0.383))
        elif orientation_index == 7:
            obstacle_pose = Pose(Point(x=x_world,y=y_world,z=0.0),Quaternion(x=0,y=0,z=0.707,w=-0.707))
        elif orientation_index == 8:
            obstacle_pose = Pose(Point(x=x_world,y=y_world,z=0.0),Quaternion(x=0,y=0,z=-0.383,w=0.924))


        self.spawn_model(obstacle_name, xml, "", obstacle_pose, "world")


    def delete_obstacle(self, id):
        print("deleting")
        self.delete_model(id)
