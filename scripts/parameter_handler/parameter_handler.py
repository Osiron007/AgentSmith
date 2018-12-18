#! /usr/bin/env python

########################################################################################################################
# This file contains functions for goal point/trajectory generation
#   public functions:
#   __init__
#   read_parameter
#   print_parameter
#
#
#
#
#
#
#
########################################################################################################################

import ConfigParser

class param_handler(object):

    def __init__(self):

        self.init = False

        self.config = ConfigParser.ConfigParser()

        # Hyperparameter
        self.state_dim = 0
        self.action_dim = 0
        self.control_rate = 0 #[Hz]

        # Exploration Noise
        self.OU_mu = 0.0
        self.OU_theta = 0.0
        self.OU_sigma = 0.0
        self.epsilon = 0.0
        self.EXPLORE = 0.0
        self.use_joystick = False
        self.use_ANN = False

        # environment
        self.state_space_version = 1
        self.simulation = True

        # robot parameter
        self.wheel_diameter = 0.0
        self.axis_length = 0.0

        # reward calculation
        self.reward_version = 0
        self.goal_max_dist = 0.0
        self.goal_area_x = 0.0
        self.goal_area_y = 0.0
        self.goal_area_angle = 0.0
        self.goal_point_creator_x_area = 0.0
        self.goal_point_creator_y_area = 0.0
        self.goal_point_creator_step = 0.0
        self.distance_reward_factor = 0.0
        self.episode_timeout = 0.0

        #goal point list
        self.gpl_index = 0
        self.gpl_use_random_orientation = False
        self.gpl_use_random_goal = False

        # logging and visualization
        self.visu_debug_infos = False
        self.visu_show_step_data = False
        self.experiment_ID = 99.99
        self.log_path = "NOT PATH DEFINED"


    def read_parameter(self, filename):

        # print("#############Read configuration###################")
        print("OPENING CONFIGURATION AT: " + str(filename))
        self.config.readfp(open(filename))

        # hyperparameter
        self.state_dim = self.config.getint("hyperparameter", "state_dim")
        self.action_dim = self.config.getint("hyperparameter", "action_dim")
        self.control_rate = self.config.getint("hyperparameter", "control_rate")

        # exploration noise
        self.EXPLORE = self.config.getfloat("exploration_noise", "explore")
        self.OU_mu = self.config.getfloat("exploration_noise", "ou_mu")
        self.OU_theta = self.config.getfloat("exploration_noise", "ou_theta")
        self.OU_sigma = self.config.getfloat("exploration_noise", "ou_sigma")
        self.epsilon = self.config.getfloat("exploration_noise", "epsilon")
        self.use_joystick = self.config.getboolean("exploration_noise", "use_joystick")
        self.use_ANN = self.config.getboolean("exploration_noise", "use_ANN")

        # environment
        self.state_space_version = self.config.getint("environment", "state_space_version")
        self.simulation = self.config.getboolean("environment", "simulation")

        # robot parameter
        self.wheel_diameter = self.config.getfloat("robot", "wheel_diameter")
        self.axis_length = self.config.getfloat("robot", "axis_length")

        # reward calculation
        self.reward_version = self.config.getint("reward_calculation", "reward_version")
        self.goal_max_dist = self.config.getfloat("reward_calculation", "goal_max_dist")
        self.goal_area_x = self.config.getfloat("reward_calculation", "goal_area_x")
        self.goal_area_y = self.config.getfloat("reward_calculation", "goal_area_y")
        self.goal_area_angle = self.config.getfloat("reward_calculation", "goal_area_angle")
        self.goal_point_creator_x_area = self.config.getfloat("reward_calculation", "goal_point_creator_x_area")
        self.goal_point_creator_y_area = self.config.getfloat("reward_calculation", "goal_point_creator_y_area")
        self.goal_point_creator_step = self.config.getfloat("reward_calculation", "goal_point_creator_step")
        self.distance_reward_factor = self.config.getfloat("reward_calculation", "distance_reward_factor")
        self.episode_timeout = self.config.getfloat("reward_calculation", "episode_timeout")

        # goal point list
        self.gpl_index = self.config.getint("gpl", "gpl_index")
        self.gpl_use_random_orientation = self.config.getboolean("gpl", "gpl_use_random_orientation")
        self.gpl_use_random_goal = self.config.getboolean("gpl", "gpl_use_random_goal")

        # logging and visualization
        self.visu_debug_infos = self.config.getboolean("visualization", "visu_debug_infos")
        self.visu_show_step_data = self.config.getboolean("visualization", "visu_show_step_data")
        self.experiment_ID = self.config.getfloat("logging", "experiment_ID")
        self.log_path = self.config.get("logging", "log_path")

        self.init = True

        return True

    def print_parameter(self):

        print("#############Hyperparameter#####################")
        print("State dimension: " + str(self.state_dim))
        print("Action dimension: " + str(self.action_dim))
        print("Control Rate: " + str(self.control_rate))

        print("#############ExplorationNoise#####################")
        print("EXPLORE: " + str(self.EXPLORE))
        print("OU mu: " + str(self.OU_mu))
        print("OU theta: " + str(self.OU_theta))
        print("OU sigma: " + str(self.OU_sigma))
        print("epsilon: " + str(self.epsilon))
        print("use_joystick: " + str(self.use_joystick))
        print("use_ANN: " + str(self.use_ANN))

        print("##############Environment#####################")
        print("state_space_version: " + str(self.state_space_version))
        print("simulation: " + str(self.simulation))

        print("##############Robot parameter#####################")
        print("wheel_diameter: " + str(self.wheel_diameter))
        print("axis_length: " + str(self.axis_length))

        print("#############Reward Calculation###################")
        print("reward_version: " + str(self.reward_version))
        print("goal_max_dist: " + str(self.goal_max_dist))
        print("goal_area_x: " + str(self.goal_area_x))
        print("goal_area_y: " + str(self.goal_area_y))
        print("goal_area_angle: " + str(self.goal_area_angle))
        print("goal_point_creator_x_area: " + str(self.goal_point_creator_x_area))
        print("goal_point_creator_y_area: " + str(self.goal_point_creator_y_area))
        print("goal_point_creator_step: " + str(self.goal_point_creator_step))
        print("distance_reward_factor: " + str(self.distance_reward_factor))
        print("episode_timeout: " + str(self.episode_timeout))

        print("#############Goal Point list###################")
        print("gpl_index: " + str(self.gpl_index))
        print("gpl_use_random_orientation: " + str(self.gpl_use_random_orientation))
        print("gpl_use_random_goal: " + str(self.gpl_use_random_goal))

        print("###########Logging and Visualization#################")
        print("visu_debug_infos: " + str(self.visu_debug_infos))
        print("visu_show_step_data: " + str(self.visu_show_step_data))
        print("experiment_ID: " + str(self.experiment_ID))
        print("log_path: " + str(self.log_path))

        print("\n")


